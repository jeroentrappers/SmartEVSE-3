# Host simulation & behavior test strategy

**Goal.** Run the SmartEVSE-3 firmware logic on a developer laptop, drive
it through scripted scenarios, capture its exact behavior today, and
assert that behavior does not change as we refactor. Safety-critical
items (contactor, RCM, overcurrent, state transitions, input clamping)
must have a characterization test locked in before the first refactor PR.

**Non-goals.**
- Cycle-accurate ESP32 emulation.
- Replacing the existing MQTT-side simulator under `simulator/` — that
  remains useful for app-facing testing. This effort targets firmware
  behavior equivalence.
- Running the full binary image (QEMU) on day one. Revisited in Phase 4.

---

## 1. Levels of simulation

Three layers, each with a different cost/fidelity tradeoff. We build
all three, but start at the top.

| Layer | What runs | Fidelity | Cost to stand up |
|---|---|---|---|
| **L1 — Logic simulation** | pure firmware C++ compiled natively, hardware stubbed | High for logic, zero for hardware timing | Low–Medium |
| **L2 — MQTT-contract simulation** | existing Python simulator + real app | Black-box over MQTT | Already done |
| **L3 — Full-image emulation** | actual `.bin` in Espressif QEMU | Highest, slowest, fragile | High |

L1 is where the refactor safety lives. L2 catches app-facing drift. L3
is a later investment for end-to-end pre-release gates.

---

## 2. L1 design — native-compiled firmware with a HAL shim

### 2.1 PlatformIO `env:native`

Add a `native` build env that compiles with the host toolchain (clang or
gcc) and links against a shim library replacing Arduino/ESP-IDF.

```
[env:native]
platform = native
test_framework = unity
build_flags =
  -std=c++17 -Wall -Wextra -Wpedantic
  -DSMARTEVSE_NATIVE=1
  -Isrc -Itest/native/shim
build_src_filter =
  +<state_machine.cpp> +<limits.cpp> +<modbus.cpp>
  +<meter.cpp> +<mqtt_dispatch.cpp>
  +<../test/native/shim/**/*.cpp>
```

`build_src_filter` expands as we extract modules out of `main.cpp` /
`esp32.cpp` / `network_common.cpp` into host-compilable units. Anything
that still drags in `Arduino.h` stays out of the native target until
shimmed.

### 2.2 The HAL shim

`test/native/shim/` provides drop-in headers for the APIs the firmware
actually uses. Each shim is thin — it records calls and forwards to
controllable test fixtures.

```
test/native/shim/
├── Arduino.h                 # pinMode, digitalWrite, millis, Serial
├── Preferences.h             # in-memory NVS
├── WiFi.h                    # no-op
├── freertos/FreeRTOS.h       # xTaskCreate → std::thread OR pumped ticks
├── esp_ota_ops.h             # record-only
├── esp_task_wdt.h            # record-only
├── mbedtls/                  # real mbedTLS from system (keep crypto real)
├── mqtt_client.h             # in-memory broker, routes to TestBus
├── mongoose.h                # minimal server stub + HTTP request injection
├── esp_system.h              # deterministic reset cause, mac, heap
└── hal_record.{h,cpp}        # the global "what happened" recorder
```

**Rule:** the shim never *simulates physics* — it records and forwards.
Physics (pilot-voltage shape, meter readings, EV plug events) is driven
from tests via the harness.

### 2.3 FreeRTOS model

Two modes:

- **Ticked** (default for tests): `xTaskCreate` registers a task but
  doesn't spawn a thread. `hal_tick(ms)` advances the clock and runs the
  tasks whose period has elapsed, in priority order. Deterministic, no
  races.
- **Threaded** (optional, for race hunting): real `std::thread`s, a
  shared spinlock shim for `portMUX_TYPE`. Used later to surface the
  concurrency bugs documented in `ANALYSIS.md §2`.

### 2.4 Recorder (`hal_record`)

One global recorder with a scoped reset per test. Every shim write pushes
onto an append-only event log:

```cpp
struct HalEvent {
  uint64_t t_us;
  enum { GPIO_WRITE, MQTT_PUBLISH, LCD_TEXT, LOG, CONTACTOR, PWM, ... } kind;
  std::string key;   // "PIN_SSR", "ChargeCurrent", ...
  std::string value;
};
```

Tests assert on the event log, not on globals. This is what lets us
refactor internals freely — the contract is "same inputs → same event
sequence."

---

## 3. Harness API (`DeviceUnderTest`)

One C++ class tests drive. Everything is deterministic, nothing blocks.

```cpp
DeviceUnderTest dut;
dut.boot();                                // runs setup() on the shim
dut.set_config({ .max_current = 16, .mode = Mode::Smart, .nr_phases = 3 });
dut.tick(50ms);                            // advance virtual time

dut.plug_ev();                             // pilot 12V → 9V
dut.tick(200ms);                           // let state machine settle
dut.inject_meter(MainsL1{10.0}, MainsL2{9.8}, MainsL3{9.9});
dut.mqtt_receive("Set/CurrentOverride", "120");
dut.tick(5s);

EXPECT_EQ(dut.contactor_closed(), true);
EXPECT_EQ(dut.state(), State::Charging);
EXPECT_THAT(dut.mqtt_publishes(),
            Contains(Event("ChargeCurrent", "120")));
```

Inputs the harness supports (matching real-hardware inputs):

- **Pilot voltage** (mV, mapped through the CP ADC path)
- **Contactor feedback** (closed/open, per phase if applicable)
- **RCM pulse** (current-sense differential tripped)
- **Temperature** (from CH32)
- **Mains meter** (per-phase current/voltage/power, via Modbus or API)
- **EV meter** (same)
- **RFID scan** (UID bytes)
- **Button presses** (short / long / external switch)
- **MQTT `Set/*` commands** (any topic)
- **HTTP form submissions** (settings page POSTs)
- **OCPP messages** (if OCPP build flag on)
- **CH32 UART tokens** (`"Irms:..."`, `"ExtSwitch:..."`)
- **Clock** (`tick()` advances virtual time; `jump(hours)` for long
  scenarios)

Outputs the harness records:

- GPIO pin states (contactor, LEDs, cable lock)
- PWM duty cycle on CP
- All MQTT publishes (topic + payload + retain + qos)
- All Modbus master writes
- All CH32 UART writes
- LCD frame buffer snapshots (for UI regression)
- Error flag transitions
- State machine transitions with reason codes
- Panic / watchdog / reboot events

---

## 4. Characterization tests — the safety corpus

We commit **snapshot tests** (golden masters) for these scenarios
*before* touching the code. Any behavior change shows as a diff.

Format: a scenario is a `.scenario.yaml` file (inputs + expected event
log) generated once, reviewed by a human, then locked. Regeneration
requires an explicit flag.

### 4.1 Contactor safety corpus (must pass)

1. **Plug in on cold boot, no mains meter** — contactor never closes
   without meter confirmation.
2. **Plug in with mode=Off** — stays in STATE_B, contactor open.
3. **Plug in + start charging + RCM trip mid-charge** — contactor opens
   within N ms; error latches; manual reset required.
4. **Overcurrent on L1 > MaxMains** — charge current reduces, then if
   held, contactor opens; error published.
5. **CH32 UART stops responding mid-charge** — stale temperature handled,
   fail-safe path engages.
6. **EV disconnect during charging** (pilot 9V → 12V) — contactor opens
   before PWM changes; session ends; energy totals frozen correctly.
7. **Contactor feedback mismatch** (commanded closed, feedback open) —
   latched error, no retry loop.
8. **Emergency stop via external switch** — immediate contactor open,
   ignores MQTT/HTTP until reset.

### 4.2 Input-clamping corpus

9. **MQTT `Set/CurrentOverride` = 99999** — clamped to `MaxCurrent*10`.
10. **MQTT `Set/Mode` = "Banana"** — rejected, state unchanged, error
    logged.
11. **HTTP `mqtt_port` = 999999** — form rejected, config unchanged.
12. **Modbus holding register write above `MaxMains`** — clamped or
    rejected depending on register, current behavior captured as-is.
13. **CH32 UART injecting `"Mode:99"`** — rejected.

### 4.3 Mode-transition corpus

14. `Off → Normal`, `Normal → Smart`, `Smart → Solar`, `Solar → Off`
    during active charge — contactor dynamics, current ramp, MQTT
    publishes.
15. **Solar surplus fluctuation** — charge current tracks surplus, stops
    below minimum, resumes above threshold + hysteresis delay.
16. **Load balancing master/slave** — 3 virtual EVSEs, Modbus broadcast,
    current apportionment matches firmware today.

### 4.4 OTA corpus

17. **OTA upload, signature valid** — partition activated, reboot event.
18. **OTA upload, signature invalid** — partition erased, current image
    kept running.
19. **OTA upload, image truncated mid-stream** — partition left clean,
    error surfaced.

### 4.5 OCPP corpus (if `ENABLE_OCPP=1`)

20. **OCPP RemoteStart** arriving in STATE_A / B / C — expected state +
    MQTT + contactor outcome for each.
21. **OCPP pause during charge** — current to zero, contactor stays
    closed (per spec), resumes on unpause.

Each scenario is 30–100 lines of YAML. Collectively they pin the
behaviors we refuse to let drift silently.

### 4.6 Differential tests against real hardware (optional, later)

For a handful of the scenarios above, record the same trace from a
real unit (MQTT capture + GPIO logic analyzer via a test jig). Any diff
between hardware trace and simulator trace is a fidelity bug in the
simulator, not a firmware bug. Goes into a separate `hw-trace/`
directory and runs on a human-triggered CI job.

---

## 5. Phased delivery

Each phase is a mergeable PR series. Exit criteria are concrete.

### Phase A — Scaffolding (2–3 days)

- `test/native/` tree, PIO `env:native`, minimal `Arduino.h` shim,
  `hal_record`, empty `DeviceUnderTest`.
- One trivial test that boots, runs one tick, asserts `millis()`
  advanced. Proves the toolchain compiles on macOS + Linux CI.
- GitHub Actions job `native-test` running on every PR.

Exit: `pio test -e native` green on CI.

### Phase A.5 — Dashboard (2 days)

A small web UI served by the native test binary (or, for bootstrapping,
by a Python sidecar driving a mock backend). Same `DeviceUnderTest`
underneath — clicks in the browser produce exactly the same events as
scripted scenarios.

**Inputs** surfaced as controls: pilot CP voltage (slider + preset
buttons for 12/9/6/3 V), EV plug toggle, contactor feedback per phase,
RCM trip button, temperature, mains/EV meter sliders, RFID scan, mode
select, button presses, MQTT `Set/*` sender, HTTP form replay, CH32
UART injector, **clock controls** (tick 100 ms / 1 s / 1 min, play at
1×/10×/100×, pause).

**Outputs** live-streamed via SSE: big contactor indicator (per phase),
PWM duty + decoded advertised current, LED RGB swatch, cable-lock
indicator, MQTT publish stream, Modbus master writes, CH32 UART writes,
error-flag ribbon, state-transition timeline with reason codes, LCD
frame render, event log with filter.

**The payoff feature:** every click is recorded as a
`(t_virtual, kind, payload)` tuple. One button dumps the session as a
`.scenario.yaml` into `testbench/scenarios/` and it's ready to run as a
characterization test. You break the device by poking → you save the
regression in the same session. That's what turns the dashboard from a
debug toy into the primary authoring tool for the safety corpus.

Protocol between UI and device: plain JSON lines over stdio. Device
binary reads commands on stdin, writes events on stdout. Dashboard is
a thin Python Flask sidecar that subprocesses the binary and bridges
to the browser via SSE + POST. No WebSocket dependency, no C++ HTTP
server to build.

Exit: `python3 dashboard.py` opens `http://localhost:7777` with a
clickable device; one manually-driven scenario can be exported to YAML
and re-run headlessly with identical output.

### Phase B — Extract state machine + input validator (3–5 days)

- Pull `setState()` and its direct dependencies out of `main.cpp` into
  `src/state_machine.cpp` + `state_machine.h`. No behavior change.
- Create `src/limits.cpp` (Phase 2 of ANALYSIS.md) and route
  MQTT/HTTP/Modbus/CH32 inputs through it — again, no behavior change,
  only consolidation.
- First 3 characterization tests green (plug in, plug in + Off,
  CurrentOverride clamp).

Exit: `git diff` shows extraction only; 3 tests green; ESP32 build
byte-identical within float-rounding tolerance.

**Status (B step 1 landed):** `testbench/native/` host-compiles a
`firmware_core` module containing the IEC 61851 pilot decoder and
setState transition rules, transliterated from `main.cpp:794-947` and
`main.cpp:1034-1057`. `build/smartevse-sim` speaks the stdio JSON
protocol and plugs into the dashboard via `--backend`. Scenario 01
(plug in + Smart mode → state C + contactor close) drives through it
cleanly.

**Status (B step 2 landed):** Scenario 01's native event stream is
locked as `testbench/scenarios/01_plug_in_charge.expected.jsonl` (14
events). `testbench/run_scenarios.py` + `make check` diff stdout
against the golden on every run. `firmware_core.{h,cpp}` moved to
`SmartEVSE-3/src/` — the firmware build pulls it in via its normal src
glob, and the testbench's Makefile pulls the same translation unit from
`../../SmartEVSE-3/src`. A flat C shim (`firmware_core_shim.h`) bridges
the namespaced `evse::` API into firmware `main.cpp`, avoiding the
macro/enum collision with `main.h`'s `PILOT_12V` / `STATE_A` /
`MODE_SMART` defines. Firmware `Pilot()` now delegates its threshold
table to the shared `evse_decode_pilot_mv`; `make check` still passes.

**Phase B exit gate — first 3 characterization tests green:**

| # | Scenario | Invariant | Events |
|---|---|---|---|
| 01 | plug in, Smart mode, contactor closes | advertised 16 A, L1+L2 closed within 1 s of pilot 6 V | 14 |
| 02 | plug in with mode=Off, contactor stays open | B→C guard `mode!=OFF && access_on` blocks transition | 9 |
| 07 | MQTT `Set/CurrentOverride 99999` clamps | `in_.override_a` capped at `max_current_a` (16 A) before duty is set | 16 |

All three drive the shared `firmware_core` translation unit and pass
`run_scenarios.py` byte-identically across repeated runs. This
satisfies the Phase B exit criterion from §5 ("First 3 characterization
tests green"). `setState()` / `SetCurrent()` migration and
`limits.cpp` extraction remain — listed below.

**Status (B step 3 landed):** `SetCurrent()` now delegates to a new
`evse::cp_duty_for_current_v3(current_dA, min_current_dA)` in
`firmware_core.cpp`. The helper is a line-for-line transliteration of
the firmware's original v3 branch — deci-amp input, truncating
float-to-uint conversion, 1024-scale output, 100-duty fallback for
out-of-range requests. Divisors are kept as `double` literals (not
`0.6f`), preserving the firmware's original IEEE 754 precision where
`60 / 0.6` truncates to 100 but `60 / 0.6f` would truncate to 99 — a
one-duty-unit drift at the minimum-current boundary that the float
version would have introduced. A one-shot exhaustive sweep over every
`current_dA` in [0, 1000] × `MIN_CURRENT ∈ {5, 6}` produced **0 diffs**
against the original inline algorithm. Firmware release build is
binary-size-neutral (flash 86.1%, 1,523,377 bytes) and native `make
check` still 8/8.

**Status (B step 4 landed):** `testbench/native/test/test_firmware_core.cpp`
is 8 fine-grained unit tests for `decode_pilot`, `pilot_from_cp_mv`,
`duty_for_current`, `current_for_duty`, `cp_duty_for_current_v3`, and
three `Core::tick_10ms` transitions (plug-in charge, mode=Off block,
RCM fault fast-path). It's a plain Makefile target rather than
`pio test -e native` — PIO's test runner wants to link `src/main.cpp`
alongside Unity's main(), which collides; threading that needle takes
more plumbing than the tests are worth. `make check` now runs both
unit tests and the scenario corpus on every invocation.

**Status (B step 5 landed):** `SmartEVSE-3/src/evse_limits.{h,cpp}` owns
the `/Set/CurrentOverride` policy (accept / reject / clear) that used
to live inline in `esp32.cpp:627-635`. The firmware-side call site
calls `evse_limits_decide_current_override` through `firmware_core_shim.h`;
the harness calls `evse::limits::decide_current_override` directly. One
source of truth, exercised from both paths. Five new unit tests
(`test_limits_*`) cover each arm (reject on out-of-range / slave / bad
mode, apply on in-range, clear on zero). Scenario 07 re-recaptured
(silent drop, no mqtt_publish echo — the gap flagged in the B step 4
note is now closed); scenarios 09/10/11 added for the happy path,
slave reject, and Solar-mode reject. `make check` → 13/13 unit tests +
11/11 scenarios green.

(Filename trivia: the module is `evse_limits.h`, not `limits.h`, to
avoid shadowing the C-standard `<limits.h>` that libc++ pulls in via
`<algorithm>` for `CHAR_BIT`. The firmware build adds `src/` as a user
include path, and clang will happily resolve `<limits.h>` against it
first. Learned the hard way — see `FEEDBACK.md` §6.)

**Status (B step 6 landed):** Second MQTT knob migrated to
`evse_limits`. `/Set/CurrentMaxSumMains` now calls
`evse_limits_decide_max_sum_mains` through the shim; dropped the
redundant `RequestedCurrent == 0` dead branch documented in FEEDBACK §4.
The `OverrideAction`/`OverrideDecision` types renamed to generic
`Action`/`Decision` since the three-way accept/reject/clear pattern is
the same for every knob — `value` field is documented per-helper.
Scenarios 12/13/14 pin apply / clear / triple-reject (below-range,
above-range, slave); scenarios also now display `max_sum_mains_a` in
snapshots. `make check` → 15/15 unit tests + 14/14 scenarios green.

Pattern note: this topic is **silent on success** (no `mqtt_publish`
echo) unlike `/Set/CurrentOverride`. That asymmetry is now locked by
scenario 12's golden — a future "consistency" refactor that adds an
echo would visibly fail.

**Status (B step 7 landed):** Third MQTT knob migrated —
`/Set/CPPWMOverride`, the only safety-adjacent knob in the corpus (it
directly drives the pilot PWM output and can force-disconnect the EV).
`decide_cp_pwm_override` takes a signed `int32_t` because `-1` carries
the special "release override" meaning; it accepts exactly
`{-1, 0, 1..1024}` and rejects everything else. The extraction
incidentally fixes a latent bug in the legacy inline code: `pwm < -1`
was accepted and passed to `SetCPDuty(uint32_t)`, reinterpreting `-2`
as `0xFFFFFFFE` and forwarding that to `ledcWrite`. See FEEDBACK §8.
Scenarios 15/16/17/18 lock apply / force-disconnect / release /
silent-reject (including the negative-beyond-minus-one path). The
harness's snapshot now surfaces `cp_pwm_override` (-1 = no override,
0..1024 = active override duty) so any regression to the pilot-locking
path is visible on diff. `make check` → 19/19 unit tests + 18/18
scenarios green.

**Status (B step 8 landed):** Fourth MQTT knob migrated —
`/Set/Mode`, the most externally-visible setpoint in the corpus. Unlike
the previous three knobs (which had a single accept/reject/clear
dispatch), Mode has a richer action space: five accepted payload
strings dispatch to two firmware functions (`setMode` vs `setAccess`),
with Solar also triggering a `setOverrideCurrent(0)` side effect at
the call site. Introduced a dedicated `ModeDecision { ModeAction,
target }` type and `decide_set_mode` helper. Pre-migration the harness
was silently dropping `Pause` (the old string table only listed
Off/Normal/Smart/Solar) and was missing Solar's override-clear; both
are fixed by routing through the helper. Scenarios 19 (Pause cuts
access without touching Mode var) and 20 (Solar clears override)
lock the two arms that the pre-migration harness got wrong. Unit tests
`test_limits_set_mode_*` cover all five accept strings plus prefix /
case-sensitivity rejects. `make check` → 21/21 unit tests + 20/20
scenarios green.

**Status (B step 9 landed):** Fifth MQTT knob migrated —
`/Set/EnableC2`, a dual-path parser (digit `0..4` OR one of five
exact-match strings from `StrEnableC2`). Accepts either path,
otherwise silent reject. Call site in `esp32.cpp` now calls
`evse_limits_decide_enable_c2` through the shim; the `EnableC2_t`
assignment is gated on Apply, matching the legacy contract. Harness
surfaces `enable_c2` in snapshots. While migrating, noticed the legacy
inline code called `request_write_settings()` even on rejected input
— flash-write-on-garbage is MQTT-reachable. Pinned as-is for
bit-identity, logged as FEEDBACK §9 with the one-line fix. Scenarios
21/22/23 pin digit / string / reject arms. `make check` → 25/25 unit
tests + 23/23 scenarios green.

**Status (B step 10 landed):** Sixth MQTT knob migrated —
`/Set/MainsMeter`, the first multi-value payload (`L1:L2:L3` signed
deci-amps). Introduces a new `MeterDecision` return type alongside
the existing `Decision` / `ModeDecision`, and a hand-rolled
`parse_signed_int` in an anonymous namespace so the helper stays free
of `sscanf` / libc dependencies. Admission rules: EM_API-configured
meter + master/standalone + three `:`-separated signed ints, each in
the open interval `(-2000, 2000)` dA. Call site in `esp32.cpp` now
calls `evse_limits_decide_mains_meter` through the shim; the v3
`MainsMeter.Irms[]` + `CalcIsum()` write and the v4 `Serial1.printf`
forward are unchanged, just gated on Apply. Harness gained a
`set_mains_meter_em_api` toggle (default true) so scenarios can
exercise the non-EM_API reject path; snapshot surfaces
`mains_meter_em_api`. Scenarios 24-27 pin accept / reject-slave /
reject-not-EM_API / reject-parse-and-range (incl. the exclusive
±2000 boundary). `make check` → 29/29 unit tests + 27/27 scenarios
green.

**Status (B step 11 landed):** Seventh MQTT knob migrated —
`/Set/EVMeter`, the first *partial-apply* payload. Five fields
(`L1:L2:L3:W:WH`) with three independently-gated arms: currents
(requires all three in `[0, 999]` dA — a single negative field gates
the whole block off, matching the firmware's `&&`-chain), power
(requires `W > -1`), energy (requires `WH > -1`). Caller uses `-1` as
a per-field skip sentinel. Introduces an `EvMeterDecision` struct
(`action` + three `apply_*` bools + values); the C-ABI bridge returns
a bitmask in `out_flags`. Unlike `/Set/MainsMeter`, there is no
`LoadBl` gate — any role feeds. Call site in `esp32.cpp` now
dispatches through `evse_limits_decide_ev_meter`; v3 writes to
`EVMeter.Irms[]`/`PowerMeasured`/`Import_active_energy` and v4
`Serial1.printf` forwards are each gated on their flag bit. Harness
gained a `set_ev_meter_em_api` toggle and a five-field `ev_meter`
block in snapshots; scenarios emit an `ev_meter` event per admission
with the flags bitmask visible. Scenarios 28-31 pin all-fields /
partial-arms / currents-block-off-on-negative / reject-parse-or-role.
`make check` → 32/32 unit tests + 31/31 scenarios green.

**Status (B step 12-13 landed):** Eighth and ninth MQTT knobs
migrated in one batch — `/Set/CableLock` and `/Set/HomeBatteryCurrent`.
Both extractions tightened the legacy accept set (reported in
FEEDBACK §10 and §11) while preserving bit-identity on observable
behavior. CableLock accepts exactly `{"0","1"}`; everything else is
rejected, but the unconditional `request_write_settings()` flash
write is preserved at the call site for bit-identity with the
legacy inline code — confirms FEEDBACK §9's suspicion that unknown
MQTT payloads are a reachable flash-write surface. HomeBatteryCurrent
introduces a new `SignedDecision` type because the value is `int16_t`
(signed — negative = exporting to battery) rather than `uint16_t`;
slaves (`LoadBl ≥ 2`) and non-numeric payloads are rejected, and the
`homeBatteryLastUpdate` timestamp only advances on Apply (tightening:
the legacy inline code refreshed the timestamp on parse-fail too,
so the 60 s staleness guard never fired while garbage was arriving).
Harness now surfaces `cable_lock` + `home_battery_current_dA` in
snapshots and emits a `flash_write` event per CableLock publish
(accept or reject) so scenario 33 pins five flash writes for one
accept + four rejects. Scenarios 32-35 cover the two accept paths
(cable toggle, signed battery current) and the two reject paths
(garbage/empty CableLock that still writes flash, slave/garbage
HomeBatteryCurrent with int16 saturation at ±32767). `make check`
→ 36/36 unit tests + 35/35 scenarios green.

**Status (B step 14 landed):** Tenth MQTT migration — the five
`/Set/Color{Off,Normal,Smart,Solar,Custom}` topics collapse into a
single `decide_color_rgb` helper. Legacy code had five copy-pasted
`sscanf("%d,%d,%d")` + range-check blocks with only the destination
array differing; the extraction replaces them with one admission
policy routed by topic. The hand-rolled parser is strict on
separators (comma only, no whitespace tolerance — tightening from
sscanf's permissive `%d` which skips leading whitespace); trailing
garbage after the third number is still ignored to match sscanf on
the canonical happy path. Harness gained a `color_apply` event per
admission and a five-slot `colors` block in snapshots. Scenarios
36-37 pin the five-topic fan-out (one helper, five destinations) and
the reject paths (out-of-range, negative, too-few-fields, wrong
separator, empty). `make check` → 38/38 unit tests + 37/37 scenarios
green.

**Status (B step 15 landed):** Eleventh and twelfth MQTT migrations —
`/Set/CustomButton` and `/Set/RFID`. `CustomButton` is the simplest
helper in the corpus: the legacy code is a 2-line toggle (`CustomButton
= (payload == "On")`) that always Applies, never Rejects. Lifted
verbatim into `decide_custom_button` for bit-identity, not tightening —
there's no safety or flash-write concern. `RFID` replaces ~50 lines of
inline hex-parse + length-gate + Dallas CRC-8 in `esp32.cpp` with
`decide_rfid_uid`, which trims leading/trailing whitespace, validates
all hex, and accepts length {12, 14} (6-byte iButton-family UID vs.
7-byte non-family UID). The helper computes its own CRC-8 (polynomial
0x8C reversed, init 0); values cross-verified against an independent
Python Dallas CRC-8 implementation. Harness gained `custom_button` +
`rfid_uid` snapshot fields and `custom_button` / `rfid_apply` events.
Scenarios 38-41 pin the toggle, both UID-length accept paths (emitting
`01010203040506BD` and `04A1B2C3D4E5F646` respectively), and the four
reject shapes (non-hex, short, long, empty). `make check` →
42/42 unit tests + 41/41 scenarios green.

**Status (B step 16 landed):** §4.1 safety corpus hardened ahead of
the `setState()` migration. Four new scenarios (42-45) lock edges
the first-eight corpus didn't reach: (42) dual fault (RCM +
TEMP_HIGH set on same tick) — pins both `on_error` edges firing with
exactly one C→C1 transition, guarding against a per-bit fault
evaluator regression; (43) fault clear during B1 does NOT auto-
recover — the error-edge fires (`set=false`) but state stays B1,
pinning that the *state machine* is the latch and `error_flags` is
only an instantaneous signal; (44) B1 requires `PILOT_12V` before a
`PILOT_9V` is honored — dropping pilot 12→9 while in B1 is a no-op
(the `case PILOT_9V` only transitions from STATE_A), guarding
against an over-eager "B1→B for symmetry" refactor; (45) extreme
foldback past `min_current_a` — headroom driven to 0, target drops
to 0, PWM off (1000), advertised 0, but STATE_C and both contactors
stay closed. `make check` → 42/42 unit tests + 45/45 scenarios green.

**Status (B step 17 landed):** First pure-function lift out of
`setState()`'s neighborhood. `Force_Single_Phase_Charging()` — the
decision of 1-phase vs. 3-phase charging based on the C2 contactor
configuration, current mode, and active phase count — moved from
`main.cpp:686` into `evse::force_single_phase_charging()`. It was a
perfect extraction target: pure inputs, pure output, no hardware
side effects, 5 enum branches plus a default fallback. `main.cpp`
now forwards through the `extern "C"` shim (`evse_force_single_
phase_charging`). New `EnableC2Config` enum in `firmware_core.h`
mirrors `main.h::EnableC2_t` so future Core growth can reference it
without pulling in `main.h` (the same reason `firmware_core_shim.h`
exists — see its opening comment). Test covers every combination
including the out-of-range enum → default 3P fallback. `make check`
→ 43/43 unit tests + 45/45 scenarios green. This is the first step
of the `setState()` migration — the broader 10–14 day plan is
documented in the B-step-17 recon; future steps will lift
`CheckSwitchingPhases`, `BalancedState[]`, modem-state timers, and
the LCD/LED observer hooks one piece at a time.

**Status (B step 18 landed):** Second lift out of `setState()`'s
neighborhood. `CheckSwitchingPhases()` — the 1P↔3P switching-decision
function that reads `EnableC2`, `Mode`, `State`,
`Nr_Of_Phases_Charging`, and `Switching_Phases_C2` and writes back
`Switching_Phases_C2` or `Nr_Of_Phases_Charging` plus a `_LOG_A`
line — moved from `main.cpp:547` into
`evse::check_switching_phases()`. Extraction preserves bit-identity
by returning a `PhaseSwitchReason` enum (9 values covering every
branch: `NOOP`, `STAGE_1P`, `ALREADY_1P`, `STAGE_3P`, `ALREADY_3P`,
`SET_1P_IMMEDIATE`, `SET_3P_IMMEDIATE`, `AUTO_STAGE_3P`,
`AUTO_CANCEL`). Caller dispatches the reason to the original two
global writes and the original log lines, so every
`Switching_Phases_C2` transition and every `AUTO …`/`Cancelled!`
`_LOG_A` message stays exactly as the legacy code emitted it.
Design choice: return an enum instead of a decision struct because
three branches share the same write but a *different* log string
(e.g. `PSR_STAGE_3P` vs `PSR_AUTO_STAGE_3P`) — collapsing them to
one flag would lose the log line. Test covers every reason including
the `AUTO+SMART` branch that only logs when a switch is already
pending. `make check` → 44/44 unit tests + 45/45 scenarios green.

**Status (B step 19 landed):** Third lift out of `setState()`'s
neighborhood, and the first one *inside* `setState()`'s switch body.
The STATE_C entry block at `main.cpp:872-898` (15 lines of mixed
pre-adjustment + `Force_Single_Phase_Charging()` call + two-way
write of `Nr_Of_Phases_Charging` / `CONTACTOR2`) collapsed to a
single `evse_decide_state_c_contactor2(...)` dispatch. The lifted
function is a pure composition of the already-extracted
`force_single_phase_charging` with the 3-case `Switching_Phases_C2`
pre-adjustment (`GOING_TO_SWITCH_1P=1` → effective 1,
`GOING_TO_SWITCH_3P=2` → effective 3, `NO_SWITCH=0` → unchanged).
Caller keeps all side effects around the bool: `CONTACTOR1_ON`,
the two-way `CONTACTOR2` write, `SEND_TO_ESP32(Nr_Of_Phases_Charging)`,
`setSolarStopTimer(0)`, `MaxSumMainsTimer=0`, and
`Switching_Phases_C2 := NO_SWITCH`. Test covers both pre-adjustment
branches (pending 1P/3P override current phase count) and the
unusual effective-phase-count fallback (AUTO with effective 0 or 2
→ 3P, matching `force_single_phase_charging`'s "==1 triggers 1P"
rule for AUTO). `make check` → 45/45 unit tests + 45/45 scenarios
green.

**Status (B step 20 landed):** Fourth lift — `setStatePowerUnavailable()`
at `main.cpp:729` compressed from an 8-line if/else chain to a single
helper call. The lifted function `evse::decide_state_power_unavailable`
takes the current `State` and returns the *target* state; the legacy
early-returns (`STATE_A` → nothing, `STATE_C1`/`STATE_B1` → nothing)
are encoded as fixpoints so the caller's "call setState only when
target != current" dispatch reproduces the original control flow
exactly. Unknown states (e.g. the `STATE_MODEM_*` values from
`main.h` which aren't in `evse::State`) fall through to `STATE_B1`,
matching the `else` branch. Test locks both fixpoints and the two
unusual-input paths. Called from 7 error-handler sites in `Timer1S`
(CT_NOCOMM, EV_NOCOMM, TEMP_HIGH, LESS_6A) + MQTT `/Set/PowerUnavailable`
+ `processAllNodeStates`; all now go through the pure helper.
`make check` → 46/46 unit tests + 45/45 scenarios green.

**Status (B step 21 landed):** Fifth lift — the OFF/PAUSE state
transition inside `setAccess()` at `main.cpp:962-966` (the one that
drops STATE_C to STATE_C1 and STATE_B / STATE_MODEM_* to STATE_B1)
collapsed to a single `evse_decide_state_on_access_change(Access,
State)` helper followed by the same conditional setState dispatch
as B step 20 uses. The extraction preserves a subtle legacy
behavior: the guard is `Access == OFF || Access == PAUSE`, not
`Access != ON` — so any future unknown enum value falls through
as a no-op rather than triggering a disconnect. STATE_MODEM_*
values (11..14 from `main_c.h`, not in `evse::State`) are
inlined as uint8_t constants inside the helper and documented
at the declaration. Test exhaustively covers: access==ON fixpoints
for 10 states, OFF and PAUSE mapping for STATE_C / STATE_B /
four modem states, STATE_C1 / STATE_A / STATE_B1 / STATE_D
fixpoints under OFF, and an unknown-access-value no-op.
`make check` → 47/47 unit tests + 45/45 scenarios green.

**Status (B step 22 landed):** Sixth lift — the two CP-disconnect
blocks at `main.cpp:617-637` inside `setMode()` collapsed into a
pair of helpers. `evse_decide_mode_switch_cp_reason(c2, old, new,
nr_phases)` returns a 3-value enum (`NOOP`, `SOLAR_OFF_C2_BOUNDARY`,
`AUTO_SOLAR_EXIT_1P`) capturing *which* block triggers, and
`evse_decide_cp_disconnect_target_on_mode_switch(state)` computes
the narrow state-map (`C→C1`, `B→B1`, else fixpoint). Caller does
the three side effects the legacy code did: setState when target
differs, emit the appropriate `_LOG_A` string, and fall through.
Two 8-line blocks became one 7-line dispatch. Design note: a
narrower state-map than `decide_state_power_unavailable` was
necessary — `setMode` only triggers on `STATE_B` / `STATE_C`
(modem states are fixpoints), whereas `power_unavailable` also
maps modem+D to B1. Reusing the broader helper would have
widened the set of states that get disconnected on a mode switch
— a behavioral change. Test covers: all 3 reasons × all 5 `C2`
values, legacy "same mode = NOOP" boundary behavior, AUTO with
3P already (NOOP), and the state-map's modem-state fixpoint
(11..14 left unchanged). `make check` → 48/48 unit tests + 45/45
scenarios green.

**Status (B step 23 landed):** Seventh lift — the Solar-start
phase-count decision inside `CalcBalancedCurrent()`
(`main.cpp:1186-1205`) extracted into
`evse_decide_solar_start_phase(mode, state, c2, isum_dA,
min_current_a, nr_of_phases_charging)`. Returns a 3-value enum
(`SSPR_NOOP`, `SSPR_STAGE_3P`, `SSPR_STAGE_1P`) mapping to the two
`Switching_Phases_C2` writes the legacy block performs. Gated on
`Mode==SOLAR && State==STATE_B && EnableC2==AUTO`; threshold is
`-Isum_dA >= 30 * MinCurrent_A + 30`, above which three-phase is
preferred. The `_LOG_D("waiting for Solar (B)...")` log stays at
the call site because it fires on the outer `Mode==SOLAR &&
State==STATE_B` gate regardless of `EnableC2`, and `_LOG_D` is
debug-only so not captured in scenario goldens. Only the inner
phase-decision was lifted. 20-line block became a 13-line
dispatch. Test covers: all gating branches (mode, state, all
five C2 values), threshold boundary (exact, ±1 dA), both
"already staged" NOOP cases, positive-Isum import case, and
`MinCurrent` scaling at 6A and 16A plus the `nr_phases==2`
treated-as-not-1/not-3 behavior. `make check` → 49/49 unit tests
+ 45/45 scenarios green.

**Status (B step 24 landed):** Eighth lift — the three stacked
Solar-mode admission guards inside `IsCurrentAvailable()` at
`main.cpp:1052-1065` collapsed into a single helper
`evse_decide_solar_admission(mode, active_evse, isum_dA,
start_current_a, min_current_a, import_current_a,
total_current_dA)`. Returns a 4-value enum (`SAR_ALLOW`,
`SAR_DENY_NO_SURPLUS`, `SAR_DENY_CANT_SPLIT`,
`SAR_DENY_OVER_IMPORT`) matching the three legacy `return 0`
branches plus the fallthrough. Caller emits one `_LOG_D` trace
with the reason code (legacy had three separate traces keyed
off `__LINE__`; debug-only so no golden impact) and returns 0
on any non-ALLOW reason. 14-line stacked-if block became a
6-line dispatch. Design note: mode gate moved inside the
helper (non-Solar → `SAR_ALLOW`) so the caller no longer
needs the outer `if (Mode == MODE_SOLAR)` wrap — subsequent
checks in `IsCurrentAvailable()` still run for all modes as
before. Test covers: non-Solar early-out for all 4 modes, all
three deny branches with their specific inputs, threshold
boundaries (±1 dA on guard 1, `>=` vs `>` on guards 2 and 3),
`StartCurrent=0` edge (threshold collapses to 0), `ActiveEVSE=0`
skipping guard 3, two-EVSE min-slot scaling, and the happy-path
ALLOW case. `make check` → 50/50 unit tests + 45/45 scenarios
green.

**Status (B step 25 landed):** Ninth lift — the remaining three
admission guards inside `IsCurrentAvailable()` (MaxMains,
MaxCircuit, MaxSumMains at `main.cpp:1066-1084`) collapsed into
a single `evse_decide_circuit_admission(mode, active_evse,
baseload_dA, baseload_ev_dA, isum_dA, min_current_a, max_mains_a,
max_circuit_a, max_sum_mains_a, load_balance, ev_meter_type,
phases)`. Returns a 4-value enum (`CAR_ALLOW`,
`CAR_DENY_MAX_MAINS`, `CAR_DENY_MAX_CIRCUIT`,
`CAR_DENY_MAX_SUM_MAINS`). 19-line stack became a 6-line
dispatch at the call site. Gating preserved: MaxMains/MaxSumMains
are Smart/Solar-only; MaxCircuit fires for any Master (`LoadBl==1`)
plus LoadBl==0 with an EV meter in non-Normal mode; MaxSumMains
is also gated on the limit being non-zero. `Phases` stays
precomputed at the call site (needs `Force_Single_Phase_Charging`,
already lifted). Legacy had two `printf("@MSG: …line N…")` lines
plus one commented-out third — caller now emits a single
combined `@MSG` with reason code; strings are not in scenario
goldens (verified via grep on `testbench/scenarios`) so no
bit-identity concern. OCPP-gated guard below the #if block left
in place — it uses `float` and sits under `#if ENABLE_OCPP`, not
a clean lift target. Test covers: all three guards individually
at their threshold boundary (`>` vs `>=`), MaxCircuit gating
across `LoadBl ∈ {0,1,2}` × `ev_meter ∈ {0,1}` × mode, MaxSumMains
`==0` disable and phases=1 vs phases=3 scaling, guard-ordering
precedence with triple-trigger inputs, two-EVSE scaling, and
happy-path ALLOW for both Smart and Normal modes. `make check`
→ 51/51 unit tests + 45/45 scenarios green.

**Status (B step 26 landed):** Tenth lift — the Solar-mode ramp
control block inside `CalcBalancedCurrent()` (`main.cpp:1230-1249`)
extracted into `evse_decide_solar_iset_balanced_delta_dA(active_evse,
idifference_dA, isum_import_dA, phases_last_update_flag)`. Returns
the signed delta (dA) applied to `IsetBalanced`, rather than
mutating it in place. 20-line nested block became a one-line
`IsetBalanced += evse_decide_...(...)` at the call site. Gating
captured inside the helper: zero delta when no active EVSE, when
headroom is non-positive, or when measurements are stale. Ramp
table: surplus bumps by +5 dA (~0.5 A) only when both
`IsumImport < -10` and `idifference > 10`, else +1 dA (~0.1 A);
over-draw deadband of 3 dA, then `-1` (> 3 dA), `-5` (> 10 dA),
or `-IsumImport/2` (> 20 dA). `phases_last_update_flag` crosses
the extern "C" boundary as `uint8_t` (C++ bool doesn't round-trip
cleanly). Test covers: all three gate conditions individually,
both surplus branches (including boundary where either inner
condition fails), every over-draw band plus the `== 20` /
`== 21` boundaries, dead-band edge at `IsumImport == 3 / 4`, and
multi-EVSE gate pass-through (no scaling dependency).
`make check` → 52/52 unit tests + 45/45 scenarios green.

What remains in Phase B:
- Migrate `setState()`. The harness's `Core::tick_10ms` is a
  simplified subset — no modem staging, no `BalancedState[]`, no
  `Nr_Of_Phases_Charging`, no `Switching_Phases_C2`, no LCD/LED side
  effects. Grow Core to parity, then delegate.
- Extract the remaining `esp32.cpp::mqtt_receive_callback` topics
  into `evse_limits` on the same pattern. Twelve sites are migrated
  (`Mode`, `CurrentOverride`, `CurrentMaxSumMains`, `CPPWMOverride`,
  `MainsMeter`, `EVMeter`, `HomeBatteryCurrent`, `CableLock`,
  `EnableC2`, the five `Color*` topics sharing one helper,
  `CustomButton`, and `RFID`). The only remaining inline topic is the
  `MODEM`-gated `RequiredEVCCID`, which is a pure
  `strncpy`/`Serial1.printf`/`request_write_settings` forwarder with
  no admission policy — skipping extraction because there is no
  helper-shaped behavior to lift. (Note: `/Set/MaxCurrent` and
  `/Set/Stop` were on earlier lists but neither exists as an actual
  topic.)
- Optional: repackage the Makefile unit tests as a `pio test -e native`
  env for contributors on a PIO-only toolchain.

### Phase C — Safety corpus (1–2 weeks)

- Shim the Modbus layer, CH32 UART, pilot voltage path, contactor
  feedback.
- Write and lock the full §4.1 and §4.2 corpora.
- Human review of each snapshot before it's committed as golden.

Exit: all §4.1 and §4.2 scenarios are locked. Any unintended change to
`setState()` triggers a snapshot diff in CI.

### Phase D — Extend coverage (1 week)

- §4.3 (modes), §4.4 (OTA), §4.5 (OCPP if applicable).
- Coverage measurement (`lcov` on the native env). Target: 80%+ line
  coverage on `state_machine.cpp`, `limits.cpp`, `mqtt_dispatch.cpp`,
  `ota_verify.cpp`.
- Coverage threshold wired into CI.

Exit: coverage dashboard published per PR; dropping coverage blocks
merge.

### Phase E — QEMU boot gate (later, optional)

- Build a QEMU image of the actual firmware.
- Run a subset of §4.1 through QEMU weekly, nightly, or pre-release.
- Catches bugs that only show up with real ESP-IDF, not the shim.

Exit: weekly green QEMU job; any regression is a release blocker.

---

## 6. Golden-master hygiene

- **Snapshots are committed, reviewed, and diffed like code.** A
  snapshot update PR lists *every* changed line in the description.
- **Regenerating requires `--update-snapshots`.** Not default. Reviewers
  must see a diff, not a flip.
- **Timestamps are virtualized.** No `millis()` in event payloads; the
  harness uses a `tick_counter` instead.
- **Payload stability.** MQTT payloads are canonicalized (sorted JSON
  keys, trailing-newline-stripped) before recording.
- **One scenario, one file.** Not a monolithic `snapshots.yaml`.
- **Scenarios live next to their story.** `test/native/scenarios/
  safety/03_rcm_trip_mid_charge.yaml` beats `test/native/test_main.cpp
  :L418`.

---

## 7. What this buys you

- **Refactor freely.** Any of the Phase-0/1/2/3 work in `ANALYSIS.md`
  lands with a green corpus or a visible, reviewed snapshot diff.
- **Regression net for the cloud OTA path.** Bad merges to
  `validate_sig` break a test, not a fleet.
- **Faster iteration loop.** 200 ms test run vs. 60 s flash-and-reboot.
- **Documentation by example.** Every scenario is an executable spec of
  how the device is *supposed* to behave.

---

## 8. What this costs

Honest estimate:

- Phase A: ~2–3 person-days.
- Phase B: ~3–5 person-days (slower if extraction surfaces hidden
  coupling — likely).
- Phase C: ~1.5 weeks (the shim effort for pilot/Modbus/CH32 is most
  of it).
- Phase D: ~1 week.
- Phase E: ~1 week plus flakiness tax.

Total to a durable net: ~4–6 weeks of focused work. Half of that is
reusable for the Phase 3 decomposition already described in
`ANALYSIS.md`, so it overlaps with work you were going to do anyway.

---

## 9. Recommendation

Start with **Phase A + the first 3 scenarios of Phase C** (plug in /
plug in with Off / CurrentOverride clamp). That's ~4 days and proves
the whole approach end-to-end on real code, without yet asking you to
touch `main.cpp`. If it works, continue. If it hits unexpected
coupling, we've learned cheap.

Ready to start Phase A when you are.
