# SmartEVSE-3 firmware — expert review

Scope: `SmartEVSE-3/` (ESP32 + CH32V203 EV charger firmware), ~23k lines of
C++, three PlatformIO targets, implementing IEC 61851 + ISO 15118 + Modbus
+ MQTT + OCPP 1.6j + HTTP/WS + HA discovery. Safety-critical (controls a
contactor delivering up to ~22 kW).

Opinion up front: the project punches well above typical open-source
embedded quality in **protocol breadth** and **firmware-signing
discipline**, but it's weighed down by **giant `.cpp` files**, **no
synchronization primitives around shared state in a multi-task design**,
**no watchdog**, **no unit tests**, and **FIXMEs in safety paths**. The gap
between "works" and "top-notch" here is real but tractable.

---

## 1. Architecture & modularity

**The numbers:**

| File | LOC | Role |
|---|---:|---|
| `main.cpp` | 3720 | state machine + scheduling + globals |
| `esp32.cpp` | 3666 | hardware init + FreeRTOS tasks + GPIO |
| `mongoose.c` | 22600 | vendored HTTP/WS (third-party, OK) |
| `mongoose.h` | 3354 | vendored header (OK) |
| `network_common.cpp` | 2150 | MQTT + HTTP handlers + OTA |
| `glcd.cpp` | 1625 | LCD UI |
| `tcp.cpp` | 1180 | ISO 15118 TCP handler |
| `exi2/` | ~120k | auto-generated EXI codec (OK) |

Two hand-written files above 3500 lines, one above 2000 — each doing
multiple things. `main.cpp` is state machine + task entrypoints + global
config + MQTT glue. `esp32.cpp` is board init + RTOS tasks + Modbus
bridging + OTA download. `network_common.cpp` is MQTT + HTTP + OTA verify
+ WiFi provisioning + HA discovery. Classic monolith that grew by
accretion.

**Global state is extensive.** 60+ `extern` globals declared in `main.h`
(`State`, `Mode`, `ErrorFlags`, `CurrentSet[NR_EVSES]`, `MaxCurrent`,
`MQTTclient`, …). There is no module encapsulation. `main.h` is
effectively a public bus everyone plugs into.

**No HAL layer.** Pin macros + Arduino calls are scattered across
`main.cpp` / `esp32.cpp` / `glcd.cpp`. `SMARTEVSE_VERSION` (30/40) gates
both build-time config and runtime dispatch in the same files. A v5
revision will require edits in many places.

## 2. Concurrency & thread-safety (the real issue)

**Four FreeRTOS tasks** run concurrently on the ESP32 (`Timer10ms`,
`Timer20ms`, `Timer100ms`, `Timer1S`), plus the MQTT/HTTP/mongoose event
loops and an OTA download task. All of them read and write the same set
of globals.

**No synchronization primitives in app code.** Grep confirms: the only
`portMUX_TYPE` reference is `esp32.h:99`, declared `extern` with a comment
"*TODO: Will be placed in the appropriate position after the rtc module is
finished.*" It isn't wired up. No `portENTER_CRITICAL`/`portEXIT_CRITICAL`
on state transitions. No `xSemaphore` guarding `CurrentSet` or
`ErrorFlags`. No `std::atomic`.

On Xtensa ESP32, aligned 32-bit word reads/writes are atomic, so
`uint8_t State` doesn't bit-tear in isolation. But multi-word structures
(the Modbus data buffers, `CurrentSet[]`, strings) and read-modify-write
on `ErrorFlags` from multiple tasks absolutely can interleave. An MQTT
callback setting mode while `Timer20ms` is mid-state-transition can
produce an inconsistent snapshot.

This hasn't caused production incidents (visibly) because the tasks are
periodic, slow, and the hardware itself acts as a low-pass filter. But
it's a latent hazard that invalidates reasoning about any sequence of
operations.

**No watchdog.** `esp_task_wdt_init` / `esp_task_wdt_add` are absent.
ESP-IDF's IWDT triggers at ~5s on brownout, but per-task liveness isn't
monitored. A stuck `Timer20ms` doesn't reset the MCU. In a
contactor-controlling device this is genuinely dangerous — if the
charging-state loop wedges mid-`STATE_C`, the contactor stays closed.

## 3. Safety & error paths

Error detection is via a bit-masked `uint8_t ErrorFlags` (`main.cpp:173`)
with `setErrorFlags()` / `clearErrorFlags()` helpers. Per-error handling
is scattered near the detection site; the central state machine (~174
lines in `setState()`) reacts on the next tick.

- RCM trip: flag is set, but contactor cutoff waits for the state-machine
  tick. No ISR-level cutoff.
- Overcurrent: handled via Modbus meter loop; same latency.
- Temperature: polled from CH32 over UART string tokens (`"Irms:"`,
  `"ExtSwitch:"`). No retry/timeout policy visible at the protocol layer
  — if the CH32 stops responding, the ESP32 sees stale state.

The firmware does implement a state machine cleanly (`setState()`, ~174
LOC, 9 states, `STATE_A..STATE_C1` + modem states). The machine itself is
easy to read. The concern isn't structure — it's the missing **hard
fail-safe path** that bypasses the state machine when safety-critical
conditions fire.

**FIXMEs in safety paths** are especially concerning:

- `tcp.cpp:275` — `//FIXME allow promoting from DIN to ISO2 FOR TESTBENCH
  PURPOSES ONLY!!` (testbench code possibly in production path)
- `tcp.cpp:702` — `//FIXME we have to break this endless loop after
  confirmation of the RFID`
- `esp32.cpp:1421` — `// FIXME this variable never reset to 0?`
  (`ConfigChanged` flag)
- `main.cpp:877` — `//TODO howto ensure this in v4?` (Modbus timing on v4)

40+ TODO/FIXME markers across the tree, some in protocol handlers and
state management. No visible issue-tracker link from these to GitHub
Issues.

## 4. Input validation at trust boundaries

- **MQTT `Set/*` topics** (`network_common.cpp:306+`) dispatch payloads
  into setpoints (current, mode, cable lock, RFID). Bounds enforcement is
  inconsistent across the ladder — some topics clamp, others trust the
  broker.
- **HTTP form handlers** use `.toInt()` on fields like `mqtt_port`
  without range checks; a port of `0` or `999999` is accepted.
- **Modbus RTU** is unauthenticated by protocol. Writable holding
  registers include charge current and mode. Anyone on the RS-485 bus
  owns the charger. This is inherent to Modbus; the firmware could at
  least refuse writes that push current above `MaxCurrent`/`MaxMains`,
  but the check isn't uniform.
- **CH32 UART protocol** is plaintext tokens. Injected UART traffic
  (physical access) can set modes. Low real-world risk, but no defense.

**Rule to apply:** every external input path (MQTT, HTTP, Modbus, CH32
UART) should pass through a single validator that clamps to the
configured `MinCurrent`/`MaxCurrent`/`MaxMains`/`MaxCircuit` envelope.
Today that logic is duplicated and inconsistent.

## 5. Security — corrected read

**OTA signing is real and enforced.** `network_common.cpp:552` defines
`validate_sig()`, which uses mbedTLS `mbedtls_pk_verify` (RSA-PKCS#1 v1.5
/ SHA-256) against a **hard-coded 2048-bit RSA public key** baked into the
binary (`network_common.cpp:554-569`). It's invoked *before* the OTA
partition is committed (`network_common.cpp:779`, `:1597`) and erases the
partition on failure (`:637`). This is genuinely good — better than most
hobbyist projects, and a differentiator. Keep it, document it, protect it.

What's *missing* around it:

- **No ESP32 secure boot** in `platformio.ini`, no flash encryption. A
  local attacker with UART can reflash anything, including replacing the
  public key in the image. The signed-OTA flow only defends against MITM
  on the OTA download.
- **No anti-rollback**. A signed but older (vulnerable) image can be
  downgraded.
- **OTA signing key rotation has no path** — the public key is in source.
  Rotation means a synchronized firmware push, and any unit that misses
  that push can never be updated from the new key.

**Other security observations:**

- **Web UI auth is a 4-digit PIN** (`LCDPin`), plaintext compared, no
  rate-limit, no CSRF tokens. 10,000 guesses is trivial.
- **Firmware upload endpoint (`/update`)** does not require the PIN (per
  reviewer) — worth double-checking, and fixing if confirmed. Even with
  signature verification downstream, uploading a denial-of-service blob
  without auth is bad.
- **AP mode default password** is `"00000000"` / `"12345678"` until the
  portal activates a random one — acceptable once randomization always
  fires, but the default-before-randomize window is a footgun.
- **Cloud MQTT auth** uses the first 32 hex chars of SHA-256 over the EC
  private key stored in NVS (or eFuse BLK1 fallback). NVS is **not**
  encrypted at rest without flash encryption enabled. A compromised
  device leaks the cloud credential.
- **Telnet debug** (RemoteDebug) is on port 23, unauthenticated.

## 6. Supply chain

`platformio.ini`:

- `eModbus@1.7.2` — pinned ✓
- `MicroOcpp#942fb88a...` — commit-pinned ✓
- `ArduinoJson@^6.21.4` — semver-floating ✗
- `MicroOcppMongoose` — **no version**, floats on HEAD ✗
- `espressif32@6.10.0` — pinned ✓

**Vendored Mongoose** (`mongoose.c`, 22.6k LOC) lives in `src/` as a copy.
No version marker at the top. No patch log. Updates mean a manual
re-vendoring ritual.

**Custom mbedTLS** in `custom_libs/esp32{,s3}/` is pre-compiled `.a`
archives with error strings stripped to save flash. The
`replace_mbedtls.py` script overwrites the framework's libs at build
time. **No version string, no source tree, no CVE tracking path.** If a
mbedTLS CVE lands, nobody knows if this build is exposed without
rebuilding and diffing. This is the single scariest supply-chain hole.

## 7. Testing & CI

- `test/` is **five bash scripts** that poke the REST API with curl.
  Useful for smoke testing a live device; not a unit test suite. Nothing
  runs in CI.
- `.github/workflows/` has `pio-build.yaml` (build + artifact sign) and
  `pio-release.yaml`. No `pio test`, no static analysis, no clang-tidy,
  no cppcheck, no coverage, no hardware-in-loop.
- Host-side test of pure logic (state machine, input validation, Modbus
  register decoding) would be straightforward — everything that doesn't
  touch a GPIO can run as a native PlatformIO `test/` target or as a
  GoogleTest-compiled binary. None of that exists.

The existing mobile app has `very_good_analysis` and `--fatal-warnings`
in CI; the firmware, which controls actual hardware, has nothing
comparable.

## 8. Observability in the field

- `RemoteDebug` telnet console exists, but no log-level discipline — mix
  of `_LOG_A`, `_LOG_V`, and raw `Serial.printf`. Sensitive payloads
  (MQTT passwords when logged at startup) can leak under `DBG=1`.
- No structured MQTT log topic (e.g. `.../Log` with severity + code).
  Field diagnosis requires a connection to the device.
- `coredumping.txt` documents opt-in coredump to flash, but coredumps are
  readable over UART with `esptool.py` and contain stack + RAM —
  potential credential exposure if enabled on a returned unit.

## 9. Developer ergonomics

- `main.h` is a ~364-line god-header (configuration defaults + extern
  declarations + pin macros + state names). Editing it recompiles the
  world.
- Three unguarded headers (`ipv6.h`, `qca.h`, `wchisp.h`) — trivial fix,
  but it sits there.
- Dutch comments in one spot (`meter.cpp:27`).
- No `CLAUDE.md` / `CONTRIBUTING.md` / architecture doc. A new
  contributor has to read 3700-line files to figure out the task model.
- No `clang-format` config; style drifts.
- Build scripts (`packfs.py`, `gen_ldscript.py`, `make_ch32.py`,
  `replace_mbedtls.py`) each solve a real problem but have no tests and
  no docstrings describing invariants.

## 10. Genuinely strong things (don't lose these in a rewrite)

- **Signed OTA with on-device RSA verification** — good discipline,
  preserve.
- **Per-board targets** (`release`/`v4`/`ch32`) share code cleanly via
  `SMARTEVSE_VERSION`.
- **Protocol breadth working simultaneously** (Modbus master+slave, MQTT
  + HA discovery + cloud, OCPP 1.6j, ISO 15118) — non-trivial integration.
- **Central constant header** (`main.h:76-155`) — magic numbers are
  mostly named and in one place.
- **State machine is legible** — ~174-line central `setState()` is the
  sort of thing you can still reason about.
- **Build reproducibility** (espressif32 pinned, most libs pinned, custom
  mbedTLS deterministic).

---

# Phased plan

Sized to be mergeable, each with an unambiguous exit gate. Inspired by
the app's phase tracker. Phases 0–2 are "safety net"; 3–4 are "quality
compounding"; 5+ is "polish + forward investment".

### Phase 0 — Safety net (3–5 days)

Stop the bleeding in the places that could hurt someone or lose data.

- **Enable the task watchdog** on the four Timer tasks
  (`esp_task_wdt_add`) with a generous-but-finite timeout. Have the tick
  reset it. Wire the default ESP-IDF panic handler to log, then reboot.
- **Add one `portMUX_TYPE` protecting `ErrorFlags`**, `State`, and the
  `CurrentSet[]` array. Every cross-task RMW goes through critical
  sections. This is ~50 lines; do it now before the next feature.
- **Triage every FIXME** in `tcp.cpp`, `main.cpp`, `esp32.cpp` — each
  either becomes a GitHub Issue with a target milestone or is resolved.
- **Close the `/update` endpoint auth gap** (require PIN even for
  upload; signature verification is a second layer, not a replacement).
- **Document OTA key rotation procedure** in `docs/` so future
  maintainers aren't locked out.

Exit: watchdog rebooting a wedged task is observable in a bench test;
`git grep FIXME src/ | wc -l` halves.

### Phase 1 — Tests that can run in CI (1 week)

You can unit-test a surprising amount of this codebase without hardware.

- Stand up **PlatformIO native unit tests** (`test/native/`). Start with
  what's pure:
  - State machine transitions (`setState` inputs → outputs, expected
    contactor state).
  - Input-validation layer (once it exists — see Phase 2).
  - Modbus register encode/decode.
  - MQTT topic dispatch table.
- Add **`pio test -e native`** to `pio-build.yaml`. Block merges on
  failing tests.
- Add **`cppcheck`** and **`clang-tidy`** runs in CI against `src/`.
  Accept the first-pass diff as the baseline, ratchet down over time.
- Add **`clang-format`** config + CI check.

Exit: CI runs `build + analyze + native test`; a logic change with no
test change is visibly unusual.

### Phase 2 — Extract a trust-boundary validator (3–5 days)

Consolidate all external-input clamping into one module.

- New `src/limits.cpp` / `.h` with `clamp_current()`, `validate_mode()`,
  `clamp_max_current()`, `clamp_max_mains()`.
- All MQTT `Set/*` handlers route through it. All HTTP form handlers
  too. Modbus writable registers too. CH32 UART inputs too.
- Unit-test the validator against the full firmware-configured envelope.
- Remove duplicated clamp logic from callsites.

Exit: a single function — call it `apply_external_setpoint()` — is the
only path to mutating a safety-relevant global. `grep` confirms.

### Phase 3 — Decompose the monoliths (2–3 weeks, iterative)

Land as a *series* of small PRs, not a big-bang rewrite.

- Split **`main.cpp`** into: `state_machine.cpp` (the `setState`
  dispatcher + transitions), `scheduler.cpp` (the Timer tasks + ticks),
  `config.cpp` (NVS load/save). Keep `main.cpp` under 500 LOC —
  entrypoint only.
- Split **`esp32.cpp`** along the same axes: `board_init.cpp`,
  `gpio.cpp`, `ota_download.cpp`, `rfid.cpp`, `ch32_link.cpp`.
- Split **`network_common.cpp`** into: `mqtt.cpp` (plain + HA discovery),
  `mqtt_cloud.cpp`, `http_server.cpp`, `wifi_provisioning.cpp`,
  `ota_verify.cpp`.
- Reduce **`main.h`** to `main.h` + a proper `config.h` + `globals.h` +
  a per-module public header next to each new `.cpp`. Stop editing one
  file to touch three concerns.

Each split PR is mechanical + a couple of targeted tests. Exit: no
hand-written `.cpp` over 1200 LOC; `main.h` under 100 lines.

### Phase 4 — Security hardening (1 week)

With the decomposition done, harden what's now discrete.

- **Enable ESP32 flash encryption + secure boot v2** on new manufacturing
  runs. Existing fleet: document a migration path or accept the gap.
- **Add OTA anti-rollback** via the ESP-IDF anti-rollback feature
  (requires secure boot).
- **Rotate the OTA signing key ritual** — build a documented ceremony
  and a test harness that verifies a released image against the current
  public key.
- **Rate-limit the LCD PIN** endpoint. Lock out after N attempts, unlock
  via physical button press.
- **Strip credentials from debug logs** in `_LOG_A`/`_LOG_V` macros (a
  sanitize() wrapper).

Exit: a device re-flashed via UART with an unsigned image refuses to
boot; brute-forcing the LCD PIN over WiFi is no longer practical.

### Phase 5 — Supply-chain hygiene (2–3 days)

- **Pin `ArduinoJson`** and **`MicroOcppMongoose`** to exact
  versions/commits.
- **Treat `custom_libs/` as vendored source, not binaries** — commit a
  build script that rebuilds the custom mbedTLS from the tagged ESP-IDF
  mbedTLS tree with a visible patch set. Track the base version in a
  `VERSION` file.
- **Version-stamp the vendored `mongoose.c`** at the top of the file;
  add a `UPGRADING.md` describing the re-vendoring procedure.
- **Dependabot** on the repo (at least for GitHub Actions + pub.yaml
  siblings).

Exit: `grep -r VERSION custom_libs/` returns a real string;
`mongoose.c:1` shows a version.

### Phase 6 — Observability (1 week)

- **One log macro family** (`LOG_ERROR` / `LOG_WARN` / `LOG_INFO` /
  `LOG_DEBUG`), with compile-time level filtering and a runtime override.
- **Structured MQTT log stream** on `<prefix>/Log` so field devices can
  be remotely diagnosed without telnet.
- **Expose key health metrics** as MQTT topics (free heap, minimum free
  heap, task high-water marks, uptime, last-reset-reason).

Exit: a wedged device in the field can be diagnosed without physical
access.

### Phase 7 — Forward investments (backlog)

Not blocking quality; worth flagging:

- HAL abstraction for a v5 board.
- OCPP 2.0.1 (MicroOcpp is heading there; plan the upgrade).
- Host-side simulation harness — your own simulator (the one we just
  built) is a good start; an HIL rig would be better.
- Move the EXI auto-generated code to a separate PlatformIO library so
  it stops bloating the src tree.

---

## Priority order

If you do exactly four things, in order:

1. **Phase 0** in its entirety — watchdog + critical sections + auth on
   `/update` are the cheapest wins you will ever get on a safety-critical
   device.
2. **Phase 2** (trust-boundary validator) — tiny scope, compounds every
   other change.
3. **Phase 1** (native unit tests in CI) — turns every subsequent
   refactor from a prayer into a diff.
4. **Phase 5** (supply chain) — because one unpinned dep ruins any story
   you try to tell about the other three.
