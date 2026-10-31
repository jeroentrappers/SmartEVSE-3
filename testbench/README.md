# SmartEVSE testbench

Host-side firmware simulation + interactive dashboard + characterization
test harness. This is the thing we refactor *against*: lock behavior
here, then refactor the firmware without breaking app-visible or
hardware-visible contracts.

See the strategy doc: [`../TESTING_STRATEGY.md`](../TESTING_STRATEGY.md).

## What's here today (Phase A.5)

| Path | Status | Purpose |
|---|---|---|
| `dashboard/` | runnable | Flask sidecar + web UI for driving a simulated device interactively |
| `dashboard/mock_device.py` | runnable | Python stand-in for the eventual C++ native binary; speaks the same stdio JSON protocol |
| `scenarios/` | seeded | `.scenario.yaml` files — characterization tests |
| `native/` | runnable | C++ host-compiled firmware core + harness; drop-in backend for the dashboard |

**Today you can:**
- launch the dashboard, click through device inputs, watch live outputs,
- record a session and export it as a `.scenario.yaml`,
- run the headless scenario runner against the mock backend.

**Today you cannot yet:**
- drive the full real firmware code. `native/` ships the pilot decoder
  and setState transition rules transliterated from `main.cpp`
  (same thresholds, same PWM formula), but the firmware side of
  `main.cpp` still has its own copy. Collapsing those into a single
  implementation is the next step of Phase B — see
  [`../TESTING_STRATEGY.md`](../TESTING_STRATEGY.md) §Phase B.

**Stale after the rebase onto dingo35 master (4c3de7a, 2026-10-17).**
Upstream changed these code paths after the extraction, so the firmware
uses upstream's inline code again and the extracted helpers (plus their
scenarios) describe the April 2026 behaviour, not the firmware:
- `evse_decide_circuit_admission` — upstream now checks MaxCircuit against a
  separate CircuitMeter (`Baseload_Circuit`) instead of the EV meter.
- `evse_limits_decide_mains_meter` / `evse_limits_decide_ev_meter` —
  `Set/MainsMeter` now also accepts `L1:L2:L3:W:WH`, with a ±200 A range.
  Scenarios 24–31 pin the old 3-field behaviour.

Upstream also removed all v4 / CH32 code (`df910fb`); the extraction's v4
branches were dropped during the rebase.

## Quick start

```bash
cd testbench/dashboard
uv run --with-requirements requirements.txt python dashboard.py
```

(Install `uv` first if you haven't: <https://docs.astral.sh/uv/>.)

Open <http://localhost:7777>. Toggle "EV plugged in", nudge the pilot
voltage to 6 V, press tick — the contactor should close if mode isn't
Off and the mains meter has headroom.

To drive the host-compiled C++ firmware core instead of the Python
mock (same protocol, same dashboard, real extracted state machine):

```bash
( cd ../native && make )
uv run --with-requirements requirements.txt python dashboard.py --backend ../native/build/smartevse-sim
```

## Protocol

One JSON command per line on stdin, one JSON event per line on stdout.
Deterministic — virtual time only advances on `tick` commands.

### Commands (stdin)

```json
{"cmd": "tick", "ms": 100}
{"cmd": "plug_ev", "connected": true}
{"cmd": "set_pilot_mv", "mv": 6000}
{"cmd": "set_mode", "mode": "Smart"}
{"cmd": "rcm_trip"}
{"cmd": "rcm_clear"}
{"cmd": "set_temp_c", "temp": 45}
{"cmd": "set_mains", "l1": 10.0, "l2": 9.8, "l3": 9.9}
{"cmd": "mqtt_set", "topic": "SmartEVSE-12345/Set/CurrentOverride", "payload": "120"}
{"cmd": "ext_switch", "on": true}
{"cmd": "reset"}
{"cmd": "snapshot"}
```

### Events (stdout)

```json
{"t": 1200, "kind": "state", "from": "B", "to": "C", "reason": "pilot"}
{"t": 1200, "kind": "contactor", "phase": 1, "closed": true}
{"t": 1200, "kind": "mqtt_publish", "topic": "SmartEVSE-12345/State", "payload": "C", "retain": true}
{"t": 1200, "kind": "pwm", "duty": 10, "advertised_a": 16.0}
{"t": 1200, "kind": "error", "flag": "RCM_TRIPPED", "set": true}
{"t": 1200, "kind": "snapshot", ...full state...}
{"t": 1200, "kind": "log", "level": "info", "msg": "…"}
```

Logs on stderr are surfaced in the UI as `log/stderr` events but don't
affect the scenario contract.

## Authoring a characterization scenario

The dashboard is the authoring tool.

1. Click **● rec** in the top right.
2. Poke the device through the scenario you want to pin — plug in, set
   mode, advance time, trip RCM, whatever.
3. Click **■ stop & export** — the browser downloads a
   `scenario-<ts>.scenario.yaml`.
4. Drop it under `testbench/scenarios/`, rename it something descriptive,
   and (eventually) run it in CI.

Scenario files are deliberately human-readable:

```yaml
# tests/safety/03_rcm_trip_mid_charge.scenario.yaml
steps:
  - {"cmd":"set_mode","mode":"Smart"}
  - {"cmd":"plug_ev","connected":true}
  - {"cmd":"set_pilot_mv","mv":6000}
  - {"cmd":"tick","ms":1000}
  - {"cmd":"rcm_trip"}
  - {"cmd":"tick","ms":200}
```

Golden matching is driven by a sibling `.expected.jsonl` file — one
committed event per line in the exact order the native binary emits
them. `run_scenarios.py` runs each `.scenario.yaml` and diffs stdout
against the matching `.expected.jsonl`; any drift fails the run.

```bash
# run the whole corpus against the native binary (must be built first)
(cd testbench/native && make check)

# regenerate the goldens after an intentional behavior change
(cd testbench/native && make update-goldens)
```

Regeneration requires the explicit `--update-goldens` flag (or the
`update-goldens` make target) — reviewers see the diff, not a silent
flip.

## Layout

```
testbench/
├── README.md              ← you are here
├── dashboard/
│   ├── dashboard.py       ← Flask sidecar (subprocesses the backend, SSE → browser)
│   ├── mock_device.py     ← stand-in for the real native binary
│   ├── requirements.txt
│   ├── templates/
│   │   └── index.html
│   └── static/
│       ├── app.js
│       └── styles.css
├── scenarios/             ← committed golden scenarios
│   └── 01_plug_in_charge.scenario.yaml
└── native/                ← Phase B: PIO native env + harness + C++ shim
    └── README.md
```

## Status of the safety corpus

| # | Scenario | Status |
|---|---|---|
| 01 | plug in, smart mode, contactor closes | locked (native, 14 events) |
| 02 | plug in with mode=Off, contactor stays open | locked (native, 9 events) |
| 03 | RCM trip mid-charge | locked (native, 24 events) |
| 04 | overcurrent fold-back | locked (native, 15 events) |
| 05 | EV disconnect during charge | locked (native, 19 events) |
| 06 | emergency switch cuts everything | locked (native, 21 events) |
| 07 | MQTT override rejected when out-of-range | locked (native, 15 events) |
| 08 | Invalid MQTT mode is rejected | locked (native, 7 events) |
| 09 | MQTT override applied when in-range | locked (native, 16 events) |
| 10 | MQTT override rejected on slave (LoadBl≥2) | locked (native, 8 events) |
| 11 | MQTT override rejected in Solar mode | locked (native, 8 events) |
| 12 | MQTT CurrentMaxSumMains applied when in-range | locked (native, 9 events) |
| 13 | MQTT CurrentMaxSumMains cleared on "0" | locked (native, 10 events) |
| 14 | MQTT CurrentMaxSumMains rejected (below/above/slave) | locked (native, 11 events) |
| 15 | MQTT CPPWMOverride applied (mid-range 512) | locked (native, 11 events) |
| 16 | MQTT CPPWMOverride forces disconnect (pwm=0) | locked (native, 11 events) |
| 17 | MQTT CPPWMOverride released (pwm=-1) | locked (native, 14 events) |
| 18 | MQTT CPPWMOverride rejected (>1024, pwm<-1) | locked (native, 11 events) |
| 19 | MQTT Mode "Pause" cuts access without touching Mode var | locked (native, 12 events) |
| 20 | MQTT Mode "Solar" clears OverrideCurrent as side effect | locked (native, 12 events) |
| 21 | MQTT EnableC2 digit payload accepted | locked (native, 9 events) |
| 22 | MQTT EnableC2 string payload accepted | locked (native, 8 events) |
| 23 | MQTT EnableC2 rejects (out-of-range, wrong case, empty, negative) | locked (native, 13 events) |
| 24 | MQTT MainsMeter accepted (L1:L2:L3, signed dA) | locked (native, 9 events) |
| 25 | MQTT MainsMeter rejected on slave (LoadBl≥2) | locked (native, 8 events) |
| 26 | MQTT MainsMeter rejected when meter is not EM_API | locked (native, 8 events) |
| 27 | MQTT MainsMeter rejected (parse fail + range boundary) | locked (native, 14 events) |
| 28 | MQTT EVMeter all fields applied (L1:L2:L3:W:WH) | locked (native, 9 events) |
| 29 | MQTT EVMeter partial-apply arms (currents / power / energy) | locked (native, 13 events) |
| 30 | MQTT EVMeter currents-block-off on single negative field | locked (native, 11 events) |
| 31 | MQTT EVMeter rejected (parse fail + not EM_API) | locked (native, 13 events) |
| 32 | MQTT CableLock accept path ("1" locks, "0" unlocks) | locked (native, 11 events) |
| 33 | MQTT CableLock rejects but still writes flash (pins §11) | locked (native, 18 events) |
| 34 | MQTT HomeBatteryCurrent accepts signed dA (incl. int16 saturation) | locked (native, 12 events) |
| 35 | MQTT HomeBatteryCurrent rejects (slave + garbage + empty) | locked (native, 12 events) |
| 36 | MQTT Color* — five topics fan out to five RGB slots | locked (native, 17 events) |
| 37 | MQTT Color* rejects (range / neg / too-few / wrong sep / empty) | locked (native, 15 events) |
| 38 | MQTT CustomButton toggle ("On" ⇒ 1, anything else ⇒ 0) | locked (native, 20 events) |
| 39 | MQTT RFID 12-hex ⇒ 6-byte UID with 0x01 family prefix + CRC | locked (native, 9 events) |
| 40 | MQTT RFID 14-hex ⇒ 7-byte UID + CRC (no family prefix) | locked (native, 9 events) |
| 41 | MQTT RFID rejects (non-hex / odd length / empty) | locked (native, 14 events) |
| 42 | Dual fault (RCM + TEMP_HIGH on same tick) — both edges + single C1 | locked (native, 25 events) |
| 43 | Fault clear during B1 does NOT auto-recover (error-edge only, state latched) | locked (native, 27 events) |
| 44 | B1 requires PILOT_12V (unplug) before 9V→B works; 9V-on-B1 is a no-op | locked (native, 35 events) |
| 45 | Foldback past min_current keeps STATE_C + contactors closed, PWM off | locked (native, 18 events) |

All forty-five Phase-C safety scenarios are locked and green on every
`make check` run. The first three (01, 02, 07) satisfy the Phase B exit
criterion in [`../TESTING_STRATEGY.md`](../TESTING_STRATEGY.md); 03–06
and 08 extend coverage to the full §4.1 and §4.2 safety corpus:
fault→contactor-open (03), mains-driven fold-back (04), unplug mid-charge
(05), operator-initiated shutoff with grace window (06), MQTT payload
rejection (08). Scenarios 09–11 pin the accept / reject-slave /
reject-wrong-mode arms of the MQTT CurrentOverride policy now owned by
the extracted `evse::limits` helper (`SmartEVSE-3/src/evse_limits.h`);
12–14 do the same for the `/Set/CurrentMaxSumMains` policy (apply /
clear / triple-reject), plus pin the "no echo publish" difference
between these two topics; 15–18 cover `/Set/CPPWMOverride` (apply
mid-range duty / force-disconnect / release / silent-reject including
the negative-pwm path that the legacy inline code UB'd on — see
FEEDBACK §8); 19–20 cover the non-obvious arms of `/Set/Mode` that the
pre-migration harness got wrong (Pause was silently dropped; Solar's
side-effect override-clear was missing); 21–23 cover `/Set/EnableC2`
(digit path / string path / rejects including out-of-range, wrong
case, empty, and leading non-digit), surfacing the new `enable_c2`
field in snapshots so a regression to the dual-path parser is visible
on diff; 24–27 cover `/Set/MainsMeter` (accept signed `L1:L2:L3` dA
triple / reject-on-slave / reject-when-meter-is-not-EM_API /
reject-on-parse-or-range — including the open-interval boundaries at
±2000 dA that the legacy inline `sscanf` check pinned); 28–31 cover
`/Set/EVMeter` (five-field `L1:L2:L3:W:WH` with three independently-
gated partial-apply arms — currents / power / energy — including the
non-obvious behavior that a single negative current gates the *whole*
currents block off, not just the negative field); 32–33 cover
`/Set/CableLock` (accept toggle `{"0","1"}` / reject-everything-else
pinned alongside the unconditional `request_write_settings()` flash
write that the legacy inline code emitted on *any* payload — FEEDBACK
§11 confirms the same pattern §9 flagged for EnableC2), with scenario
33 pinning five `flash_write` events for one accept + four rejects so
a future cleanup of the reject-path flash write is visible on diff;
34–35 cover `/Set/HomeBatteryCurrent` (signed `int16_t` dA, positive =
charging from battery, negative = exporting to battery, with int16
saturation at ±32767 / reject on slave or non-numeric — the legacy
code silently wrote 0 on parse fail *and* refreshed the staleness
timestamp, so the 60 s staleness guard never fired while garbage was
arriving; FEEDBACK §10 documents the tightening); 36–37 cover the
five `/Set/Color{Off,Normal,Smart,Solar,Custom}` topics that collapse
to one `decide_color_rgb` helper (five copy-pasted `sscanf` sites in
the legacy inline code, now one admission policy routed by topic),
with scenario 36 proving per-topic fan-out (five `color_apply` events
landing in five distinct snapshot slots) and scenario 37 pinning five
reject shapes (out-of-range, negative, too-few-fields, wrong
separator, empty) — a pre-seeded value survives all five unchanged;
38 pins the legacy `/Set/CustomButton` 2-state toggle — bit-identical
to the original `CustomButton = (payload == "On")` one-liner, so
anything other than the exact string `On` (including `"on"`, `""`,
`"Off"`) flips to 0 — always-Apply, no Reject path; 39–41 cover
`/Set/RFID` (12-hex ⇒ 6-byte UID with 0x01 iButton family prefix at
`uid[0]`; 14-hex ⇒ 7-byte UID with no family prefix; both followed
by a Dallas CRC-8 at `uid[7]` computed with polynomial 0x8C reversed
and init 0, with CRCs cross-verified against an independent Python
Dallas CRC-8 implementation), plus four reject shapes (non-hex char,
odd 11-char length, odd 13-char length, empty) that leave a
pre-seeded UID untouched. Scenarios 42-45 harden the core state
machine ahead of the `setState()` migration with four edge cases
that are not covered by the earlier corpus: 42 sets RCM and
TEMP_HIGH on the *same* tick and pins both error edges firing with
exactly one C→C1 transition (a regression that re-enters C1 per
error bit would diff loudly); 43 trips RCM mid-charge, waits for
C1→B1, then *clears* the RCM bit and pins that the state stays B1
and only an `on_error set=false` edge fires — no automatic recover,
the state machine is the latch; 44 extends 43 by dropping the pilot
from 12 V→9 V while still in B1 (without unplugging first) and pins
that this is a no-op — the `case PILOT_9V` in the pilot switch only
recognizes state A, so recovery requires a full PILOT_12V
pass-through (unplug/replug); 45 pins the extreme overcurrent
foldback path that scenario 04 doesn't cover — when headroom drops
below `min_current_a`, `compute_target_current` returns 0, PWM duty
goes to 1000 and advertised falls to 0, *but the charger stays in
STATE_C with both contactors closed*. A refactor that treats
`advertised_a == 0` as "end session" would break this.

One behavior worth calling out, now locked by the corpus: on a
**fault** (RCM, TEMP_HIGH) the firmware bypasses the 6-second C1 grace
timer and opens contactors within the next 10 ms tick (scenario 03).
On an **operator-initiated** stop (ext_switch off), the same C1 state
honors the grace timer and waits 6 s before opening contactors
(scenario 06). Preserving both of those behaviors through a refactor is
what the corpus is for.

As we move to Phase B and land the real firmware behind the harness,
each of these gets re-recorded against the real code and locked as the
golden contract for the refactor.
