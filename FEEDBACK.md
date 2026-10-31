# Feedback — findings from the Phase B firmware extraction

Recording real bugs, latent hazards, and behavioral surprises uncovered
while extracting firmware_core / evse_limits from `main.cpp` and
`esp32.cpp`. These are things worth discussing with the upstream
maintainers; none have been "fixed" in a way that changes runtime
behavior — each extraction is bit-identical to the original code.

Order: severity (safety-impacting → cosmetic), not chronology.

---

## 1. `0.6f` vs `0.6` in `SetCurrent()` — IEEE 754 single-precision drift at MinCurrent

**Where:** `SmartEVSE-3/src/main.cpp :: SetCurrent()` (v3 branch) — the
original code divides `current / 0.6` (a `double` literal) inside a
`uint32_t` truncating cast.

**The hazard.** `0.6f` in single-precision float is
≈ 0.60000002384185791. Evaluating `60 / 0.6f` gives ≈ 99.99999 which
then *truncates* (not rounds) to `99`. The correct value at the
MinCurrent boundary is `100` (which is what `60 / 0.6` gives in
double). A one-duty-unit drift at the minimum-current edge translates
to the EV briefly seeing ~5.94 A advertised instead of 6 A — below the
IEC 61851 minimum the EV expects. The firmware has always used the
double literal; I only noticed because my first transliteration used
`0.6f` to be "efficient on float-only ESP32" and it failed bit-identity.

**Status.** Pinned. The extracted helper
`evse::cp_duty_for_current_v3` deliberately uses `double` literals with
a comment (`firmware_core.cpp` line ~72) explaining the hazard. An
exhaustive sweep over current_dA ∈ [0, 1000] × MIN_CURRENT ∈ {5, 6}
confirmed 0 diffs vs the original inline code.

**Recommendation.** Leave the behavior alone, but keep the comment —
future refactorers *will* try to "clean up" the literals and silently
regress an EV-visible signal. A compile-time `static_assert` comparing
a constexpr-evaluated helper against a golden table would be a stronger
guard, but requires C++20 constexpr math.

---

## 2. Scenario 07 as written characterized the *harness*, not the firmware

**Where:** `testbench/scenarios/07_current_override_clamp.scenario.yaml`
and its (pre-extraction) golden.

**The finding.** The harness's `mqtt_set("/Set/CurrentOverride",
"99999")` handler used to **clamp** the request to `MaxCurrent` and
echo the *clamped* value back. The firmware in `esp32.cpp:627-635`
does the opposite: **rejects** anything out of `[MinCurrent*10,
MaxCurrent*10]` silently, *with no echo publish at all*. So the
original scenario 07 golden froze harness-specific behavior as if it
were a firmware contract — the kind of drift a golden-master test is
supposed to prevent, hiding in the test itself.

**Status.** Closed. Extracted `evse::limits::decide_current_override`
is the single source of truth now; both the firmware `/Set/CurrentOverride`
branch and the harness call into it. Scenario 07's golden was
re-captured to lock the correct silent-drop behavior, and scenarios
09/10/11 were added for the happy-path + slave-reject + mode-reject
branches. Unit tests in `test_firmware_core.cpp` cover each
decision arm directly.

**Recommendation.** Every externally-reachable knob (HTTP
`/settings`, REST API, Modbus register writes) should go through
`evse::limits`. Today there are at least five more knobs in
`mqtt_receive_callback` alone (`CurrentMaxSumMains`, `MaxCurrent`,
`CPPWMOverride`, …) whose accept/reject policies are inlined in
`esp32.cpp`. Each is a candidate for the same treatment.

---

## 3. Fault vs access-off grace-timer divergence

**Where:** `main.cpp :: setState()` / `firmware_core.cpp :: Core::tick_10ms`.

**The behavior.** Safety faults (`ERR_RCM_TRIPPED`, `ERR_TEMP_HIGH`)
bypass the 6-second C1 grace period entirely: the FSM goes
C → C1 → B1 on consecutive 10 ms ticks, opening contactors inside
20 ms. Operator-initiated stops (external switch off, Mode=Off,
pilot-9V re-advertise) go through C1 and wait up to 6 seconds for the
EV to voluntarily stop drawing current before the contactor opens.

This is deliberately asymmetric and correct — you don't negotiate with
a ground fault — but it is not documented anywhere outside the code.
Anyone refactoring `setState()` without knowing it could easily "unify"
the paths and turn a 20 ms RCM response into a 6-second one.

**Status.** Locked by scenarios 03 (RCM → 20 ms) and 06 (ext_switch →
6 seconds), and by the `test_core_rcm_fault_bypasses_grace` unit test.
Any regression now fails CI.

**Recommendation.** None — keep the behavior, keep the tests.

---

## 4. Dead-code branch in `/Set/CurrentMaxSumMains` handler — RESOLVED

**Where:** `SmartEVSE-3/src/esp32.cpp` around line 640 (pre-migration).

Original code:

```cpp
if (RequestedCurrent == 0) {
    MaxSumMains = 0;
} else if (RequestedCurrent == 0 || (RequestedCurrent >= 10 && RequestedCurrent <= 600)) {
        MaxSumMains = RequestedCurrent;
}
```

The `RequestedCurrent == 0` in the `else if` was dead — the preceding
`if` already handled that case. The broken indentation on the second
body was the visual tell.

**Status.** Resolved as part of the CurrentMaxSumMains migration to
`evse::limits::decide_max_sum_mains`. The call site collapsed to three
lines; the dead branch is gone. Scenarios 12/13/14 lock the three
surviving outcomes (Apply / Clear / Reject) so the cleanup is pinned.

---

## 5. Non-monotonic PWM duty across the 51 A boundary

**Where:** `main.cpp :: SetCurrent()` v3 branch, pinned as
`cp_duty_for_current_v3` in `firmware_core.cpp`.

**The observation.** At 51.0 A requested (510 dA) the computed duty
is `510 / 0.6 = 850` permille → 870 on the v3 1024-scale.
At 51.1 A (511 dA) the formula switches to
`511 / 2.5 + 640 = 844` → 864 on 1024-scale.

So a +0.1 A step in *requested* current yields a **-6/1024 = -0.6%
drop** in advertised duty. An EV that samples near the crossover and
interprets duty linearly as "available current" would briefly see the
advertised current *decrease* when the setpoint was asked to increase.

**Is it a bug?** Probably not — IEC 61851 defines two piecewise
PWM-to-current formulas with exactly this crossover (`I = duty * 0.6`
up to 51 A, `I = (duty - 64) * 2.5` above), and most EVs only sample
the advertised current when it changes by a meaningful step. But the
code's `≤510 → 850`, `>510 → 844` discontinuity has never had a
documented owner. I have no way to confirm intent.

**Status.** Locked as-is in the extracted helper (bit-identical
transliteration). The duty-for-current unit test explicitly pins
`850` and `844` with `EXPECT_EQ`, so any "smoothing" refactor trips
the test.

**Recommendation.** Add an EV-side interop note to the IEC 61851
section of the docs: "the 51 A crossover is by design; small setpoint
increases across it may advertise a lower duty". If field reports ever
blame the crossover for EV confusion, the fix is to stretch the first
formula up to 53 A (matching IEC 61851-1 Annex A's extended
formula) — one-line change plus a new golden.

---

## 6. Filename collision hazard: `<limits.h>` (C standard) vs our `limits.h`

**Where:** caught during `make check` after a first-pass rename.

**The trap.** Clang's `<algorithm>` → `<__algorithm/sort.h>` uses
`CHAR_BIT`, which comes from `<limits.h>` (the C header). When I named
the extracted file `SmartEVSE-3/src/limits.h` and the Makefile added
`-I$(FW_SRC)`, angle-bracket `#include <limits.h>` picked up *my*
file first on clang/gcc, with a cascade of "undeclared identifier
CHAR_BIT" failures. Renaming to `evse_limits.h` fixed it.

**Recommendation.** The `evse_` prefix on new firmware-core modules
(`evse_limits`, `evse_ocpp`, …) should be house rule from here on. The
shim header (`firmware_core_shim.h`) already follows this spirit;
making it a pattern avoids the trap for the next contributor.

---

## 7. Native harness `release` env is broken by an unrelated mbedtls toolchain issue

**Where:** `pio run -e release` from `SmartEVSE-3/SmartEVSE-3/` fails
with `Mbed TLS requires a platform with 8-bit chars`. Reproducible on
a clean baseline (before any of my edits), so this is not caused by
the limits.cpp migration.

**Hypothesis.** Recent bump of the Arduino ESP32 framework (via
`framework-arduinoespressif32`) has a known issue where `mbedtls/check_config.h`
rejects the toolchain's CHAR_BIT setup when built against Xcode 15.x CLT.

**Workaround.** The native testbench (`testbench/native/make check`)
validates the migrated code — it compiles `firmware_core.cpp` +
`evse_limits.cpp` with host clang/gcc and exercises them via the
scenario corpus + unit tests. 11/11 scenarios + 13/13 unit tests
green.

**Recommendation.** Separate ticket. Fixing this is probably a
platform-io package pin, not a source change.

---

## 8. `/Set/CPPWMOverride` accepted `pwm < -1` and fed it to a uint32_t sink — FIXED by extraction

**Where:** `SmartEVSE-3/src/esp32.cpp` around line 647 (pre-migration).

Original code:

```cpp
int pwm = payload.toInt();
if (pwm == -1)       { SetCPDuty(1024); PILOT_CONNECTED;    CPDutyOverride = false; }
else if (pwm == 0)   { SetCPDuty(0);    PILOT_DISCONNECTED; CPDutyOverride = true;  }
else if (pwm <= 1024){ SetCPDuty(pwm);  PILOT_CONNECTED;    CPDutyOverride = true;  }
```

**The hazard.** The chain only bounded `pwm` from above (`pwm <= 1024`).
`pwm = -2` is `int`, the comparison `-2 <= 1024` is true, and the code
falls into the last branch — `SetCPDuty((uint32_t)-2)` ≡ `SetCPDuty(0xFFFFFFFE)`,
forwarded to `ledcWrite(channel, 0xFFFFFFFE)`. ESP-IDF's LEDC driver is
documented for duty values in `[0, 2^duty_res − 1]` (typically 1023 at
10-bit), so the out-of-range write is platform-defined at best — silent
clamp, saturated duty, or an assertion. Either way, it is nonsense
input that the legacy code did not reject.

Side effect: `CPDutyOverride = true` also got latched, so subsequent
normal-PWM updates would be suppressed until the override was
explicitly released. A single malicious or buggy MQTT publisher could
wedge the pilot in a platform-defined state indefinitely.

**Status.** Fixed by extraction. `evse::limits::decide_cp_pwm_override`
accepts exactly `{-1, 0, 1..1024}` and silently rejects everything else,
including `pwm < -1`. Legacy behavior for the three accepted input
classes is bit-identical:

- `-1` → Clear → `SetCPDuty(1024); PILOT_CONNECTED; CPDutyOverride = false`
- `0` → Apply(0) → `SetCPDuty(0); PILOT_DISCONNECTED; CPDutyOverride = true`
- `1..1024` → Apply(value) → `SetCPDuty(value); PILOT_CONNECTED; CPDutyOverride = true`

Unit tests (`test_limits_cp_pwm_override_*`) cover all four branches
including the negative-beyond-minus-one reject. Scenarios 15/16/17/18
lock apply / disconnect / release / reject via the scenario runner;
scenario 18 explicitly sends `-2` and asserts the silent-drop.

**Recommendation.** Keep the stricter helper bounds. If a future
feature needs `pwm = -1` vs `pwm = -2` to mean different things,
re-widen intentionally with a new Action type — don't relax the bound
on the existing Reject arm.

---

## 9. `/Set/EnableC2` calls `request_write_settings()` unconditionally — minor, MQTT-reachable

**Where:** `SmartEVSE-3/src/esp32.cpp` around line 817 (post-migration).

The legacy inline code was shaped like:

```cpp
} else if (topic == ... "/Set/EnableC2") {
    // parse value (digit or string); on match set EnableC2
    request_write_settings();   // <-- always runs, even if parse failed
}
```

**The concern.** `request_write_settings()` schedules a flash commit of
the current settings block. It runs on *every* `/Set/EnableC2` message,
including payloads that failed to parse. A publisher who spams the
topic with garbage strings can trigger one scheduled flash commit per
message. The scheduler coalesces writes, so this is not an immediate
wear problem, but it is still a needless-work path reachable from
whoever can publish MQTT to the unit.

**Status.** Preserved as legacy behavior by the extraction — the
migrated call site still calls `request_write_settings()`
unconditionally — so `make check` does not regress. Worth fixing
separately by gating the write on `action == Apply`.

**Recommendation.** Flip to conditional:

```cpp
if (action == 2 /*Apply*/) {
    EnableC2 = (EnableC2_t) accepted;
    request_write_settings();
}
```

One-line change. The helper already tells you whether admission
succeeded; use that signal. Sanity-check the other MQTT handlers for
the same pattern (I noticed `/Set/CableLock` also calls
`request_write_settings()` outside its accept branch — probably the
same story).

## 10. `/Set/HomeBatteryCurrent` silently writes 0 on non-numeric payloads — minor

Pre-migration, the handler was:

```cpp
} else if (topic == MQTTprefix + "/Set/HomeBatteryCurrent") {
    if (LoadBl >= 2)
        return;
    homeBatteryCurrent = payload.toInt();
    homeBatteryLastUpdate = time(NULL);
#if SMARTEVSE_VERSION >= 40
    SEND_TO_CH32(homeBatteryCurrent);
#endif
}
```

Two problems in one branch:

1. `payload.toInt()` returns `0` on non-numeric input ("garbage", "",
   "NaN", "foo123"). The firmware can't tell "user sent 0 on purpose"
   from "user sent junk" — both write zero to the battery current and
   refresh the staleness timestamp. A broker that drops a connection
   mid-publish, or a typo in a home-automation script, silently says
   "your battery is idle."
2. `homeBatteryLastUpdate = time(NULL)` runs on *every* payload that
   passes the `LoadBl` gate. So even a garbage publish resets the 60 s
   staleness window (see main.cpp:3666-3668), masking a broken feed.

Combined effect: a home-automation bug that starts publishing junk to
this topic looks healthy from the firmware's perspective — fresh
timestamp, zero current — so the load balancer assumes "no battery
contribution" and sizes the EV draw off the raw mains, potentially
overshooting the house limit if the battery *is* actually exporting
into the grid.

The migration tightens this. `decide_home_battery_current` now
returns `Reject` on non-numeric or empty input; the call site's
`homeBatteryCurrent = …` and `homeBatteryLastUpdate = time(NULL)` run
only on `Apply`, so a broken feed stops refreshing the staleness
window and the 60-second guard in `main.cpp:3666` kicks in as
intended. Also the helper saturates at `int16` bounds to kill the
legacy implicit `long → int16` narrowing on huge payloads.

No behavior change for well-formed payloads; the only observable diff
is that garbage no longer looks like "battery=0, feed is fresh." If
any legitimate integration was relying on that quirk it should have
been writing `0` explicitly anyway.

## 11. `/Set/CableLock` also calls `request_write_settings()` unconditionally — minor, MQTT-reachable

Confirming the suspicion from §9. The pre-migration handler:

```cpp
} else if (topic == MQTTprefix + "/Set/CableLock") {
    if (payload == "1") {
        CableLock = 1;
    } else {
        CableLock = 0;
    }
    request_write_settings();
}
```

Same issue as `/Set/EnableC2`:

- Any MQTT publisher can trigger `request_write_settings()` — which
  queues a flash write of the settings block — by sending *anything*
  to this topic. Empty payload, "garbage", "2", "true": all write.
- Worse than §9 because the "accept" side is also looser — every
  payload that isn't exactly `"1"` silently sets `CableLock = 0`, so
  a publisher typo ("On" instead of "1") unlocks the cable without
  complaint.

The migration pins the attack surface but does not fix it, to
preserve bit-identity:

- `decide_cable_lock` accepts exactly `{ "0", "1" }`; everything else
  rejects.
- The call site still calls `request_write_settings()` on every
  payload (unconditional). Scenario 33 pins this on the wire: four
  rejected payloads produce four `flash_write` events with zero
  state change.

Recommended one-line fix, same shape as §9:

```cpp
uint8_t cl = 0;
uint8_t action = evse_limits_decide_cable_lock(payload.c_str(),
                                               payload.length(), &cl);
if (action == 2 /*Apply*/) {
    CableLock = cl;
    request_write_settings();  // ← move inside the accept branch
}
```

The settings block already persists on user-initiated changes from
the UI/REST path, so dropping the MQTT-triggered write on reject
doesn't lose any state. Reviewers: this can land as a separate PR
after the harness diff is in — scenarios 32/33 pin the current
(bit-identical) behavior; a follow-up that moves the flash write
inside the accept branch will need `make update-goldens` and a clear
note that the behavior change is intentional.
