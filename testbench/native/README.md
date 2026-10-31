# Native harness

Host-compiled test harness for the SmartEVSE firmware. Produces
`build/smartevse-sim`, a binary that speaks the same stdio JSON
protocol as `dashboard/mock_device.py` and can be dropped into the
dashboard with `--backend`.

This is the Phase B foundation from
[`../../TESTING_STRATEGY.md`](../../TESTING_STRATEGY.md): a vehicle for
pinning firmware behavior into characterization tests, so the big
`main.cpp` / `esp32.cpp` refactor can proceed without losing
safety-critical invariants.

## Status (Phase B step 2)

What's extracted and host-compiling today:

- **`SmartEVSE-3/src/firmware_core.{h,cpp}`** — the IEC 61851 pilot
  decoder and setState transition rules, extracted from
  `SmartEVSE-3/src/main.cpp` (`Pilot()` at lines 1034-1057 and
  `setState()` at 794-947). The firmware build picks it up via its
  `+<**/*.cpp>` glob; the native harness pulls the same source file
  via `FW_SRC` in the Makefile. One source of truth.
- **`SmartEVSE-3/src/firmware_core_shim.h`** — flat C wrappers
  (`evse_decode_pilot_mv`) used by firmware `main.cpp`. Necessary
  because `main.h` declares `PILOT_12V` / `STATE_A` / `MODE_SMART`
  as preprocessor macros that would clobber the matching `evse::`
  enum members.
- **`harness/DeviceUnderTest.{h,cpp}`** — wraps `firmware_core` with
  virtual hardware (pilot source, mains meter, RCM latch, MQTT router)
  and translates command structs into Core ticks.
- **`src/main.cpp`** — stdio JSON loop. One command per line on stdin,
  one event per line on stdout. Time only advances on `tick` commands.

Firmware `main.cpp :: Pilot()` now delegates its threshold table to
`evse_decode_pilot_mv`; the inline 3055/2735/2400/… constants live in
one place (`firmware_core.cpp`).

What's **not** here yet (Phase B step 3+):

- `SetCurrent()` still owns its own PWM-duty math. The firmware
  truncates (`uint32_t = current / 0.6`) whereas `duty_for_current`
  rounds. Needs a bit-for-bit helper in `firmware_core.cpp` before
  delegation.
- `setState()` still owns the full transition + side-effect block.
  `firmware_core::Core` implements a simplified subset (no modem,
  balanced, LCD, Nr_Of_Phases_Charging logic). Growing Core to parity
  is the remaining chunk of Phase B.
- Metering, modbus, load balancing, WiFi, MQTT client — all stubbed
  in the harness. Characterization scenarios that exercise those paths
  need more extraction first.

## Build

```bash
make              # produces build/smartevse-sim
make test         # compile & run unit tests on firmware_core helpers
make check        # unit tests + full scenario corpus with golden diff
make clean
```

Or via PlatformIO:

```bash
pio run -e native
# binary lands at .pio/build/native/program (or similar — see pio output)
```

Requires a C++17 compiler. On macOS, Apple clang from Xcode is fine.
On Linux, GCC 7+ or clang 5+.

## Drive it

Scenario-style:

```bash
uv run --with pyyaml python -c '
import yaml, json, subprocess
s = yaml.safe_load(open("../scenarios/01_plug_in_charge.scenario.yaml"))
r = subprocess.run(["./build/smartevse-sim"],
                   input="".join(json.dumps(x)+"\n" for x in s["steps"]),
                   capture_output=True, text=True, timeout=5)
print(r.stdout)'
```

Interactively via the dashboard:

```bash
cd ../dashboard
uv run --with-requirements requirements.txt python dashboard.py \
    --backend ../native/build/smartevse-sim
```

The dashboard UX is identical to driving the mock — same controls,
same event stream — but you're now exercising real C++ code that
shares its state-machine source with the firmware.

## Layout

```
testbench/native/
├── Makefile          ← host-native C++17 build (pulls firmware_core
│                       from ../../SmartEVSE-3/src)
├── platformio.ini    ← equivalent [env:native] for PIO users
├── harness/          ← virtual hardware + event sink
│   ├── DeviceUnderTest.h
│   └── DeviceUnderTest.cpp
├── shim/             ← (empty for now) Arduino/FreeRTOS stubs
│                       — will be populated as more firmware modules
│                       are compiled against it
└── src/
    └── main.cpp      ← JSON protocol loop

SmartEVSE-3/src/       ← firmware source; shared with native build
├── firmware_core.{h,cpp}      ← pilot decoder + state-machine core
└── firmware_core_shim.h       ← flat C API the firmware main.cpp uses
                                  (avoids enum/macro name collision with
                                  main.h's PILOT_12V / STATE_A defines)
```

## Protocol

Identical to [`../README.md`](../README.md), `## Protocol`. Any command
or event documented there is (or will be) supported here. The mock and
the native binary are meant to be interchangeable; divergence is a
bug.

## Re-recording scenarios

Once a behavior path is covered by the native binary, re-record the
matching scenarios against it:

```bash
uv run --with-requirements ../dashboard/requirements.txt python \
    ../dashboard/dashboard.py --backend ./build/smartevse-sim
```

Drive the dashboard through the scenario, export the YAML, commit.
That YAML (plus its captured event stream, once Phase C `expect:`
lands) becomes the golden contract for subsequent refactor work.
