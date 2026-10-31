# SmartEVSE simulator

A self-contained fake SmartEVSE-3 device for testing the
[SmartEVSE mobile app](../../SmartEVSE-app) without real hardware.

It pretends to be a charging station by:

- running a local MQTT broker (Eclipse Mosquitto);
- publishing the same topics the firmware does on the prefix
  `SmartEVSE-<serial>/…` (see `simulator/device.py`);
- honouring the `Set/Mode` and `Set/CurrentOverride` commands the app sends;
- exposing the same `/settings` JSON the firmware serves over LAN, so the
  app's HTTP fallback works too;
- giving you a small web UI on `http://localhost:8080` to plug/unplug the
  simulated EV, flip mode, trigger errors, and watch live MQTT traffic.

## Quick start

```bash
cd SmartEVSE-3/simulator
cp .env.example .env          # optional — defaults are fine
docker compose up --build
```

When it's up you get:

| URL / endpoint                            | what it is                              |
|-------------------------------------------|-----------------------------------------|
| `http://localhost:8080`                   | control panel + live state              |
| `http://localhost:8080/settings`          | device `/settings` JSON (HTTP fallback) |
| `mqtt://localhost:1883`                   | plain MQTT for the app                  |
| `ws://localhost:9001`                     | MQTT over websockets (handy for mqttx)  |

The simulator comes up in state **Ready to Charge**. Click **Plug in EV** →
**Start charging** in the web UI to see telemetry start flowing.

## Pointing the mobile app at the simulator

The app hard-codes `mqtt.smartevse.nl:8883` (TLS) as the broker
(`SmartEVSE-app/lib/mqtt_service.dart:36-37`). You have two options:

### Option A — run the app in DEV_MODE (no real MQTT, fastest)

Useful for UI work that doesn't need the simulator's behaviour:

```bash
cd SmartEVSE-app
fvm flutter run --dart-define=DEV_MODE=true
```

This bypasses pairing and uses built-in mock data. The simulator is not
involved.

### Option B — point the app at this simulator (real MQTT round-trip)

Edit `SmartEVSE-app/lib/mqtt_service.dart` temporarily:

```diff
- static const String _broker = 'mqtt.smartevse.nl';
- static const int _port = 8883;
+ static const String _broker = '10.0.2.2';   // Android emulator → host
+ static const int _port = 1883;
```

…then in the same file disable TLS and cert checks for the dev broker:

```diff
-   _client!.secure = true;
-   _client!.securityContext = SecurityContext.defaultContext;
+   _client!.secure = false;
```

Broker host per platform:

| Where the app runs        | Host to use     |
|---------------------------|-----------------|
| iOS simulator             | `localhost`     |
| Android emulator          | `10.0.2.2`      |
| Physical phone on same LAN | your Mac's LAN IP (`ipconfig getifaddr en0`) |

You'll also need a **paired device record** in the app. The simulator has
serial `12345` by default. You can either:

- run through the normal pairing flow (the app will write the serial + a
  token into secure storage — the simulator accepts any credentials), or
- keep the app in DEV_MODE first to see the mock device `12345`, then flip
  DEV_MODE off once you've verified connectivity.

> Don't commit those edits — they're dev-only. There is an open item on the
> app side (see `SmartEVSE-app/CLAUDE.md` §4.6) to read the broker from
> `--dart-define`, which will make this a one-flag flip.

## MQTT topic map

Prefix is `SmartEVSE-<DEVICE_SERIAL>` (default `SmartEVSE-12345`).

Published by the simulator (retained):

```
Version                      v3.13.0-sim
Access                       1
ChargeCurrent                160        # deciamps (16.0 A)
ChargeCurrentOverride        0          # deciamps, 0 = no override
Mode                         Smart      # Off | Normal | Solar | Smart
NrOfPhases                   3
State                        Charging   # state name, matches firmware
Error                        None
LoadBl                       0
SolarStopTimer               0          # seconds
MainsCurrentL1 / L2 / L3     160        # deciamps per phase
EVChargePower                11040      # watts
EVEnergyCharged              5400       # watt-hours (session)
EVImportActiveEnergy         5400       # watt-hours (cumulative)
MaxCurrent                   320        # deciamps (32.0 A)
connected                    online     # LWT flips to "offline"
```

Subscribed:

```
Set/Mode              "Off" | "Normal" | "Solar" | "Smart"
Set/CurrentOverride   integer deciamps (e.g. "160" for 16 A, "0" to clear)
App/Status            "online" | "offline" (informational, published by app)
```

## Configuration

Everything is controlled by env vars — see `.env.example`. The commonly
useful ones:

- `DEVICE_SERIAL` — change the simulated serial. The app uses this as the
  topic prefix and as the key in its `stored_devices` list.
- `INITIAL_MODE`, `MAX_CURRENT_A`, `MIN_CURRENT_A`, `NR_OF_PHASES` — tune the
  starting point.
- `SIM_TICK_SECONDS` — how often the physics advances and state is
  re-published (default 2 s).
- `HTTP_PORT` — change if 8080 collides. If you want the app's HTTP fallback
  to reach it without a port number, set this to `80` (requires Docker to
  bind a privileged port).

## Observing traffic

Tail the broker:

```bash
docker compose exec mosquitto \
  mosquitto_sub -h localhost -t 'SmartEVSE-12345/#' -v
```

Send a command by hand:

```bash
docker compose exec mosquitto \
  mosquitto_pub -h localhost -t 'SmartEVSE-12345/Set/Mode' -m 'Solar'
```

Or just use the web UI at `http://localhost:8080` — every MQTT RX is
logged there.

## Layout

```
simulator/
├── docker-compose.yml
├── .env.example
├── mosquitto/config/mosquitto.conf
└── simulator/
    ├── Dockerfile
    ├── requirements.txt
    ├── app.py                # MQTT bridge + HTTP + web UI
    ├── device.py             # simulated device state machine
    └── templates/index.html  # control panel
```

## Not simulated (on purpose)

- Home Assistant MQTT auto-discovery (`homeassistant/...`). The app doesn't
  read those topics (see `mqtt_service.dart`), so they'd only be noise here.
  The firmware does emit them — if you need HA testing, use the real device.
- The SmartEVSE cloud broker's pairing/PIN exchange. The simulator accepts
  any credentials; pairing is out of scope.
- mDNS/nsd discovery. Add the device manually or seed SharedPreferences.
