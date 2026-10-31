# Expert review — SmartEVSE simulator

Honest read: the simulator is a solid MVP but ships with a real concurrency
bug, thin test coverage (zero), a brittle app-integration story, and a few
architecture choices that will bite once it stops being a toy. Below is the
diagnosis, then a phased plan.

---

## 1. Correctness & runtime bugs

**1.1 Race on `_last_published` (real bug)** — `simulator/app.py`
`MqttBridge._publish_all` mutates a plain `dict` from three threads: the
paho callback thread (`_on_message`), the tick loop (`tick_loop`), and
Flask's WSGI worker threads (`/control/*`). `dict` ops are *mostly* atomic
in CPython but the `get → compare → set` pattern isn't, and you can
duplicate-publish or lose a diff under contention. Needs a `threading.Lock`
around the diff+publish, or move to a single-writer queue and have the MQTT
thread drain it.

**1.2 Flask handlers reach into `_publish_all()`** — a private, cross-thread
call. Same race, plus tight coupling. The right shape is: mutators post an
event to an in-process queue; the MQTT thread is the sole writer to the
broker and the sole owner of `_last_published`.

**1.3 Connect-retry loop blocks startup forever** — `MqttBridge.start()`
retries `connect()` in a `while True`. If the broker never comes up (e.g.
misconfig), the container runs but is useless and has no unhealthy signal.
Add a max-retry + `SIGTERM` exit, plus a Docker `HEALTHCHECK`.

**1.4 No graceful shutdown** — no `client.loop_stop()` /
`client.disconnect()` on SIGTERM, so the LWT fires instead of publishing a
clean `offline`. Minor but visible as spurious "device disconnected" alerts
in clients.

**1.5 `tick()` holds the lock while doing arithmetic + calls `time.time()`**
— fine now, but `tick` and `mqtt_payloads` both grab the same lock and
`mqtt_payloads` is called from Flask and the tick thread simultaneously.
Low risk of deadlock as written, but any future callback-in-callback will
trip it. Worth switching to a copy-under-lock pattern (lock → snapshot →
release → format).

**1.6 `override_current == 0` semantics** — the app treats `0` as "no
override, fall back to min" (`main.dart:642-644`). The simulator treats `0`
as "unset → use `max_current_a`". That's a subtle diff that changes what
the app shows when the slider is at its default.

## 2. Fidelity vs firmware / app

**2.1 `/settings` schema is reverse-engineered from what the *app* reads,
not what the *firmware* serves** — works today, drifts silently tomorrow.
At minimum, link to the firmware file/line and add a `// from: firmware
X.Y` comment. Better: pull a real response from a firmware unit and check
it into `testdata/settings.json`, then assert the simulator's output matches
the same keys.

**2.2 Retain flags are over-eager** — I `retain=True` every topic; firmware
retains only a subset (e.g. `MainsCurrent*`, `EVChargePower` are *not*
retained per the exploration report). The app happens not to care, but a
future subscriber that assumes "if it was ever published I'll see it on
reconnect" will be misled. Match firmware semantics.

**2.3 Command validation is loose** — `Set/CurrentOverride` doesn't enforce
the firmware's `MinCurrent*10 … MaxCurrent*10` bounds (firmware
`esp32.cpp:627`), and `Set/Mode` doesn't round-trip a rejection path.
Simulator should reject exactly what firmware rejects, otherwise the app
can pass QA against the simulator and fail against hardware.

**2.4 Missing plausibility** — when mode is `Off` the simulator zeroes
charge current, but the real device keeps publishing `ChargeCurrent` with
the *configured* current (it just refuses to close the contactor). Small
thing; matters for UI regression tests.

**2.5 No pairing flow** — the app's pairing is 6-digit PIN → HTTP → MQTT
token. Without it, the only way to get the app into a "paired" state
against the simulator is to seed `SharedPreferences` or run pairing once
against prod. That friction cancels out half the simulator's value.

## 3. App-side integration (the real UX problem)

**3.1 The "patch two lines in `mqtt_service.dart`" workaround is fragile**
— a merge conflict waiting to happen, and every teammate re-discovers it.
The app's own CLAUDE.md §4.6 already mandates `String.fromEnvironment` for
broker config. *This is the highest-leverage fix on the board.* One
~15-line PR on the app unlocks a one-flag workflow:

```bash
fvm flutter run \
  --dart-define=MQTT_BROKER=10.0.2.2 \
  --dart-define=MQTT_PORT=1883 \
  --dart-define=MQTT_TLS=false
```

**3.2 No mDNS announcement** — app discovers via `nsd`. Simulator could
publish an `_http._tcp` record using the `zeroconf` Python package and
suddenly "discover device" works end-to-end.

**3.3 No multi-device mode** — app has a device list screen, simulator
runs one device. `docker compose --profile fleet up` bringing up 3
pre-configured serials would unblock that screen.

## 4. Testability

**4.1 Zero tests.** `device.py` is pure, stateful Python — trivially
unit-testable. The fact that it isn't is the loudest signal the project is
still "demo grade."

**4.2 Globals everywhere** — `DEVICE`, `BRIDGE`, `EVENT_LOG` at module
import time read env and construct state. You can't instantiate two
devices in one process, can't override env per-test, can't inject a fake
MQTT client. Factor into a small composition root:
`build_simulator(config) -> (bridge, app)`.

**4.3 No contract test against the app** — the whole point is "the app
behaves the same as against hardware." Without a scripted Flutter
integration test that drives the app against the simulator, regressions
on either side go undetected.

## 5. Security / hardening

- **Mosquitto is `allow_anonymous true`** — documented, but the web UI has
  no auth and no CSRF protection on POSTs. Fine on `localhost`, landmine
  if exposed on a LAN.
- **Simulator container runs as root** — `Dockerfile` has no `USER` line.
- **No `.dockerignore`** — `.env`, `__pycache__`, git cruft all land in
  the image.
- **Floating base tags** — `eclipse-mosquitto:2`, `python:3.12-slim`.
  Pin by digest for reproducibility.
- **Flask's dev server in prod container** — fine for a sim, but should
  emit a `WARNING` banner so nobody mistakes it for production.

## 6. Observability

- `log_event` writes to a deque *and* to `LOG.info` — two sources of truth
  that will drift.
- No structured logging (JSON lines).
- No `/healthz` / `/readyz`. `docker compose ps` can't tell "broker up,
  MQTT client connected, first publish done."
- No metrics (publishes/sec, commands received, last-tick-age). A 20-line
  Prometheus endpoint pays for itself the first time something looks
  frozen.

## 7. DX

- Auto-refresh via full-page reload every 3 s flashes the UI and resets
  form focus. SSE on `/state.json` is a 30-line upgrade.
- No Makefile / justfile — `docker compose exec mosquitto mosquitto_pub …`
  is a lot to type. `just pub mode Solar` is 100x better.
- README is solid but buries the "here's the exact fvm flutter run command"
  one click deep. Top-of-file copy-paste block wins.
- No CI — no `ruff`, no `mypy`, no `pytest`. The Flutter app has
  `very_good_analysis` enforced; this side has nothing.

## 8. Code quality nits

- `Enum`s for `Mode` and `State`. String typing + magic dicts for state
  names will regress silently.
- `230` V, `3600` s, deque size `100` — name them.
- Inconsistent type hints (some functions annotated, some not — e.g.
  `main()` yes, `env()` partial).
- `MqttBridge._on_message` is a dispatch ladder; a
  `{topic_suffix: handler}` map is clearer and testable.
- `app.py` is 180 lines doing four jobs (MQTT, HTTP, UI, orchestration).
  Split into `mqtt_bridge.py`, `http_api.py`, `wsgi.py`.

---

# Phased plan

Sized like the Flutter app's phase tracker, so they compose. Each phase is
a mergeable chunk with a clear exit criterion.

### Phase 0 — Stop the bleeding (½ day)

Fix the bug, tighten the container, ship the one PR that makes everything
else easier.

- Lock `_last_published`, route all publishes through a single writer
  (MQTT thread).
- Graceful shutdown: SIGTERM handler → publish `offline` explicitly →
  `loop_stop`/`disconnect`.
- `HEALTHCHECK` in compose for both services. `/healthz` in the simulator.
- Add `.dockerignore`, `USER sim` in the Dockerfile, pin base images by
  digest.
- Open a PR against **SmartEVSE-app** implementing `MQTT_BROKER` /
  `MQTT_PORT` / `MQTT_TLS` via `String.fromEnvironment` (their CLAUDE.md
  §4.6 already mandates this). Remove the "patch two lines" workaround
  from the simulator README the same day.

Exit: `docker compose up`, app runs with `--dart-define=MQTT_BROKER=…`,
no file edits, no crashes on Ctrl-C.

### Phase 1 — Fidelity & validation (1–2 days)

Make the simulator honest about what firmware does.

- Capture a real `/settings` JSON from hardware, commit as
  `testdata/settings.json`, diff against simulator output in a test.
- Retention flags per-topic to match firmware (table in `device.py`
  docstring, with firmware file:line citations).
- Enforce `Set/CurrentOverride` bounds and `Set/Mode` enum; log rejections
  in the UI and on MQTT under a `Error` topic as firmware does.
- Fix the override==0 semantics to match app expectation.
- Add the remaining topics the firmware *also* emits but the current app
  doesn't read (Access, RFID, OCPP etc) behind feature flags — they're
  cheap and future-proof the sim.

Exit: a firmware-vs-sim topic diff produces an empty set for the subset
we claim to cover.

### Phase 2 — Testability & CI (1 day)

- Refactor globals into `build_simulator(config)`; make `Device`,
  `MqttBridge`, and the Flask app independently constructable.
- `Enum` for `Mode` and `State`.
- Add `pytest` suite: unit tests for `Device` transitions, a test using
  the paho `loop_forever` in-process against a local Mosquitto container
  (or the `amqtt` in-process broker) that asserts published topics match
  `testdata/`.
- `ruff` + `mypy --strict` locally and in CI.
- GitHub Actions: `build`, `lint`, `test`, `docker build`.

Exit: `pytest` green, `ruff check` clean, `mypy --strict` clean, CI runs
on PR.

### Phase 3 — App integration & pairing (2–3 days)

Close the loop so testing the mobile app end-to-end needs one command.

- Implement a fake pairing endpoint: `POST /pair` with a 6-digit PIN →
  returns `{serial, token}`. Accept any PIN in dev.
- mDNS announcement via `zeroconf` so `nsd` on the app discovers the
  simulator automatically.
- `docker compose --profile fleet up` brings up N serials (param via
  `.env`) for the device-list screen.
- A Flutter integration test harness (`flutter drive`) that launches the
  simulator via `docker compose` as a setup step, pairs, plugs in, charges,
  unplugs — runnable in the app's CI.

Exit: a fresh checkout + `docker compose up` + `fvm flutter run` gets you
to the charging screen without touching any file or seeding storage.

### Phase 4 — Polish & productization (1–2 days)

- SSE on `/state.json`; replace full-page reload.
- Structured JSON logs.
- `justfile` with `up`, `down`, `logs`, `pub`, `tail`, `reset`.
- Prometheus `/metrics`: publish rate, commands received, reconnect count,
  last-tick-age.
- TLS profile: self-signed cert, mosquitto TLS listener on 8883, README
  section for trusting it — for users who want to test the *real* TLS
  path without a local app rebuild.
- LICENSE file (match parent repo).

Exit: a teammate can onboard by reading one README section and running
`just up`.

### Phase 5 — Nice-to-haves (backlog, not blocking)

- Home Assistant auto-discovery topics, so the simulator doubles as an HA
  sandbox.
- Scenario scripts (`scenarios/solar-noon.yaml`) the web UI can load.
- Record/replay: capture a real device's MQTT stream for a session, replay
  it through the simulator for deterministic regression runs.
- OCPP/RFID/modem topic coverage for when the app grows into them.

---

## Recommended order of priority

If you only do two things this week: **Phase 0 + the app-side
`--dart-define` PR**. Everything else compounds off that one change — no
more file patches means teammates actually *use* the simulator, which
means real feedback, which justifies the rest of the plan.
