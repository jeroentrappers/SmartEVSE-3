"""SmartEVSE simulator.

Runs three things in a single process:
  * an MQTT client that publishes device telemetry and listens for Set/*
    commands on the broker;
  * an HTTP /settings endpoint matching the firmware's LAN fallback;
  * a small web UI for manually driving the simulated device.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from collections import deque
from typing import Deque, Tuple

import paho.mqtt.client as mqtt
from flask import Flask, jsonify, redirect, render_template, request, url_for

from device import Device, VALID_MODES

LOG = logging.getLogger("simulator")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")


def env(key: str, default: str) -> str:
    v = os.environ.get(key)
    return v if v not in (None, "") else default


DEVICE = Device(
    serial=env("DEVICE_SERIAL", "12345"),
    version=env("DEVICE_VERSION", "v3.13.0-sim"),
    mode=env("INITIAL_MODE", "Smart"),
    max_current_a=int(env("MAX_CURRENT_A", "32")),
    min_current_a=int(env("MIN_CURRENT_A", "6")),
    nr_of_phases=int(env("NR_OF_PHASES", "3")),
)
PREFIX = f"SmartEVSE-{DEVICE.serial}"
TICK = float(env("SIM_TICK_SECONDS", "2"))

MQTT_HOST = env("MQTT_HOST", "localhost")
MQTT_PORT = int(env("MQTT_PORT", "1883"))
MQTT_USERNAME = env("MQTT_USERNAME", "")
MQTT_PASSWORD = env("MQTT_PASSWORD", "")

# Small ring buffer of recent events, surfaced in the web UI.
EVENT_LOG: Deque[Tuple[float, str]] = deque(maxlen=100)
EVENT_LOCK = threading.Lock()


def log_event(msg: str) -> None:
    with EVENT_LOCK:
        EVENT_LOG.appendleft((time.time(), msg))
    LOG.info(msg)


# --------------------------------------------------------------------- MQTT


class MqttBridge:
    def __init__(self) -> None:
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"simulator-{DEVICE.serial}",
        )
        if MQTT_USERNAME:
            self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.will_set(f"{PREFIX}/connected", payload="offline", qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self._last_published: dict[str, str] = {}

    def start(self) -> None:
        LOG.info("Connecting to MQTT %s:%s", MQTT_HOST, MQTT_PORT)
        while True:
            try:
                self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
                break
            except OSError as exc:
                LOG.warning("MQTT connect failed (%s); retrying in 2s", exc)
                time.sleep(2)
        self.client.loop_start()

    # ---- callbacks

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        log_event(f"MQTT connected ({reason_code})")
        client.publish(f"{PREFIX}/connected", "online", qos=1, retain=True)
        # Set/# is the wildcard the firmware itself uses.
        client.subscribe(f"{PREFIX}/Set/#", qos=1)
        client.subscribe(f"{PREFIX}/App/Status", qos=0)
        self._publish_all(force=True)

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace").strip()
        log_event(f"RX {topic} = {payload!r}")

        if topic == f"{PREFIX}/Set/Mode":
            if DEVICE.set_mode(payload):
                log_event(f"Mode -> {payload}")
            else:
                log_event(f"Ignored invalid mode: {payload!r}")

        elif topic == f"{PREFIX}/Set/CurrentOverride":
            try:
                # Firmware expects deciamps; convert to A for the model.
                DEVICE.set_override_current_a(int(payload) / 10)
                log_event(f"CurrentOverride -> {payload} dA")
            except ValueError:
                log_event(f"Ignored non-integer override: {payload!r}")

        elif topic == f"{PREFIX}/App/Status":
            # Informational — app publishes online/offline every 5s.
            pass

        else:
            log_event(f"Unhandled topic {topic}")

        self._publish_all()

    # ---- publishing

    def _publish_all(self, *, force: bool = False) -> None:
        for suffix, value in DEVICE.mqtt_payloads().items():
            topic = f"{PREFIX}/{suffix}"
            if not force and self._last_published.get(topic) == value:
                continue
            self._last_published[topic] = value
            self.client.publish(topic, value, qos=0, retain=True)

    def tick_loop(self) -> None:
        while True:
            DEVICE.tick(TICK)
            self._publish_all()
            time.sleep(TICK)


BRIDGE = MqttBridge()


# --------------------------------------------------------------------- HTTP


app = Flask(__name__, static_folder="static", template_folder="templates")


@app.get("/settings")
def settings_endpoint():
    return jsonify(DEVICE.settings_json())


@app.get("/")
def home():
    with EVENT_LOCK:
        events = [(time.strftime("%H:%M:%S", time.localtime(t)), m) for t, m in EVENT_LOG]
    return render_template(
        "index.html",
        device=DEVICE,
        prefix=PREFIX,
        mqtt_host=MQTT_HOST,
        mqtt_port=MQTT_PORT,
        payloads=DEVICE.mqtt_payloads(),
        events=events,
        modes=VALID_MODES,
    )


@app.get("/state.json")
def state_json():
    with EVENT_LOCK:
        events = [(t, m) for t, m in EVENT_LOG]
    return jsonify(
        {
            "payloads": DEVICE.mqtt_payloads(),
            "settings": DEVICE.settings_json(),
            "events": events,
        }
    )


@app.post("/control/<action>")
def control(action: str):
    if action == "plug":
        DEVICE.plug_in()
        log_event("UI: plug in")
    elif action == "unplug":
        DEVICE.unplug()
        log_event("UI: unplug")
    elif action == "start":
        DEVICE.start_charging()
        log_event("UI: start charging")
    elif action == "stop":
        DEVICE.stop_charging()
        log_event("UI: stop charging")
    elif action == "mode":
        mode = request.form.get("mode", "")
        if DEVICE.set_mode(mode):
            log_event(f"UI: mode -> {mode}")
    elif action == "override":
        try:
            amps = float(request.form.get("amps", "0"))
            DEVICE.set_override_current_a(amps)
            log_event(f"UI: override -> {amps} A")
        except ValueError:
            pass
    elif action == "error":
        err = request.form.get("error", "").strip() or "None"
        DEVICE.set_error(err)
        log_event(f"UI: error -> {err}")
    elif action == "reset":
        DEVICE.reset_error()
        log_event("UI: reset error")
    else:
        return ("unknown action", 400)

    BRIDGE._publish_all()
    return redirect(url_for("home"))


# --------------------------------------------------------------------- main


def main() -> None:
    BRIDGE.start()
    threading.Thread(target=BRIDGE.tick_loop, name="tick", daemon=True).start()
    LOG.info("Simulator ready. Prefix=%s HTTP=:80", PREFIX)
    app.run(host="0.0.0.0", port=80, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
