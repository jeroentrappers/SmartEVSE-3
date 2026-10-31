"""Mock SmartEVSE device backend for the testbench dashboard.

Speaks the same stdio JSON protocol the C++ native binary will use once
Phase B extracts the real state machine out of `main.cpp`. For now this
is a faithful-ish model of IEC 61851 + the firmware's outer loop — good
enough to drive the dashboard UI and author the first characterization
scenarios against.

Protocol
--------
stdin:  one JSON command per line, e.g. {"cmd": "tick", "ms": 100}
stdout: one JSON event  per line, e.g. {"t": 123, "kind": "state", ...}
stderr: free-form human-readable logs (dashboard surfaces them as such)

Virtual time is driven only by `tick` commands — the process itself
never sleeps. This keeps tests deterministic and snapshots stable.
"""

from __future__ import annotations

import json
import sys
import threading
from dataclasses import dataclass, field
from typing import Any

# --------------------------------------------------------------- model

STATES = ("A", "B", "C", "C1", "FAULT")
MODES = ("Off", "Normal", "Solar", "Smart")
ERROR_FLAGS = ("RCM_TRIPPED", "TEMP_HIGH", "LESS_6A", "CT_NOCOMM", "EV_NOCOMM")


@dataclass
class Device:
    t_ms: int = 0

    # Inputs (driven by commands)
    pilot_mv: int = 12000
    ev_connected: bool = False
    rcm_latched: bool = False
    temp_c: float = 25.0
    mains_l1_a: float = 0.0
    mains_l2_a: float = 0.0
    mains_l3_a: float = 0.0
    ext_switch: bool = False

    # Configuration
    mode: str = "Smart"
    max_current_a: int = 16
    max_mains_a: int = 25
    min_current_a: int = 6
    nr_phases: int = 3

    # Live state
    state: str = "A"
    contactor_closed: tuple = (False, False, False)
    pwm_duty: int = 0
    error_flags: set = field(default_factory=set)
    charge_current_a: float = 0.0
    override_current_a: float = 0.0

    def effective_target_a(self) -> float:
        if self.mode == "Off":
            return 0.0
        base = self.override_current_a if self.override_current_a > 0 else float(self.max_current_a)
        # Crude headroom calc vs mains: leave 1A margin.
        worst_mains = max(self.mains_l1_a, self.mains_l2_a, self.mains_l3_a)
        headroom = max(0.0, float(self.max_mains_a) - worst_mains - 1.0)
        return max(0.0, min(base, headroom if worst_mains > 0 else base))


class MockDevice:
    """Holds the Device, mutates it from commands, emits events."""

    def __init__(self) -> None:
        self.d = Device()
        self._lock = threading.Lock()
        self._emit("boot", version="mock-0.1.0", prefix="SmartEVSE-12345")
        self._emit_state_snapshot()

    # -- emit helpers --------------------------------------------------

    def _emit(self, kind: str, **payload: Any) -> None:
        msg = {"t": self.d.t_ms, "kind": kind, **payload}
        sys.stdout.write(json.dumps(msg) + "\n")
        sys.stdout.flush()

    def _emit_state_snapshot(self) -> None:
        self._emit(
            "snapshot",
            state=self.d.state,
            mode=self.d.mode,
            pilot_mv=self.d.pilot_mv,
            ev_connected=self.d.ev_connected,
            contactor=list(self.d.contactor_closed),
            pwm_duty=self.d.pwm_duty,
            errors=sorted(self.d.error_flags),
            charge_current_a=self.d.charge_current_a,
            override_current_a=self.d.override_current_a,
            max_current_a=self.d.max_current_a,
            max_mains_a=self.d.max_mains_a,
            temp_c=self.d.temp_c,
            mains=[self.d.mains_l1_a, self.d.mains_l2_a, self.d.mains_l3_a],
        )

    # -- state machine -------------------------------------------------

    def _transition(self, new_state: str, reason: str) -> None:
        if new_state == self.d.state:
            return
        old = self.d.state
        self.d.state = new_state
        self._emit("state", **{"from": old, "to": new_state, "reason": reason})

    def _set_contactor(self, closed: bool) -> None:
        target = (closed, closed and self.d.nr_phases >= 2, closed and self.d.nr_phases >= 3)
        if target != self.d.contactor_closed:
            self.d.contactor_closed = target
            for i, c in enumerate(target, start=1):
                self._emit("contactor", phase=i, closed=c)

    def _set_error(self, flag: str, on: bool) -> None:
        had = flag in self.d.error_flags
        if on and not had:
            self.d.error_flags.add(flag)
            self._emit("error", flag=flag, set=True)
        elif not on and had:
            self.d.error_flags.discard(flag)
            self._emit("error", flag=flag, set=False)

    def _evaluate(self, reason: str) -> None:
        """Recompute outputs from current inputs. Called after every mutation."""
        # Fault latch wins over everything.
        if self.d.rcm_latched or self.d.ext_switch:
            self._set_contactor(False)
            self.d.pwm_duty = 0
            self._transition("FAULT", reason)
            self._emit("pwm", duty=0)
            return

        # Pilot voltage → IEC 61851 state (simplified).
        if not self.d.ev_connected or self.d.pilot_mv >= 11000:
            self._set_contactor(False)
            self.d.charge_current_a = 0.0
            self.d.pwm_duty = 0
            self._emit("pwm", duty=0)
            self._transition("A", reason)
            return

        if 8000 <= self.d.pilot_mv < 11000:
            # Connected, not requesting energy yet.
            self._set_contactor(False)
            target = self.d.effective_target_a()
            # 1 kHz CP PWM: duty% = 0.6 * amps for 6-51 A per IEC 61851
            duty = int(round(target * 0.6)) if target >= 6 else 100
            self.d.pwm_duty = duty
            self._emit("pwm", duty=duty, advertised_a=(duty / 0.6 if duty <= 85 else None))
            self._transition("B", reason)
            self.d.charge_current_a = 0.0
            return

        if 5000 <= self.d.pilot_mv < 8000:
            # EV requesting charge.
            target = self.d.effective_target_a()
            if target >= self.d.min_current_a and self.d.mode != "Off":
                self._set_contactor(True)
                self.d.charge_current_a = target
                self.d.pwm_duty = int(round(target * 0.6))
                self._emit("pwm", duty=self.d.pwm_duty, advertised_a=target)
                self._transition("C", reason)
            else:
                self._set_contactor(False)
                self.d.charge_current_a = 0.0
                self._transition("C1", reason)
            return

        # Anything lower: treat as short / fault.
        self._set_error("CT_NOCOMM", True)
        self._set_contactor(False)
        self._transition("FAULT", reason)

    def _publish_status_mqtt(self) -> None:
        """Simulate the firmware's MQTT periodic publish for the app."""
        prefix = "SmartEVSE-12345"
        pubs = {
            "State": self.d.state,
            "Mode": self.d.mode,
            "ChargeCurrent": str(int(self.d.charge_current_a * 10)),
            "ChargeCurrentOverride": str(int(self.d.override_current_a * 10)),
            "MaxCurrent": str(self.d.max_current_a * 10),
            "NrOfPhases": str(self.d.nr_phases),
            "Error": ",".join(sorted(self.d.error_flags)) or "None",
            "MainsCurrentL1": str(int(self.d.mains_l1_a * 10)),
            "MainsCurrentL2": str(int(self.d.mains_l2_a * 10)),
            "MainsCurrentL3": str(int(self.d.mains_l3_a * 10)),
        }
        for k, v in pubs.items():
            self._emit("mqtt_publish", topic=f"{prefix}/{k}", payload=v, retain=True)

    # -- command dispatch ----------------------------------------------

    def handle(self, cmd: dict) -> None:
        with self._lock:
            name = cmd.get("cmd")
            handler = getattr(self, f"cmd_{name}", None)
            if handler is None:
                self._emit("log", level="warn", msg=f"unknown command: {name}")
                return
            try:
                handler(cmd)
            except Exception as exc:
                self._emit("log", level="error", msg=f"{name} failed: {exc}")

    # --- individual commands

    def cmd_tick(self, c: dict) -> None:
        ms = int(c.get("ms", 100))
        steps = max(1, ms // 100)
        for _ in range(steps):
            self.d.t_ms += 100
        self._evaluate(reason="tick")
        if self.d.t_ms % 1000 == 0:
            self._publish_status_mqtt()

    def cmd_set_pilot_mv(self, c: dict) -> None:
        self.d.pilot_mv = int(c["mv"])
        self._evaluate(reason="pilot")

    def cmd_plug_ev(self, c: dict) -> None:
        self.d.ev_connected = bool(c["connected"])
        # Plugging changes pilot voltage for convenience.
        if self.d.ev_connected and self.d.pilot_mv >= 11000:
            self.d.pilot_mv = 9000
        if not self.d.ev_connected:
            self.d.pilot_mv = 12000
        self._evaluate(reason="plug")

    def cmd_set_mode(self, c: dict) -> None:
        mode = c["mode"]
        if mode not in MODES:
            self._emit("log", level="warn", msg=f"invalid mode: {mode}")
            return
        self.d.mode = mode
        self._evaluate(reason="mode")
        self._publish_status_mqtt()

    def cmd_rcm_trip(self, c: dict) -> None:
        self.d.rcm_latched = True
        self._set_error("RCM_TRIPPED", True)
        self._evaluate(reason="rcm")

    def cmd_rcm_clear(self, c: dict) -> None:
        self.d.rcm_latched = False
        self._set_error("RCM_TRIPPED", False)
        self._evaluate(reason="rcm_clear")

    def cmd_set_temp_c(self, c: dict) -> None:
        self.d.temp_c = float(c["temp"])
        self._set_error("TEMP_HIGH", self.d.temp_c > 65)
        self._evaluate(reason="temp")

    def cmd_set_mains(self, c: dict) -> None:
        self.d.mains_l1_a = float(c.get("l1", self.d.mains_l1_a))
        self.d.mains_l2_a = float(c.get("l2", self.d.mains_l2_a))
        self.d.mains_l3_a = float(c.get("l3", self.d.mains_l3_a))
        self._evaluate(reason="mains")

    def cmd_mqtt_set(self, c: dict) -> None:
        topic = c["topic"]
        payload = c["payload"]
        self._emit("mqtt_rx", topic=topic, payload=payload)
        # Mimic firmware's dispatch (very simplified).
        if topic.endswith("/Set/Mode"):
            self.cmd_set_mode({"mode": payload})
        elif topic.endswith("/Set/CurrentOverride"):
            try:
                amps = int(payload) / 10
                clamped = max(0.0, min(float(self.d.max_current_a), amps))
                if clamped != amps:
                    self._emit(
                        "log",
                        level="info",
                        msg=f"clamped override {amps}A -> {clamped}A",
                    )
                self.d.override_current_a = clamped
                self._evaluate(reason="mqtt_override")
                self._publish_status_mqtt()
            except ValueError:
                self._emit("log", level="warn", msg=f"invalid override: {payload}")

    def cmd_ext_switch(self, c: dict) -> None:
        self.d.ext_switch = bool(c["on"])
        self._evaluate(reason="ext_switch")

    def cmd_reset(self, c: dict) -> None:
        self.d = Device()
        self._emit("reset")
        self._emit_state_snapshot()
        self._evaluate(reason="reset")

    def cmd_snapshot(self, c: dict) -> None:
        self._emit_state_snapshot()


def main() -> None:
    dev = MockDevice()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            cmd = json.loads(line)
        except json.JSONDecodeError as exc:
            dev._emit("log", level="error", msg=f"bad json: {exc}")
            continue
        dev.handle(cmd)


if __name__ == "__main__":
    main()
