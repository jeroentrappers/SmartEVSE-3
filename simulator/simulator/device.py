"""In-memory model of a simulated SmartEVSE device.

Kept deliberately small: one dataclass plus a handful of transition helpers.
All values are stored in the same units the firmware publishes on MQTT
(currents in deciamps, energy in Wh, power in W), so they can be serialised
straight to topic payloads.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict

STATE_NAMES = {
    0: "Ready to Charge",
    1: "Connected to EV",
    2: "Charging",
    9: "Charging Stopped",
    10: "Stop Charging",
}

VALID_MODES = ("Off", "Normal", "Solar", "Smart")


@dataclass
class Device:
    serial: str = "12345"
    version: str = "v3.13.0-sim"

    # Config
    mode: str = "Smart"
    max_current_a: int = 32          # A
    min_current_a: int = 6           # A
    nr_of_phases: int = 3

    # Live state
    state_id: int = 0                # 0 Ready, 1 Connected, 2 Charging, 10 Stop
    error: str = "None"
    access: int = 1
    load_bl: int = 0
    solar_stop_timer: int = 0

    # Currents, in deciamps
    charge_current_da: int = 160
    charge_current_override_da: int = 0
    mains_l1_da: int = 0
    mains_l2_da: int = 0
    mains_l3_da: int = 0

    # EV side
    ev_charge_power_w: int = 0
    ev_energy_charged_wh: int = 0
    ev_import_active_energy_wh: int = 0

    # Session bookkeeping
    session_started_at: float = 0.0

    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    # ---- transitions -----------------------------------------------------

    def state_name(self) -> str:
        return STATE_NAMES.get(self.state_id, f"State {self.state_id}")

    def is_connected(self) -> bool:
        return self.state_id in (1, 2)

    def plug_in(self) -> None:
        with self._lock:
            if self.state_id == 0:
                self.state_id = 1

    def unplug(self) -> None:
        with self._lock:
            self.state_id = 0
            self.ev_charge_power_w = 0
            self.mains_l1_da = 0
            self.mains_l2_da = 0
            self.mains_l3_da = 0
            self.session_started_at = 0.0

    def start_charging(self) -> None:
        with self._lock:
            if self.state_id == 1 and self.mode != "Off":
                self.state_id = 2
                if self.session_started_at == 0.0:
                    self.session_started_at = time.time()
                    self.ev_energy_charged_wh = 0

    def stop_charging(self) -> None:
        with self._lock:
            if self.state_id == 2:
                self.state_id = 1
                self.ev_charge_power_w = 0
                self.mains_l1_da = 0
                self.mains_l2_da = 0
                self.mains_l3_da = 0

    def set_mode(self, mode: str) -> bool:
        if mode not in VALID_MODES:
            return False
        with self._lock:
            self.mode = mode
            if mode == "Off" and self.state_id == 2:
                self.state_id = 1
                self.ev_charge_power_w = 0
        return True

    def set_override_current_a(self, amps: float) -> None:
        with self._lock:
            amps = max(0.0, min(float(self.max_current_a), float(amps)))
            self.charge_current_override_da = int(round(amps * 10))

    def set_error(self, error: str) -> None:
        with self._lock:
            self.error = error or "None"

    def reset_error(self) -> None:
        with self._lock:
            self.error = "None"

    # ---- per-tick physics ------------------------------------------------

    def tick(self, dt_seconds: float) -> None:
        """Advance the simulated physics by ``dt_seconds``."""
        with self._lock:
            # Effective target current: override if set, else max.
            override_a = self.charge_current_override_da / 10
            target_a = override_a if override_a > 0 else float(self.max_current_a)

            if self.mode == "Solar" and self.state_id == 2:
                # Pretend PV output varies gently around 10A.
                target_a = min(target_a, 8 + ((int(time.time()) % 10) * 0.4))

            if self.mode == "Off":
                target_a = 0.0

            self.charge_current_da = int(round(target_a * 10))

            if self.state_id == 2:
                per_phase_da = int(round(target_a * 10))
                self.mains_l1_da = per_phase_da
                self.mains_l2_da = per_phase_da if self.nr_of_phases >= 2 else 0
                self.mains_l3_da = per_phase_da if self.nr_of_phases >= 3 else 0
                # 230 V/phase × amps × phases
                power_w = int(round(target_a * 230 * self.nr_of_phases))
                self.ev_charge_power_w = power_w
                self.ev_energy_charged_wh += int(round(power_w * dt_seconds / 3600))
                self.ev_import_active_energy_wh += int(
                    round(power_w * dt_seconds / 3600)
                )
            else:
                self.mains_l1_da = 0
                self.mains_l2_da = 0
                self.mains_l3_da = 0
                self.ev_charge_power_w = 0

    # ---- serialisation ---------------------------------------------------

    def mqtt_payloads(self) -> Dict[str, str]:
        """Map of `topic_suffix -> payload string`.

        Keys are the parts after the `SmartEVSE-<serial>/` prefix.
        """
        with self._lock:
            return {
                "Version": self.version,
                "Access": str(self.access),
                "ChargeCurrent": str(self.charge_current_da),
                "ChargeCurrentOverride": str(self.charge_current_override_da),
                "Mode": self.mode,
                "NrOfPhases": str(self.nr_of_phases),
                "State": self.state_name(),
                "Error": self.error,
                "LoadBl": str(self.load_bl),
                "SolarStopTimer": str(self.solar_stop_timer),
                "MainsCurrentL1": str(self.mains_l1_da),
                "MainsCurrentL2": str(self.mains_l2_da),
                "MainsCurrentL3": str(self.mains_l3_da),
                "EVChargePower": str(self.ev_charge_power_w),
                "EVEnergyCharged": str(self.ev_energy_charged_wh),
                "EVImportActiveEnergy": str(self.ev_import_active_energy_wh),
                "MaxCurrent": str(self.max_current_a * 10),
            }

    def settings_json(self) -> dict:
        """Mirror of the `/settings` JSON the real firmware serves."""
        with self._lock:
            return {
                "version": self.version,
                "mode": self.mode.lower(),
                "settings": {
                    "charge_current": self.charge_current_da,
                    "current_min": self.min_current_a,
                    "current_max": self.max_current_a,
                    "override_current": self.charge_current_override_da,
                    "mains_meter": "Sensorbox",
                },
                "evse": {
                    "state": self.state_name(),
                    "state_id": self.state_id,
                    "loadbl": self.load_bl,
                    "connected": self.is_connected(),
                    "error": self.error,
                    "nrofphases": self.nr_of_phases if self.state_id == 2 else 0,
                    "solar_stop_timer": self.solar_stop_timer,
                },
                "ev_meter": {
                    "description": "Simulated",
                    "import_active_power": self.ev_charge_power_w,
                    "charged_wh": self.ev_energy_charged_wh,
                },
                "phase_currents": {
                    "L1": self.mains_l1_da,
                    "L2": self.mains_l2_da,
                    "L3": self.mains_l3_da,
                },
            }
