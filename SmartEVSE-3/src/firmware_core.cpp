// firmware_core.cpp — see firmware_core.h for the source-of-truth map.

#include "firmware_core.h"
#include "firmware_core_shim.h"

#include <algorithm>

namespace evse {

Pilot pilot_from_cp_mv(uint32_t cp_mv) {
  // Tolerance bands around the nominal IEC 61851 CP voltages. The
  // dashboard emits 12000 / 9000 / 6000 / 3000 as round-number knobs;
  // tolerate ±1000 mV so fractional signals still resolve.
  if (cp_mv >= 11000)              return PILOT_12V;
  if (cp_mv >= 8000  && cp_mv < 11000) return PILOT_9V;
  if (cp_mv >= 5000  && cp_mv < 8000)  return PILOT_6V;
  if (cp_mv >= 2000  && cp_mv < 5000)  return PILOT_3V;
  if (cp_mv < 500)                 return PILOT_SHORT;
  return PILOT_NOK;
}

Pilot decode_pilot(uint32_t min_mv, uint32_t max_mv) {
  // main.cpp:1050-1056 (ESP32 v3 thresholds). Order matters: the diode
  // check uses a much narrower Max band so we evaluate it first-equal
  // with the 12V band but separated by Max ceiling.
  if (min_mv >= 3055)                       return PILOT_12V;
  if (min_mv >= 2735 && max_mv < 3055)      return PILOT_9V;
  if (min_mv >= 2400 && max_mv < 2735)      return PILOT_6V;
  if (min_mv >= 2000 && max_mv < 2400)      return PILOT_3V;
  if (min_mv >= 1600 && max_mv < 2000)      return PILOT_SHORT;
  if (min_mv > 100   && max_mv < 300)       return PILOT_DIODE;
  return PILOT_NOK;
}

uint16_t duty_for_current(float amps) {
  // Ported from main.cpp:722-736 SetCurrent(). The firmware passes
  // `current` in deci-amps (A×10); the duty it computes is in permille.
  //   60..510 dA  : duty = current / 0.6
  //   510..800 dA : duty = current / 2.5 + 640
  //   else        : PWM off (duty = 1000, pilot held at +12V)
  // 16 A → 160 dA → 266.7‰ ≈ 27% PWM (advertises 16 A to the EV).
  float dA = amps * 10.0f;
  if (dA < 60.0f) return 1000;
  if (dA <= 510.0f) {
    float d = dA / 0.6f;
    if (d > 1000.0f) d = 1000.0f;
    return static_cast<uint16_t>(d + 0.5f);
  }
  if (dA <= 800.0f) {
    float d = dA / 2.5f + 640.0f;
    if (d > 1000.0f) d = 1000.0f;
    return static_cast<uint16_t>(d + 0.5f);
  }
  return 1000;
}

float current_for_duty(uint16_t duty_permille) {
  // Inverse of duty_for_current — returns amps (not deci-amps).
  if (duty_permille >= 1000) return 0.0f;
  if (duty_permille <= 850)  return duty_permille * 0.06f;            // /10 after ×0.6
  return (duty_permille - 640) * 0.25f;                               // /10 after ×2.5
}

uint8_t force_single_phase_charging(EnableC2Config c2, Mode mode, uint8_t nr_phases_charging) {
  switch (c2) {
    case C2_NOT_PRESENT: return 0;
    case C2_ALWAYS_OFF:  return 1;
    case C2_SOLAR_OFF:   return mode == MODE_SOLAR ? 1 : 0;
    case C2_AUTO:        return nr_phases_charging == 1 ? 1 : 0;
    case C2_ALWAYS_ON:   return 0;
  }
  return 0;
}

PhaseSwitchReason check_switching_phases(EnableC2Config c2,
                                         Mode           mode,
                                         uint8_t        state,
                                         uint8_t        nr_phases_charging,
                                         uint8_t        switching_phases_c2) {
  // Legacy branch guard: obey EnableC2 at all times unless we're in the
  // special case of AUTO+non-solar (which falls through to the SMART arm).
  const bool obey_c2 = (c2 != C2_AUTO) || (mode == MODE_SOLAR);
  if (obey_c2) {
    if (force_single_phase_charging(c2, mode, nr_phases_charging)) {
      // Want 1P.
      if (nr_phases_charging != 1) {
        return state != STATE_A ? PSR_STAGE_1P : PSR_SET_1P_IMMEDIATE;
      }
      return PSR_ALREADY_1P;
    }
    // Want 3P.
    if (nr_phases_charging != 3) {
      return state != STATE_A ? PSR_STAGE_3P : PSR_SET_3P_IMMEDIATE;
    }
    return PSR_ALREADY_3P;
  }
  // AUTO + (Normal | Smart | Off): only SMART participates.
  if (mode == MODE_SMART) {
    if (nr_phases_charging != 3) return PSR_AUTO_STAGE_3P;
    // Already 3P: cancel a pending switch if one is staged; else noop.
    // NO_SWITCH is value 0 in Switch_Phase_t (main.h:325).
    if (switching_phases_c2 != 0) return PSR_AUTO_CANCEL;
  }
  return PSR_NOOP;
}

bool decide_state_c_contactor2(EnableC2Config c2,
                               Mode           mode,
                               uint8_t        nr_of_phases_charging,
                               uint8_t        switching_phases_c2) {
  // Pre-adjustment mirrors the staged-switch branch at main.cpp:884-889:
  // Switching_Phases_C2 may already hold a pending target that overrides
  // the current nr_of_phases_charging before Force_Single_Phase_Charging
  // re-evaluates. GOING_TO_SWITCH_1P=1, GOING_TO_SWITCH_3P=2 in
  // Switch_Phase_t (main.h:325).
  uint8_t effective = nr_of_phases_charging;
  if (switching_phases_c2 == 1) effective = 1;
  else if (switching_phases_c2 == 2) effective = 3;
  // Force_Single_Phase_Charging returns 1 for 1P, 0 for 3P; CONTACTOR2
  // engages iff 3P.
  return force_single_phase_charging(c2, mode, effective) == 0;
}

uint8_t decide_state_power_unavailable(uint8_t state) {
  // Mirrors main.cpp:729-740. Caller suppresses the setState() call
  // when target == current (e.g. STATE_A, STATE_C1, STATE_B1 all map
  // to themselves — no transition), so the original early-returns in
  // the legacy function are encoded as fixpoints here.
  if (state == STATE_A)  return STATE_A;
  if (state == STATE_C)  return STATE_C1;
  if (state == STATE_C1) return STATE_C1;
  if (state == STATE_B1) return STATE_B1;
  return STATE_B1;
}

ModeSwitchReason decide_mode_switch_cp_reason(EnableC2Config c2,
                                              Mode           old_mode,
                                              Mode           new_mode,
                                              uint8_t        nr_of_phases_charging) {
  // Block 1 (main.cpp:617-626): C2==SOLAR_OFF, crossing the SOLAR
  // boundary in either direction.
  if (c2 == C2_SOLAR_OFF) {
    const bool entering_solar = (old_mode != MODE_SOLAR) && (new_mode == MODE_SOLAR);
    const bool leaving_solar  = (old_mode == MODE_SOLAR) && (new_mode != MODE_SOLAR);
    if (entering_solar || leaving_solar) return MSR_SOLAR_OFF_C2_BOUNDARY;
  }
  // Block 2 (main.cpp:630-637): C2==AUTO, leaving MODE_SOLAR while
  // currently charging on 1P (need to switch back to 3P for
  // Smart/Normal). `old_mode != new_mode` pins that we're actually
  // changing mode; the original code tested this explicitly.
  if (c2 == C2_AUTO &&
      old_mode != new_mode &&
      old_mode == MODE_SOLAR &&
      nr_of_phases_charging == 1) {
    return MSR_AUTO_SOLAR_EXIT_1P;
  }
  return MSR_NOOP;
}

SolarStartPhaseReason decide_solar_start_phase(Mode           mode,
                                               uint8_t        state,
                                               EnableC2Config c2,
                                               int32_t        isum_dA,
                                               uint16_t       min_current_a,
                                               uint8_t        nr_of_phases_charging) {
  if (mode != MODE_SOLAR) return SSPR_NOOP;
  if (state != STATE_B) return SSPR_NOOP;
  if (c2 != C2_AUTO) return SSPR_NOOP;
  // Solar charging drains the mains — surplus shows up as negative Isum.
  // Threshold: 30x MinCurrent (3 phases × 10-tenths per amp) + 30 dA
  // (3 × 1.0 A regulation headroom). See main.cpp:1193.
  const int32_t three_phase_threshold =
      static_cast<int32_t>(30) * static_cast<int32_t>(min_current_a) + 30;
  if (-isum_dA >= three_phase_threshold) {
    return nr_of_phases_charging != 3 ? SSPR_STAGE_3P : SSPR_NOOP;
  }
  return nr_of_phases_charging != 1 ? SSPR_STAGE_1P : SSPR_NOOP;
}

SolarAdmissionReason decide_solar_admission(Mode     mode,
                                            uint8_t  active_evse,
                                            int32_t  isum_dA,
                                            uint16_t start_current_a,
                                            uint16_t min_current_a,
                                            uint16_t import_current_a,
                                            int32_t  total_current_dA) {
  if (mode != MODE_SOLAR) return SAR_ALLOW;

  // Guard 1 (main.cpp:1053): no active EVSE yet. Isum >= -StartCurrent*10
  // means net draw is at most StartCurrent amps of export — i.e. surplus
  // hasn't exceeded the threshold to spin up the first EVSE.
  if (active_evse == 0) {
    const int32_t threshold = -static_cast<int32_t>(start_current_a) * 10;
    if (isum_dA >= threshold) return SAR_DENY_NO_SURPLUS;
  }

  // Guard 2 (main.cpp:1057): the min-per-EVSE budget exceeds what's
  // already divvied up across active EVSEs — admitting another one
  // would mean each can't get MinCurrent.
  const int32_t min_slot_total =
      static_cast<int32_t>(active_evse) * static_cast<int32_t>(min_current_a) * 10;
  if (min_slot_total > total_current_dA) return SAR_DENY_CANT_SPLIT;

  // Guard 3 (main.cpp:1061): admitting one more EVSE would push Isum
  // above the ImportCurrent budget. Only applies when at least one
  // EVSE is already charging.
  if (active_evse > 0) {
    const int32_t import_budget =
        static_cast<int32_t>(import_current_a) * 10 + total_current_dA - min_slot_total;
    if (isum_dA > import_budget) return SAR_DENY_OVER_IMPORT;
  }

  return SAR_ALLOW;
}

CircuitAdmissionReason decide_circuit_admission(Mode     mode,
                                                uint8_t  active_evse,
                                                int32_t  baseload_dA,
                                                int32_t  baseload_ev_dA,
                                                int32_t  isum_dA,
                                                uint16_t min_current_a,
                                                uint16_t max_mains_a,
                                                uint16_t max_circuit_a,
                                                uint16_t max_sum_mains_a,
                                                uint8_t  load_balance,
                                                uint8_t  ev_meter_type,
                                                uint8_t  phases) {
  const int32_t min_slot_total =
      static_cast<int32_t>(active_evse) * static_cast<int32_t>(min_current_a) * 10;

  // Guard 1 (main.cpp:1066): MaxMains — only Smart/Solar.
  if (mode != MODE_NORMAL) {
    const int32_t max_mains_dA = static_cast<int32_t>(max_mains_a) * 10;
    if (min_slot_total + baseload_dA > max_mains_dA) return CAR_DENY_MAX_MAINS;
  }

  // Guard 2 (main.cpp:1070): MaxCircuit — fires for LoadBl==1 always,
  // or LoadBl==0 with an EV meter in non-Normal mode.
  const bool check_max_circuit =
      (load_balance == 0 && ev_meter_type != 0 && mode != MODE_NORMAL) ||
      (load_balance == 1);
  if (check_max_circuit) {
    const int32_t max_circuit_dA = static_cast<int32_t>(max_circuit_a) * 10;
    if (min_slot_total + baseload_ev_dA > max_circuit_dA) return CAR_DENY_MAX_CIRCUIT;
  }

  // Guard 3 (main.cpp:1081): MaxSumMains — only Smart/Solar and only
  // when the limit is configured (non-zero).
  if (mode != MODE_NORMAL && max_sum_mains_a != 0) {
    const int32_t scaled =
        static_cast<int32_t>(phases) * min_slot_total;
    const int32_t max_sum_dA = static_cast<int32_t>(max_sum_mains_a) * 10;
    if (scaled + isum_dA > max_sum_dA) return CAR_DENY_MAX_SUM_MAINS;
  }

  return CAR_ALLOW;
}

int32_t decide_solar_iset_balanced_delta_dA(uint8_t active_evse,
                                            int32_t idifference_dA,
                                            int32_t isum_import_dA,
                                            bool    phases_last_update_flag) {
  if (active_evse == 0) return 0;
  if (idifference_dA <= 0) return 0;
  if (!phases_last_update_flag) return 0;

  if (isum_import_dA < 0) {
    // Surplus available — ramp up gently.
    if (isum_import_dA < -10 && idifference_dA > 10) return 5;
    return 1;
  }
  // Over-draw — ramp down. The three step-down bands + dead band:
  if (isum_import_dA > 20) return -(isum_import_dA / 2);
  if (isum_import_dA > 10) return -5;
  if (isum_import_dA > 3)  return -1;
  return 0;
}

uint8_t decide_cp_disconnect_target_on_mode_switch(uint8_t state) {
  // Legacy: `if (State == STATE_C) setState(STATE_C1); else if
  // (State != STATE_C1 && State == STATE_B) setState(STATE_B1);`
  // The `State != STATE_C1` guard is a no-op inside the else branch
  // (STATE_C1 can never satisfy `State == STATE_B`), but preserved
  // here as a comment so the source map back to main.cpp is clear.
  if (state == STATE_C) return STATE_C1;
  if (state == STATE_B) return STATE_B1;
  return state;  // fixpoint — legacy leaves state unchanged
}

uint8_t decide_state_on_access_change(uint8_t access, uint8_t state) {
  // Mirrors main.cpp:962-966. AccessStatus_t { OFF=0, ON=1, PAUSE=2 };
  // the legacy code does nothing unless Access == OFF || Access == PAUSE
  // (literal equality, not `!= ON` — any future non-OFF/PAUSE/ON value
  // would fall through). STATE_MODEM_* values from main_c.h (11..14)
  // map to STATE_B1 just like STATE_B.
  constexpr uint8_t ACCESS_OFF   = 0;
  constexpr uint8_t ACCESS_PAUSE = 2;
  constexpr uint8_t STATE_MODEM_REQUEST = 11;
  constexpr uint8_t STATE_MODEM_WAIT    = 12;
  constexpr uint8_t STATE_MODEM_DONE    = 13;
  constexpr uint8_t STATE_MODEM_DENIED  = 14;

  if (access != ACCESS_OFF && access != ACCESS_PAUSE) return state;
  if (state == STATE_C) return STATE_C1;
  if (state == STATE_C1) return state;  // excluded by `State != STATE_C1` guard
  if (state == STATE_B ||
      state == STATE_MODEM_REQUEST ||
      state == STATE_MODEM_WAIT ||
      state == STATE_MODEM_DONE ||
      state == STATE_MODEM_DENIED) {
    return STATE_B1;
  }
  return state;
}

uint16_t cp_duty_for_current_v3(uint16_t current_dA, uint16_t min_current_dA) {
  // Byte-identical transliteration of main.cpp::SetCurrent()'s v3 branch
  // (SMARTEVSE_VERSION >= 30 && < 40). Divergence from duty_for_current():
  //   * inputs are deci-amps, not amps
  //   * float→uint32 cast is truncating; duty_for_current rounds
  //   * out-of-range falls back to 100 permille (≈6 A) not 1000 (off)
  //   * final ×1024/1000 rescale because v3's PWM driver is 1024-scale
  // Changing any of those changes what the EV sees on the pilot.
  // Divisions deliberately use `double` literals (not `0.6f`) to match the
  // firmware's original precision. In float, 0.6f ≈ 0.60000002, which
  // makes `60 / 0.6f ≈ 99.999…` truncate to 99 instead of 100 — off by
  // one advertised-current unit at the minimum-current boundary.
  uint32_t duty;
  if (current_dA >= min_current_dA && current_dA <= 510) {
    duty = static_cast<uint32_t>(current_dA / 0.6);
  } else if (current_dA > 510 && current_dA <= 800) {
    duty = static_cast<uint32_t>(current_dA / 2.5) + 640;
  } else {
    duty = 100;
  }
  return static_cast<uint16_t>(duty * 1024 / 1000);
}

const char* state_name(State s) {
  switch (s) {
    case STATE_A:  return "A";
    case STATE_B:  return "B";
    case STATE_C:  return "C";
    case STATE_D:  return "D";
    case STATE_B1: return "B1";
    case STATE_C1: return "C1";
  }
  return "?";
}

const char* mode_name(Mode m) {
  switch (m) {
    case MODE_OFF:    return "Off";
    case MODE_NORMAL: return "Normal";
    case MODE_SMART:  return "Smart";
    case MODE_SOLAR:  return "Solar";
  }
  return "?";
}

void Core::reset() {
  out_ = Outputs{};
  charge_delay_s_ = 0;
  c1_timer_s_ = 0;
  pilot_force_disconnected_ = false;
  last_errors_ = 0;
  if (obs_) {
    obs_->on_pwm(out_.pwm_duty_permille, out_.advertised_a);
    obs_->on_pilot(out_.pilot_connected);
  }
}

float Core::compute_target_current(const Inputs& in) const {
  if (in.mode == MODE_OFF) return 0.0f;
  float base = in.override_a > 0.0f ? in.override_a : static_cast<float>(in.max_current_a);
  if (base > in.max_current_a) base = in.max_current_a;
  // Load balance against mains headroom — 1 A safety margin, matching the
  // firmware's IsCurrentAvailable logic spiritually (not literally).
  float worst = std::max({in.l1_a, in.l2_a, in.l3_a});
  if (worst > 0.0f) {
    float headroom = static_cast<float>(in.max_mains_a) - worst - 1.0f;
    if (headroom < 0.0f) headroom = 0.0f;
    if (base > headroom) base = headroom;
  }
  if (base < static_cast<float>(in.min_current_a)) return 0.0f;
  return base;
}

void Core::apply_entry_effects(State s, const Inputs& in) {
  // Mirrors the per-state block inside setState() in main.cpp:794-947.
  switch (s) {
    case STATE_A:
    case STATE_B1: {
      bool c1 = out_.contactor1, c2 = out_.contactor2;
      out_.contactor1 = false;
      out_.contactor2 = false;
      if (c1 && obs_) obs_->on_contactor(1, false);
      if (c2 && obs_) obs_->on_contactor(2, false);
      out_.pwm_duty_permille = 1000;   // SetCPDuty(1024) → pilot steady +12V
      out_.advertised_a = 0.0f;
      if (obs_) obs_->on_pwm(out_.pwm_duty_permille, out_.advertised_a);
      if (s == STATE_B1 && !pilot_force_disconnected_ && in.access_on) {
        pilot_force_disconnected_ = true;
        out_.pilot_connected = false;
        if (obs_) obs_->on_pilot(false);
      }
      if (s == STATE_A) {
        charge_delay_s_ = 0;
        pilot_force_disconnected_ = false;
        if (!out_.pilot_connected) {
          out_.pilot_connected = true;
          if (obs_) obs_->on_pilot(true);
        }
      }
      break;
    }
    case STATE_B: {
      bool c1 = out_.contactor1, c2 = out_.contactor2;
      out_.contactor1 = false;
      out_.contactor2 = false;
      if (c1 && obs_) obs_->on_contactor(1, false);
      if (c2 && obs_) obs_->on_contactor(2, false);
      float target = compute_target_current(in);
      out_.advertised_a = target;
      out_.pwm_duty_permille = duty_for_current(target);
      if (obs_) obs_->on_pwm(out_.pwm_duty_permille, out_.advertised_a);
      if (!out_.pilot_connected) {
        out_.pilot_connected = true;
        if (obs_) obs_->on_pilot(true);
      }
      break;
    }
    case STATE_C: {
      float target = compute_target_current(in);
      out_.charge_current_a = target;
      out_.advertised_a = target;
      out_.pwm_duty_permille = duty_for_current(target);
      // Close L1 always; L2/L3 follow three-phase config.
      if (!out_.contactor1) {
        out_.contactor1 = true;
        if (obs_) obs_->on_contactor(1, true);
      }
      if (!out_.contactor2) {
        out_.contactor2 = true;
        if (obs_) obs_->on_contactor(2, true);
      }
      if (obs_) obs_->on_pwm(out_.pwm_duty_permille, out_.advertised_a);
      break;
    }
    case STATE_C1: {
      out_.pwm_duty_permille = 1000;  // SetCPDuty(1024) — EV sees +12V, must stop
      out_.advertised_a = 0.0f;
      c1_timer_s_ = 6;                // max 6 s before we force contactors off
      charge_delay_s_ = 15;
      if (obs_) obs_->on_pwm(out_.pwm_duty_permille, out_.advertised_a);
      break;
    }
    default:
      break;
  }
}

void Core::transition_to(State s, const char* reason, const Inputs& in) {
  if (s == out_.state) return;
  State from = out_.state;
  out_.state = s;
  apply_entry_effects(s, in);
  if (obs_) obs_->on_state(from, s, reason ? reason : "");
}

void Core::tick_10ms(const Inputs& in) {
  // Surface error-flag edges so the dashboard / scenarios see transitions.
  if (in.error_flags != last_errors_) {
    uint8_t changed = in.error_flags ^ last_errors_;
    auto bit = [&](uint8_t mask, const char* name) {
      if (changed & mask) {
        bool set = (in.error_flags & mask) != 0;
        if (obs_) obs_->on_error(mask, set);
        (void)name;
      }
    };
    bit(ERR_LESS_6A,     "LESS_6A");
    bit(ERR_CT_NOCOMM,   "CT_NOCOMM");
    bit(ERR_TEMP_HIGH,   "TEMP_HIGH");
    bit(ERR_EV_NOCOMM,   "EV_NOCOMM");
    bit(ERR_RCM_TRIPPED, "RCM_TRIPPED");
    last_errors_ = in.error_flags;
  }

  // Safety: RCM or TEMP_HIGH or Mode=Off kicks us back to C1/B1.
  bool fault = (in.error_flags & (ERR_RCM_TRIPPED | ERR_TEMP_HIGH)) != 0;
  if (fault) {
    if (out_.state == STATE_C) {
      transition_to(STATE_C1, "fault", in);
      return;
    }
    if (out_.state != STATE_B1 && out_.state != STATE_A) {
      transition_to(STATE_B1, "fault", in);
      return;
    }
  }

  if (!in.access_on && out_.state == STATE_C) {
    transition_to(STATE_C1, "access_off", in);
    return;
  }

  // Pilot-driven transitions — mirrors Timer10ms_singlerun logic.
  switch (in.pilot) {
    case PILOT_12V:
      if (out_.state != STATE_A) transition_to(STATE_A, "pilot_12v", in);
      break;
    case PILOT_9V:
      if (out_.state == STATE_A) transition_to(STATE_B, "pilot_9v", in);
      else if (out_.state == STATE_C) transition_to(STATE_C1, "pilot_9v", in);
      break;
    case PILOT_6V:
      if (out_.state == STATE_B && charge_delay_s_ == 0 && in.access_on && in.mode != MODE_OFF) {
        transition_to(STATE_C, "pilot_6v", in);
      }
      break;
    case PILOT_NOK:
    case PILOT_SHORT:
      if (out_.state == STATE_C) transition_to(STATE_C1, "pilot_bad", in);
      else if (out_.state != STATE_A) transition_to(STATE_B1, "pilot_bad", in);
      break;
    default:
      break;
  }

  // While in C, keep the advertised current in sync with headroom.
  if (out_.state == STATE_C) {
    float target = compute_target_current(in);
    if (target != out_.charge_current_a) {
      out_.charge_current_a = target;
      out_.advertised_a = target;
      uint16_t new_duty = duty_for_current(target);
      if (new_duty != out_.pwm_duty_permille) {
        out_.pwm_duty_permille = new_duty;
        if (obs_) obs_->on_pwm(out_.pwm_duty_permille, out_.advertised_a);
      }
    }
  }
}

void Core::tick_1s() {
  if (charge_delay_s_ > 0) --charge_delay_s_;
  if (out_.state == STATE_C1) {
    if (c1_timer_s_ > 0) --c1_timer_s_;
    if (c1_timer_s_ == 0) {
      // Timer expired → open contactors and fall back to B1 (equivalent
      // to main.cpp's "force contactor off" branch).
      bool c1 = out_.contactor1, c2 = out_.contactor2;
      out_.contactor1 = false;
      out_.contactor2 = false;
      if (c1 && obs_) obs_->on_contactor(1, false);
      if (c2 && obs_) obs_->on_contactor(2, false);
      out_.state = STATE_B1;
      if (obs_) obs_->on_state(STATE_C1, STATE_B1, "c1_timeout");
    }
  }
}

}  // namespace evse

// Flat C ABI bridge — see firmware_core_shim.h for the rationale.
extern "C" uint8_t evse_decode_pilot_mv(uint32_t min_mv, uint32_t max_mv) {
  return static_cast<uint8_t>(evse::decode_pilot(min_mv, max_mv));
}

extern "C" uint16_t evse_cp_duty_for_current_v3(uint16_t current_dA,
                                                uint16_t min_current_dA) {
  return evse::cp_duty_for_current_v3(current_dA, min_current_dA);
}

extern "C" uint8_t evse_force_single_phase_charging(uint8_t c2, uint8_t mode,
                                                    uint8_t nr_phases_charging) {
  return evse::force_single_phase_charging(
      static_cast<evse::EnableC2Config>(c2),
      static_cast<evse::Mode>(mode),
      nr_phases_charging);
}

extern "C" uint8_t evse_check_switching_phases(uint8_t c2, uint8_t mode, uint8_t state,
                                               uint8_t nr_phases_charging,
                                               uint8_t switching_phases_c2) {
  return static_cast<uint8_t>(evse::check_switching_phases(
      static_cast<evse::EnableC2Config>(c2),
      static_cast<evse::Mode>(mode),
      state,
      nr_phases_charging,
      switching_phases_c2));
}

extern "C" uint8_t evse_decide_state_c_contactor2(uint8_t c2, uint8_t mode,
                                                  uint8_t nr_of_phases_charging,
                                                  uint8_t switching_phases_c2) {
  return evse::decide_state_c_contactor2(
      static_cast<evse::EnableC2Config>(c2),
      static_cast<evse::Mode>(mode),
      nr_of_phases_charging,
      switching_phases_c2) ? 1 : 0;
}

extern "C" uint8_t evse_decide_state_power_unavailable(uint8_t state) {
  return evse::decide_state_power_unavailable(state);
}

extern "C" uint8_t evse_decide_state_on_access_change(uint8_t access, uint8_t state) {
  return evse::decide_state_on_access_change(access, state);
}

extern "C" uint8_t evse_decide_mode_switch_cp_reason(uint8_t c2, uint8_t old_mode,
                                                    uint8_t new_mode,
                                                    uint8_t nr_of_phases_charging) {
  return static_cast<uint8_t>(evse::decide_mode_switch_cp_reason(
      static_cast<evse::EnableC2Config>(c2),
      static_cast<evse::Mode>(old_mode),
      static_cast<evse::Mode>(new_mode),
      nr_of_phases_charging));
}

extern "C" uint8_t evse_decide_cp_disconnect_target_on_mode_switch(uint8_t state) {
  return evse::decide_cp_disconnect_target_on_mode_switch(state);
}

extern "C" uint8_t evse_decide_solar_start_phase(uint8_t mode, uint8_t state,
                                                 uint8_t c2, int32_t isum_dA,
                                                 uint16_t min_current_a,
                                                 uint8_t nr_of_phases_charging) {
  return static_cast<uint8_t>(evse::decide_solar_start_phase(
      static_cast<evse::Mode>(mode),
      state,
      static_cast<evse::EnableC2Config>(c2),
      isum_dA,
      min_current_a,
      nr_of_phases_charging));
}

extern "C" uint8_t evse_decide_solar_admission(uint8_t mode, uint8_t active_evse,
                                               int32_t isum_dA,
                                               uint16_t start_current_a,
                                               uint16_t min_current_a,
                                               uint16_t import_current_a,
                                               int32_t total_current_dA) {
  return static_cast<uint8_t>(evse::decide_solar_admission(
      static_cast<evse::Mode>(mode),
      active_evse,
      isum_dA,
      start_current_a,
      min_current_a,
      import_current_a,
      total_current_dA));
}

extern "C" uint8_t evse_decide_circuit_admission(uint8_t mode, uint8_t active_evse,
                                                 int32_t baseload_dA,
                                                 int32_t baseload_ev_dA,
                                                 int32_t isum_dA,
                                                 uint16_t min_current_a,
                                                 uint16_t max_mains_a,
                                                 uint16_t max_circuit_a,
                                                 uint16_t max_sum_mains_a,
                                                 uint8_t load_balance,
                                                 uint8_t ev_meter_type,
                                                 uint8_t phases) {
  return static_cast<uint8_t>(evse::decide_circuit_admission(
      static_cast<evse::Mode>(mode),
      active_evse,
      baseload_dA,
      baseload_ev_dA,
      isum_dA,
      min_current_a,
      max_mains_a,
      max_circuit_a,
      max_sum_mains_a,
      load_balance,
      ev_meter_type,
      phases));
}

extern "C" int32_t evse_decide_solar_iset_balanced_delta_dA(uint8_t active_evse,
                                                            int32_t idifference_dA,
                                                            int32_t isum_import_dA,
                                                            uint8_t phases_last_update_flag) {
  return evse::decide_solar_iset_balanced_delta_dA(
      active_evse, idifference_dA, isum_import_dA,
      phases_last_update_flag != 0);
}
