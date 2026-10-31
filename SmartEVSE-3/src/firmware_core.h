// Host-compilable extraction of the SmartEVSE IEC 61851 state machine.
//
// Source of truth:
//   - pilot voltage thresholds: SmartEVSE-3/src/main.cpp :: Pilot()   (v3 ESP32 variant)
//   - state transition rules:   SmartEVSE-3/src/main.cpp :: setState()
//   - state/error enums:        SmartEVSE-3/src/main_c.h
//
// The firmware side still owns the live code for now (Phase B step 1).
// As the firmware refactor proceeds, main.cpp will be migrated to call
// into this module directly so there is only one implementation.

#pragma once

#include <cstdint>

namespace evse {

enum State : uint8_t {
  STATE_A  = 0,   // vehicle not connected
  STATE_B  = 1,   // vehicle connected, not ready
  STATE_C  = 2,   // vehicle ready to charge
  STATE_D  = 3,   // ventilation required (unused)
  STATE_B1 = 9,   // connected, EVSE not ready (PWM off)
  STATE_C1 = 10,  // stopping charge (PWM off, grace period)
};

enum Pilot : uint8_t {
  PILOT_SHORT = 255,
  PILOT_12V   = 12,
  PILOT_9V    = 9,
  PILOT_6V    = 6,
  PILOT_3V    = 3,
  PILOT_DIODE = 1,
  PILOT_NOK   = 0,
};

enum Mode : uint8_t {
  MODE_OFF    = 255,
  MODE_NORMAL = 0,
  MODE_SMART  = 1,
  MODE_SOLAR  = 2,
};

// Contactor 2 configuration — matches main.h::EnableC2_t. Determines
// whether a second-phase contactor is wired and under what conditions
// the firmware is allowed to engage it.
enum EnableC2Config : uint8_t {
  C2_NOT_PRESENT = 0,  // no C2 contactor wired
  C2_ALWAYS_OFF  = 1,  // C2 wired but never close (force 1-phase)
  C2_SOLAR_OFF   = 2,  // C2 closed except in Solar mode (1P on solar)
  C2_ALWAYS_ON   = 3,  // C2 always closed (force 3-phase)
  C2_AUTO        = 4,  // C2 follows Nr_Of_Phases_Charging
};

// Error bit masks — values match main_c.h.
constexpr uint8_t ERR_NONE        = 0;
constexpr uint8_t ERR_LESS_6A     = 1;
constexpr uint8_t ERR_CT_NOCOMM   = 2;
constexpr uint8_t ERR_TEMP_HIGH   = 4;
constexpr uint8_t ERR_EV_NOCOMM   = 8;
constexpr uint8_t ERR_RCM_TRIPPED = 16;

struct Inputs {
  Pilot    pilot          = PILOT_12V;
  Mode     mode           = MODE_SMART;
  bool     access_on      = true;     // ExtSwitch / AccessStatus
  uint8_t  error_flags    = ERR_NONE; // RCM, TEMP, etc.
  uint16_t max_current_a   = 16;       // user-set max per firmware MaxCurrent
  uint16_t max_mains_a     = 25;       // MaxMains
  uint16_t max_sum_mains_a = 0;        // MaxSumMains (0 = unset / disabled)
  uint16_t min_current_a   = 6;        // MinCurrent
  float    l1_a           = 0.0f;
  float    l2_a           = 0.0f;
  float    l3_a           = 0.0f;
  float    override_a     = 0.0f;     // MQTT CurrentOverride (0 = unset)
  float    temp_c         = 25.0f;
  uint8_t  load_balance   = 0;        // LoadBl: 0=standalone, 1=master, >=2=slave
};

struct Outputs {
  State    state              = STATE_A;
  bool     contactor1         = false;
  bool     contactor2         = false;
  uint16_t pwm_duty_permille  = 1000; // 1000 = 100% = PWM off (pilot high)
  bool     pilot_connected    = true;
  float    advertised_a       = 0.0f; // what the EV sees via PWM duty
  float    charge_current_a   = 0.0f; // actual SetCurrent() target
};

// Observer interface. Override to capture transitions + side effects.
// The harness uses this to emit the stdio JSON events.
class Observer {
 public:
  virtual ~Observer() = default;
  virtual void on_state(State /*from*/, State /*to*/, const char* /*reason*/) {}
  virtual void on_contactor(uint8_t /*phase*/, bool /*closed*/) {}
  virtual void on_pilot(bool /*connected*/) {}
  virtual void on_pwm(uint16_t /*duty_permille*/, float /*advertised_a*/) {}
  virtual void on_error(uint8_t /*flag*/, bool /*set*/) {}
  virtual void on_log(const char* /*level*/, const char* /*msg*/) {}
};

// Transliteration of main.cpp Pilot() (ESP32 v3 thresholds, lines 1050-1056).
// Min/Max are in millivolts at the ADC (post-divider).
Pilot decode_pilot(uint32_t min_mv, uint32_t max_mv);

// Convenience overload — when you only have a single stable reading.
inline Pilot decode_pilot(uint32_t mv) { return decode_pilot(mv, mv); }

// Convenience for the test harness / dashboard: decode from nominal CP
// line voltage in mV (12000 = +12V, 9000 = +9V, 6000 = +6V, etc.).
// Uses ±1000 mV tolerance bands. The firmware itself never sees CP
// voltage directly — it reads the divided-down ADC signal, decoded by
// decode_pilot(min,max) above. This helper exists purely so tests can
// simulate pilot levels in their natural units.
Pilot pilot_from_cp_mv(uint32_t cp_mv);

// Duty cycle corresponding to an advertised charge current (Amps).
// Mirrors the 1000-step PWM used by main.cpp / SetCPDuty.
uint16_t duty_for_current(float amps);

// Advertised current implied by a PWM duty cycle (inverse of duty_for_current).
float current_for_duty(uint16_t duty_permille);

// SetCurrent()'s exact duty calculation for SMARTEVSE v3, 1024-scale.
// Inputs are deci-amps — see firmware_core_shim.h for the rationale.
uint16_t cp_duty_for_current_v3(uint16_t current_dA, uint16_t min_current_dA);

// Decide whether to charge on a single phase given the Contactor 2
// configuration, the current operating mode, and the currently active
// phase count. Bit-identical port of main.cpp:686
// Force_Single_Phase_Charging(). Returns 1 for 1-phase charging and
// 0 for 3-phase (or "unknown, default to 3P"). Pure function — no
// hardware side effects. `c2` values outside the EnableC2Config enum
// fall through to the 3P default, matching the legacy function.
uint8_t force_single_phase_charging(EnableC2Config c2, Mode mode, uint8_t nr_phases_charging);

// Decision codes for check_switching_phases(). Each maps 1:1 to one
// branch of main.cpp::CheckSwitchingPhases(); callers dispatch on the
// returned reason to write Switching_Phases_C2 / Nr_Of_Phases_Charging
// and emit the legacy log line. The pure decision function has no
// side effects — it just inspects its inputs and returns one enum.
enum PhaseSwitchReason : uint8_t {
  PSR_NOOP = 0,            // no write, no log (AUTO+Normal/Off, or SMART+AUTO already 3P + no pending switch)
  PSR_STAGE_1P,            // write Switching_Phases_C2=GOING_TO_SWITCH_1P, log "Switching to 1P!!!"
  PSR_ALREADY_1P,          // write Switching_Phases_C2=NO_SWITCH,          log "No need to switch to 1P !"
  PSR_STAGE_3P,            // write Switching_Phases_C2=GOING_TO_SWITCH_3P, log "Switching to 3P!!!"
  PSR_ALREADY_3P,          // write Switching_Phases_C2=NO_SWITCH,          log "No need to switch to 3P !"
  PSR_SET_1P_IMMEDIATE,    // write Nr_Of_Phases_Charging=1                 (STATE_A path, no _LOG_A)
  PSR_SET_3P_IMMEDIATE,    // write Nr_Of_Phases_Charging=3                 (STATE_A path, no _LOG_A)
  PSR_AUTO_STAGE_3P,       // write Switching_Phases_C2=GOING_TO_SWITCH_3P, log "AUTO Switching to 3P\n"
  PSR_AUTO_CANCEL,         // write Switching_Phases_C2=NO_SWITCH,          log "Switching Cancelled!\n"
};

// Decide whether / how to move between 1-phase and 3-phase charging.
// Bit-identical port of main.cpp:547 CheckSwitchingPhases(). Inputs
// are the four globals the function reads. The caller is responsible
// for writing Switching_Phases_C2 / Nr_Of_Phases_Charging and emitting
// the corresponding _LOG_A line — see PhaseSwitchReason for the
// reason→side-effect map. `switching_phases_c2` is passed as uint8_t
// because Switch_Phase_t lives in main.h, which this header cannot
// include. Only PSR_AUTO_CANCEL inspects it (cancels a pending switch).
PhaseSwitchReason check_switching_phases(EnableC2Config c2,
                                         Mode           mode,
                                         uint8_t        state,
                                         uint8_t        nr_phases_charging,
                                         uint8_t        switching_phases_c2);

// Decide whether STATE_C entry should engage CONTACTOR2 and finalize
// Nr_Of_Phases_Charging to 3 (true) or release CONTACTOR2 and finalize
// to 1 (false). Source of truth: main.cpp:872-898 STATE_C entry block.
//
// The function first applies the Switching_Phases_C2 pre-adjustment
// (GOING_TO_SWITCH_1P → effective 1, GOING_TO_SWITCH_3P → effective 3,
// NO_SWITCH → unchanged), then feeds the effective phase count through
// force_single_phase_charging() and returns the negation (1P → false,
// 3P → true). Pure composition — caller handles CONTACTOR1_ON,
// SEND_TO_ESP32, setSolarStopTimer(0), MaxSumMainsTimer=0, and
// resetting Switching_Phases_C2 to NO_SWITCH. `switching_phases_c2`
// is passed as uint8_t because Switch_Phase_t lives in main.h.
bool decide_state_c_contactor2(EnableC2Config c2,
                               Mode           mode,
                               uint8_t        nr_of_phases_charging,
                               uint8_t        switching_phases_c2);

// Decide the target state for setStatePowerUnavailable(). Source of
// truth: main.cpp:729. Returns the *target* State to transition into;
// callers should only call setState() when the returned value differs
// from the current state. Encoding:
//   STATE_A  → STATE_A   (already disconnected — no-op)
//   STATE_C  → STATE_C1  (currently charging — tell EV to stop)
//   STATE_C1 → STATE_C1  (already stopping — no-op)
//   STATE_B1 → STATE_B1  (already stopped — no-op)
//   anything else (B, D, modem states) → STATE_B1
// Passed as uint8_t because main.h's STATE_MODEM_* values aren't in
// evse::State — any unknown state falls through to STATE_B1.
uint8_t decide_state_power_unavailable(uint8_t state);

// Decide the target state when setAccess() transitions away from ON.
// Source of truth: main.cpp:956-966. Returns the target State value —
// caller should only call setState() when it differs from the current
// state (fixpoints encode the legacy no-op branches). Encoding:
//   access == ON  → state unchanged  (legacy takes no action at all)
//   access != ON  (OFF or PAUSE):
//     STATE_C                → STATE_C1
//     STATE_C1               → STATE_C1   (legacy excluded via `State != STATE_C1`)
//     STATE_B or STATE_MODEM_{REQUEST,WAIT,DONE,DENIED} → STATE_B1
//     anything else (STATE_A, STATE_B1, STATE_D) → state unchanged
// access is AccessStatus_t (main.h:326): 0=OFF, 1=ON, 2=PAUSE.
// STATE_MODEM_* values live in main_c.h (11..14) and aren't in
// evse::State — passed as uint8_t so the helper compares by value.
uint8_t decide_state_on_access_change(uint8_t access, uint8_t state);

// Decision codes for decide_mode_switch_cp_reason(). Each maps to
// one of the two if-blocks at main.cpp:617-637 inside setMode().
enum ModeSwitchReason : uint8_t {
  MSR_NOOP = 0,                // no CP disconnect, no log
  MSR_SOLAR_OFF_C2_BOUNDARY,   // log "Disconnect CP when switching C2\n"
  MSR_AUTO_SOLAR_EXIT_1P,      // log "AUTO Solar->Smart/Normal charging 1p->3p\n"
};

// Decide whether setMode() should stage a CP disconnect and which log
// line to emit. Source of truth: main.cpp:617-637. Pure function —
// caller dispatches reason to (a) the CP-disconnect state-map helper
// decide_cp_disconnect_target_on_mode_switch(state), (b) the
// corresponding _LOG_A line. The two legacy conditions are disjoint
// because EnableC2 takes one value at a time.
//   MSR_SOLAR_OFF_C2_BOUNDARY: c2==SOLAR_OFF AND mode boundary crosses SOLAR
//   MSR_AUTO_SOLAR_EXIT_1P:    c2==AUTO AND old==SOLAR AND new!=SOLAR AND 1P
ModeSwitchReason decide_mode_switch_cp_reason(EnableC2Config c2,
                                              Mode           old_mode,
                                              Mode           new_mode,
                                              uint8_t        nr_of_phases_charging);

// State-map helper for the setMode() CP-disconnect blocks. Narrower
// than decide_state_power_unavailable — only STATE_B and STATE_C
// trigger a transition; everything else is a fixpoint. Source of
// truth: the two `if (State == STATE_C) ... else if (State != STATE_C1
// && State == STATE_B) ...` branches at main.cpp:621-622 / 633-634.
uint8_t decide_cp_disconnect_target_on_mode_switch(uint8_t state);

// Decision codes for decide_solar_start_phase(). Each maps to one
// branch of the Solar-start block at main.cpp:1186-1205 inside
// CalcBalancedCurrent().
enum SolarStartPhaseReason : uint8_t {
  SSPR_NOOP = 0,    // outside Solar/State_B/AUTO window, or already at target phase count
  SSPR_STAGE_3P,    // write Switching_Phases_C2 = GOING_TO_SWITCH_3P + _LOG_D "Solar starting in 3-phase mode"
  SSPR_STAGE_1P,    // write Switching_Phases_C2 = GOING_TO_SWITCH_1P + _LOG_D "Solar starting in 1-phase mode"
};

// Decide, at the start of Solar charging (Mode==SOLAR, State==STATE_B,
// EnableC2==AUTO), whether to stage a 3P or 1P phase switch based on
// the available solar surplus. Source of truth: main.cpp:1186-1205.
//
// The threshold is `-Isum >= 30 * MinCurrent + 30` (deci-amps):
// 30A per 10A-Min-per-phase scaled to deci-amps plus a 3×1.0 A
// regulation headroom. Below that threshold the controller falls
// back to 1P. Any trigger fires only when the current phase count
// differs from the target — otherwise NOOP.
//
// Returns NOOP when (a) mode != SOLAR, (b) state != STATE_B,
// (c) c2 != C2_AUTO, or (d) already at the target phase count.
// The legacy `_LOG_D("waiting for Solar (B) ...")` at the top of the
// outer block stays at the call site (it fires unconditionally when
// mode==SOLAR && state==STATE_B, outside this helper's scope).
SolarStartPhaseReason decide_solar_start_phase(Mode           mode,
                                               uint8_t        state,
                                               EnableC2Config c2,
                                               int32_t        isum_dA,
                                               uint16_t       min_current_a,
                                               uint8_t        nr_of_phases_charging);

// Decision codes for decide_solar_admission(). Maps the three Solar-mode
// "no current available" guards at main.cpp:1052-1065 inside
// IsCurrentAvailable().
enum SolarAdmissionReason : uint8_t {
  SAR_ALLOW = 0,          // pass — non-Solar mode or all Solar guards cleared
  SAR_DENY_NO_SURPLUS,    // ActiveEVSE==0 and Isum >= -StartCurrent*10 (no solar headroom yet)
  SAR_DENY_CANT_SPLIT,    // ActiveEVSE*MinCurrent*10 > TotalCurrent (already-set budget too small to split)
  SAR_DENY_OVER_IMPORT,   // ActiveEVSE>0 and admitting one more would exceed ImportCurrent budget
};

// Decide whether to admit another EVSE for charging in Solar mode.
// Source of truth: main.cpp:1052-1065. The three guards are checked in
// order; the first match wins. Returns SAR_ALLOW for non-Solar mode,
// matching the legacy fallthrough. All currents are passed in the same
// units as the legacy code (deci-amps for per-EVSE totals and Isum;
// whole amps for the configuration knobs).
//
// Callers that use this to short-circuit IsCurrentAvailable() should
// return 0 on any non-ALLOW reason, matching the legacy `return 0`.
SolarAdmissionReason decide_solar_admission(Mode     mode,
                                            uint8_t  active_evse,
                                            int32_t  isum_dA,
                                            uint16_t start_current_a,
                                            uint16_t min_current_a,
                                            uint16_t import_current_a,
                                            int32_t  total_current_dA);

// Decision codes for decide_circuit_admission(). Maps the three
// circuit/sum guards at main.cpp:1065-1084 inside IsCurrentAvailable().
enum CircuitAdmissionReason : uint8_t {
  CAR_ALLOW = 0,              // pass — all circuit guards cleared (or gated off)
  CAR_DENY_MAX_MAINS,         // Mode != NORMAL and ActiveEVSE*MinCurrent*10 + Baseload    > MaxMains*10
  CAR_DENY_MAX_CIRCUIT,       // Mode-gated: EVMeter present (LoadBl=0) or Master, and the above for MaxCircuit
  CAR_DENY_MAX_SUM_MAINS,     // Mode != NORMAL and MaxSumMains != 0 and Phases*ActiveEVSE*MinCurrent*10 + Isum > MaxSumMains*10
};

// Decide whether admitting one more EVSE would exceed any configured
// circuit limit (MaxMains, MaxCircuit, MaxSumMains). Source of truth:
// main.cpp:1065-1084.
//
// Gating summary:
//   - MaxMains:    only Mode != NORMAL.
//   - MaxCircuit:  (LoadBl==0 && ev_meter_type!=0 && Mode != NORMAL) || LoadBl==1.
//   - MaxSumMains: Mode != NORMAL and max_sum_mains_a != 0.
//
// `active_evse` should already include the hypothetical new EVSE
// (legacy code does `ActiveEVSE++` before running these guards).
// `phases` must be precomputed by the caller: 1 by default, 1-or-3
// based on Force_Single_Phase_Charging() when LoadBl == 0.
//
// Returns CAR_ALLOW for MODE_NORMAL (matches legacy "none of these
// guards fire in Normal mode except MaxCircuit with LoadBl==1").
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
                                                uint8_t  phases);

// Compute the IsetBalanced delta (deci-amps) applied by the Solar ramp
// control block inside CalcBalancedCurrent(). Source of truth:
// main.cpp:1230-1249.
//
// The ramp only fires when (a) there is at least one active EVSE,
// (b) there is measured headroom (`idifference_dA > 0`), and
// (c) phases_last_update_flag is set (fresh measurement). Otherwise
// the delta is 0.
//
// Inside that window:
//   - IsumImport < 0 (surplus):
//       < -10 dA and `idifference_dA > 10 dA` → +5 dA  (~+0.5 A bump)
//       otherwise                             → +1 dA  (~+0.1 A trickle)
//   - IsumImport >= 0 (over-draw):
//       > 20 dA  → -(IsumImport / 2)   (halve the over-draw)
//       > 10 dA  → -5 dA
//       >  3 dA  → -1 dA
//       <= 3 dA  → 0   (inside ~0.3 A dead band, hold still)
int32_t decide_solar_iset_balanced_delta_dA(uint8_t active_evse,
                                            int32_t idifference_dA,
                                            int32_t isum_import_dA,
                                            bool    phases_last_update_flag);

// The core state machine. Tick granularity is explicit; side effects are
// dispatched through the optional Observer pointer.
class Core {
 public:
  explicit Core(Observer* obs = nullptr) : obs_(obs) { reset(); }

  void set_observer(Observer* obs) { obs_ = obs; }

  void reset();

  // Fast path: called every 10ms by the firmware's Timer10ms task.
  // Consumes Inputs.pilot, evaluates state transitions, updates contactors.
  void tick_10ms(const Inputs& in);

  // Slow path: 1-second tick, used for timers (C1Timer, ChargeDelay).
  void tick_1s();

  const Outputs& outputs() const { return out_; }
  State state() const { return out_.state; }

 private:
  void transition_to(State s, const char* reason, const Inputs& in);
  void apply_entry_effects(State s, const Inputs& in);
  float compute_target_current(const Inputs& in) const;

  Observer* obs_;
  Outputs   out_;

  // Timers carried over from main.cpp.
  uint8_t charge_delay_s_ = 0;  // ChargeDelay  — blocks B→C re-try after fault
  uint8_t c1_timer_s_     = 0;  // C1Timer      — grace period for EV to stop
  bool    pilot_force_disconnected_ = false;
  uint8_t last_errors_    = 0;
};

const char* state_name(State s);
const char* mode_name(Mode m);

}  // namespace evse
