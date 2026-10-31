// Flat C-style bridge into firmware_core.cpp for callers that cannot
// safely include firmware_core.h.
//
// Why this exists:
//   main.h defines STATE_A, PILOT_12V, MODE_SMART, PWM_100, ... as
//   preprocessor macros. firmware_core.h declares the same names as
//   `enum` members inside `namespace evse`. If both headers were pulled
//   into the same translation unit, the preprocessor would rewrite the
//   enum member `STATE_A = 0` to `0 = 0` — a syntax error.
//
//   firmware main.cpp already includes main.h pervasively, so this
//   shim exposes only the helpers main.cpp needs, as plain typed
//   integer functions with no evse:: types in the interface.
//
// Contract: every function here is a thin forward to its namespaced
// counterpart in firmware_core.cpp. See firmware_core.h / .cpp for the
// actual algorithm and the annotated source-of-truth map back to
// main.cpp.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Decode the IEC 61851 pilot level from the min/max of a window of CP
// ADC samples (millivolts, post-divider). Return value matches the
// PILOT_* macros in main.h (12, 9, 6, 3, 1, 0, 255).
uint8_t evse_decode_pilot_mv(uint32_t min_mv, uint32_t max_mv);

// Duty value for SetCPDuty() on SMARTEVSE v3 (ESP32) given a requested
// charge current in deci-amps (e.g. 160 = 16 A). Returns 1024-scale duty
// as expected by the v3 PWM driver. Preserves the firmware's exact
// truncation semantics (float→integer cast, not rounding) and fallback
// to 6 A duty on out-of-range input — changing either changes the
// advertised current on the pilot in a way EVs notice.
//
// min_current_dA mirrors the `MIN_CURRENT * 10` check at the call site,
// which can be overridden via -DMIN_CURRENT=5 at build time (see
// docs/building_flashing.md).
uint16_t evse_cp_duty_for_current_v3(uint16_t current_dA, uint16_t min_current_dA);

// Decide whether to charge single-phase given (EnableC2_t, Mode, phase count).
// Pure function — no side effects. Returns 1 for 1P, 0 for 3P (default).
// Source of truth: main.cpp:686 Force_Single_Phase_Charging().
uint8_t evse_force_single_phase_charging(uint8_t c2, uint8_t mode, uint8_t nr_phases_charging);

// Decide the 1P↔3P phase-switching action for the given (EnableC2, Mode,
// State, Nr_Of_Phases_Charging, Switching_Phases_C2) state vector. Pure
// decision function — caller writes the globals and emits the legacy
// _LOG_A line based on the returned reason code. See PhaseSwitchReason
// in firmware_core.h for the reason→side-effect map.
// Source of truth: main.cpp:547 CheckSwitchingPhases().
//
// Return codes (match evse::PhaseSwitchReason):
//   0 = NOOP                  — no writes, no log
//   1 = STAGE_1P              — Switching_Phases_C2 := GOING_TO_SWITCH_1P; log
//   2 = ALREADY_1P            — Switching_Phases_C2 := NO_SWITCH;          log
//   3 = STAGE_3P              — Switching_Phases_C2 := GOING_TO_SWITCH_3P; log
//   4 = ALREADY_3P            — Switching_Phases_C2 := NO_SWITCH;          log
//   5 = SET_1P_IMMEDIATE      — Nr_Of_Phases_Charging := 1                (no _LOG_A)
//   6 = SET_3P_IMMEDIATE      — Nr_Of_Phases_Charging := 3                (no _LOG_A)
//   7 = AUTO_STAGE_3P         — Switching_Phases_C2 := GOING_TO_SWITCH_3P; log "AUTO …"
//   8 = AUTO_CANCEL           — Switching_Phases_C2 := NO_SWITCH;          log "Cancelled!"
uint8_t evse_check_switching_phases(uint8_t c2, uint8_t mode, uint8_t state,
                                    uint8_t nr_phases_charging,
                                    uint8_t switching_phases_c2);

// Decide STATE_C entry phase action. Source of truth: main.cpp:872-898.
// Returns 1 iff CONTACTOR2 should engage and Nr_Of_Phases_Charging should
// be finalized to 3; returns 0 iff CONTACTOR2 should release and
// Nr_Of_Phases_Charging should be finalized to 1. Caller still performs
// all side-effects around this bool: CONTACTOR1_ON, the two-way
// CONTACTOR2 write, writing Nr_Of_Phases_Charging, SEND_TO_ESP32,
// setSolarStopTimer(0), MaxSumMainsTimer=0, Switching_Phases_C2:=NO_SWITCH.
// `switching_phases_c2` values follow Switch_Phase_t (main.h:325):
// 0=NO_SWITCH, 1=GOING_TO_SWITCH_1P, 2=GOING_TO_SWITCH_3P.
uint8_t evse_decide_state_c_contactor2(uint8_t c2, uint8_t mode,
                                       uint8_t nr_of_phases_charging,
                                       uint8_t switching_phases_c2);

// Decide target state for setStatePowerUnavailable(). Source of truth:
// main.cpp:729. Returns the target STATE_* value; caller must call
// setState(target) only when target != current state — the helper
// encodes the function's early-returns as same-state fixpoints
// (STATE_A→A, STATE_C1→C1, STATE_B1→B1).
uint8_t evse_decide_state_power_unavailable(uint8_t state);

// Decide target state when setAccess() transitions away from ON.
// Source of truth: main.cpp:956-966. Returns the target STATE_*
// value; caller must call setState(target) only when target != current
// state — fixpoints (access==ON, STATE_C1, STATE_A, STATE_B1, STATE_D)
// reproduce the legacy no-op branches.
uint8_t evse_decide_state_on_access_change(uint8_t access, uint8_t state);

// Decide whether setMode() should stage a CP disconnect. Source of
// truth: main.cpp:617-637. Returns a code matching evse::ModeSwitchReason:
//   0 = NOOP                       — no disconnect, no log
//   1 = SOLAR_OFF_C2_BOUNDARY      — _LOG_A "Disconnect CP when switching C2\n"
//   2 = AUTO_SOLAR_EXIT_1P         — _LOG_A "AUTO Solar->Smart/Normal charging 1p->3p\n"
// Caller dispatches the reason to the corresponding log + a call
// to evse_decide_cp_disconnect_target_on_mode_switch(State).
uint8_t evse_decide_mode_switch_cp_reason(uint8_t c2, uint8_t old_mode,
                                          uint8_t new_mode,
                                          uint8_t nr_of_phases_charging);

// State-map helper for the setMode() CP-disconnect blocks. Returns
// STATE_C1 on STATE_C, STATE_B1 on STATE_B, otherwise `state` (no-op).
uint8_t evse_decide_cp_disconnect_target_on_mode_switch(uint8_t state);

// Decide whether CalcBalancedCurrent() should stage a 3P or 1P solar
// start. Source of truth: main.cpp:1186-1205. Returns a code matching
// evse::SolarStartPhaseReason:
//   0 = NOOP      — outside trigger window OR already at target phase count
//   1 = STAGE_3P  — write Switching_Phases_C2 = GOING_TO_SWITCH_3P; _LOG_D
//   2 = STAGE_1P  — write Switching_Phases_C2 = GOING_TO_SWITCH_1P; _LOG_D
// Only fires when mode==MODE_SOLAR && state==STATE_B && c2==AUTO.
// `isum_dA` is negative when mains are exporting (solar surplus).
uint8_t evse_decide_solar_start_phase(uint8_t mode, uint8_t state,
                                      uint8_t c2, int32_t isum_dA,
                                      uint16_t min_current_a,
                                      uint8_t nr_of_phases_charging);

// Decide whether IsCurrentAvailable() should admit another EVSE in
// Solar mode. Source of truth: main.cpp:1052-1065. Returns a code
// matching evse::SolarAdmissionReason:
//   0 = ALLOW              — non-Solar OR all three Solar guards cleared
//   1 = DENY_NO_SURPLUS    — no active EVSE and Isum >= -StartCurrent*10
//   2 = DENY_CANT_SPLIT    — ActiveEVSE*MinCurrent*10 > TotalCurrent
//   3 = DENY_OVER_IMPORT   — admitting one more exceeds ImportCurrent budget
// Any non-zero reason means the legacy code returned 0 from
// IsCurrentAvailable() with a _LOG_D debug trace — caller should do
// the same. Currents: isum_dA / total_current_dA in deci-amps;
// start_current / min_current / import_current in whole amps.
uint8_t evse_decide_solar_admission(uint8_t mode, uint8_t active_evse,
                                    int32_t isum_dA,
                                    uint16_t start_current_a,
                                    uint16_t min_current_a,
                                    uint16_t import_current_a,
                                    int32_t total_current_dA);

// Decide whether admitting one more EVSE would exceed any configured
// circuit limit (MaxMains / MaxCircuit / MaxSumMains). Source of truth:
// main.cpp:1065-1084. Return codes (evse::CircuitAdmissionReason):
//   0 = ALLOW             — none of the guards fire
//   1 = DENY_MAX_MAINS    — ActiveEVSE*MinCurrent*10 + Baseload > MaxMains*10
//   2 = DENY_MAX_CIRCUIT  — ActiveEVSE*MinCurrent*10 + Baseload_EV > MaxCircuit*10
//   3 = DENY_MAX_SUM_MAINS — Phases*ActiveEVSE*MinCurrent*10 + Isum > MaxSumMains*10
// Any non-zero reason means IsCurrentAvailable() should return 0.
// `active_evse` must include the hypothetical new EVSE (+1). `phases`
// is 1 by default, 1 or 3 from Force_Single_Phase_Charging() when
// LoadBl==0. `max_sum_mains_a == 0` disables the MaxSumMains guard.
// Normal mode: only MaxCircuit can fire, and only when LoadBl==1.
uint8_t evse_decide_circuit_admission(uint8_t mode, uint8_t active_evse,
                                      int32_t baseload_dA,
                                      int32_t baseload_ev_dA,
                                      int32_t isum_dA,
                                      uint16_t min_current_a,
                                      uint16_t max_mains_a,
                                      uint16_t max_circuit_a,
                                      uint16_t max_sum_mains_a,
                                      uint8_t load_balance,
                                      uint8_t ev_meter_type,
                                      uint8_t phases);

// Compute the IsetBalanced delta (deci-amps) applied by the Solar ramp
// inside CalcBalancedCurrent(). Source of truth: main.cpp:1230-1249.
// Returns 0 when the ramp is gated off (no active EVSE, no headroom,
// or measurements not fresh). Otherwise +1 or +5 dA for surplus and
// {0, -1, -5, -IsumImport/2} dA for over-draw.
// `phases_last_update_flag` is passed as uint8_t because extern "C"
// doesn't see C++ bool — 0 means "stale", any other value means "fresh".
int32_t evse_decide_solar_iset_balanced_delta_dA(uint8_t active_evse,
                                                 int32_t idifference_dA,
                                                 int32_t isum_import_dA,
                                                 uint8_t phases_last_update_flag);

// Decide action on an MQTT /Set/CurrentOverride request. See limits.h.
// Return codes: 0=Reject (drop), 1=Clear (setOverrideCurrent(0)), 2=Apply
// (setOverrideCurrent(*out_value_dA)). out_value_dA is always written, set
// to 0 on Reject/Clear so callers can read it unconditionally.
//
// Keeps the firmware's silent-drop semantics on out-of-range input, on
// LoadBl>=2 (slave), and on modes other than Normal/Smart. Accepts exactly
// the same [MinCurrent*10, MaxCurrent*10] window as esp32.cpp did inline.
uint8_t evse_limits_decide_current_override(uint16_t requested_dA,
                                            uint16_t min_current_dA,
                                            uint16_t max_current_dA,
                                            uint8_t  load_balance,
                                            uint8_t  mode,
                                            uint16_t* out_value_dA);

// Decide action on an MQTT /Set/CurrentMaxSumMains request. See
// evse_limits.h. Return codes: 0=Reject (drop), 1=Clear (MaxSumMains=0),
// 2=Apply (MaxSumMains=*out_value_a). Input is whole amps, not deci-amps.
// Slaves (LoadBl>=2) reject all requests.
uint8_t evse_limits_decide_max_sum_mains(uint16_t requested_a,
                                         uint8_t  load_balance,
                                         uint16_t* out_value_a);

// Decide action on an MQTT /Set/CPPWMOverride request. See evse_limits.h.
// Return codes: 0=Reject (drop, pwm outside {-1, 0..1024}), 1=Clear
// (pwm==-1, restore normal PWM: SetCPDuty(1024), pilot connected,
// CPDutyOverride:=false), 2=Apply (*out_value is the 1024-scale duty to
// write; pilot_connected = *out_value > 0; CPDutyOverride := true).
// Input is signed because -1 has special release-override meaning.
uint8_t evse_limits_decide_cp_pwm_override(int32_t requested,
                                           uint16_t* out_value);

// Decide action on an MQTT /Set/Mode request. See evse_limits.h.
// Return codes: 0=Reject (drop), 1=ApplyMode (setMode(*out_target)),
// 2=ApplyAccess (setAccess(*out_target)). *out_target holds a
// main.h-compatible MODE_* or AccessStatus_t value (never mixed — which
// one depends on the action code). Callers must handle the Solar
// side-effect (setOverrideCurrent(0)) at the call site when they see
// action==1 && *out_target==MODE_SOLAR.
uint8_t evse_limits_decide_set_mode(const char* payload,
                                    uint32_t len,
                                    uint8_t* out_target);

// Decide action on an MQTT /Set/EnableC2 request. See evse_limits.h.
// Return codes: 0=Reject (drop), 2=Apply (EnableC2 = (EnableC2_t)*out_value).
// 1 (Clear) is never returned — EnableC2 has no disable-sentinel.
// *out_value is a EnableC2_t-compatible index in [0, 4].
uint8_t evse_limits_decide_enable_c2(const char* payload,
                                     uint32_t len,
                                     uint16_t* out_value);

// Decide action on an MQTT /Set/MainsMeter request. See evse_limits.h.
// Return codes: 0=Reject (drop), 2=Apply (write *out_l1/l2/l3 into
// MainsMeter.Irms[] on v3 / forward via Serial1 on v4). 1 (Clear) is
// never returned — there is no "disable mains meter" payload. Payload
// is expected to be "L1:L2:L3" deci-amps; invalid parse, range, or an
// out-of-role caller all silently reject. *out_l{1,2,3} are always
// written (zeroed on Reject).
uint8_t evse_limits_decide_mains_meter(const char* payload,
                                       uint32_t len,
                                       uint8_t mains_is_em_api,
                                       uint8_t load_balance,
                                       int16_t* out_l1,
                                       int16_t* out_l2,
                                       int16_t* out_l3);

// Decide action on an MQTT /Set/EVMeter request. See evse_limits.h.
// Return codes: 0=Reject (drop), 2=Apply. On Apply, *out_flags is a
// bitmask of live arms (bit0=currents, bit1=power, bit2=energy) —
// callers must check each bit before writing its corresponding output
// because /Set/EVMeter admits each arm independently.
// Decide action on an MQTT /Set/CableLock request. See evse_limits.h.
// Return codes: 0=Reject (unknown payload), 2=Apply (*out_value is 0 or 1).
// Legacy firmware called request_write_settings() unconditionally; the
// call site keeps that for bit-identity — see FEEDBACK §11.
uint8_t evse_limits_decide_cable_lock(const char* payload,
                                      uint32_t len,
                                      uint8_t* out_value);

// Decide action on an MQTT /Set/HomeBatteryCurrent request. See
// evse_limits.h. Return codes: 0=Reject (slave or parse fail), 2=Apply
// (*out_value is the signed int16 to write to homeBatteryCurrent).
// Legacy allowed non-numeric payloads to silently write 0 and advance
// homeBatteryLastUpdate — see FEEDBACK §10. Tightened here.
uint8_t evse_limits_decide_home_battery_current(const char* payload,
                                                uint32_t len,
                                                uint8_t load_balance,
                                                int16_t* out_value);

uint8_t evse_limits_decide_ev_meter(const char* payload,
                                    uint32_t len,
                                    uint8_t ev_is_em_api,
                                    uint8_t* out_flags,
                                    int16_t* out_l1,
                                    int16_t* out_l2,
                                    int16_t* out_l3,
                                    int32_t* out_power_w,
                                    int32_t* out_energy_wh);

// Decide action on an MQTT /Set/Color{Off,Normal,Smart,Solar,Custom}
// request. See evse_limits.h. Return codes: 0=Reject (parse or range
// fail), 2=Apply (write *out_r/g/b into the appropriate ColorXxx[3]
// global). Payload is expected to be "R,G,B" with each channel in
// [0, 256). One helper serves all five topics — the caller picks
// which ColorXxx array receives the accepted triple.
uint8_t evse_limits_decide_color_rgb(const char* payload,
                                     uint32_t len,
                                     uint8_t* out_r,
                                     uint8_t* out_g,
                                     uint8_t* out_b);

// Decide action on an MQTT /Set/CustomButton request. See evse_limits.h.
// Always returns 2 (Apply); *out_value is 1 on exact-match "On", 0 on
// anything else. Bit-identical to the legacy 2-line toggle.
uint8_t evse_limits_decide_custom_button(const char* payload,
                                         uint32_t len,
                                         uint8_t* out_value);

// Decide action on an MQTT /Set/RFID request. See evse_limits.h.
// Return codes: 0=Reject (reader disabled / bad hex / wrong length),
// 2=Apply (out_uid holds 8 bytes: family code / UID / Dallas CRC-8).
// Caller copies out_uid into the RFID[8] global and calls CheckRFID().
uint8_t evse_limits_decide_rfid_uid(const char* payload,
                                    uint32_t len,
                                    uint8_t reader_enabled,
                                    uint8_t* out_uid);

#ifdef __cplusplus
}
#endif
