// Host-compilable policy for admitting externally-requested setpoints on
// the SmartEVSE firmware (MQTT Set/* topics today; HTTP /settings later).
//
// File is named `evse_limits.h` not `limits.h` to avoid shadowing the C
// standard `<limits.h>` header that libc++/libstdc++ pull in via
// `<algorithm>` for CHAR_BIT. The firmware build adds src/ as a user
// include path, and angle-bracket includes are allowed to hit user paths
// first on clang/gcc, which caused a fatal build break when this file
// was named limits.h.
//
// Source of truth: SmartEVSE-3/src/esp32.cpp :: mqtt_receive_callback().
// Each helper here is a line-equivalent transliteration of one MQTT
// topic's accept/reject logic; the firmware calls them through
// `firmware_core_shim.h`, and the native harness pulls them in via the
// Makefile so the same decision tree runs against both.

#pragma once

#include <cstdint>

namespace evse::limits {

// Three-way outcome shared by every setpoint-admission helper:
//   Reject : drop silently. Firmware does NOT echo anything back.
//   Clear  : caller requested "0" — reset this knob to its unset state.
//   Apply  : caller requested a valid value — caller should use .value.
enum class Action : uint8_t {
  Reject = 0,
  Clear  = 1,
  Apply  = 2,
};

// Opaque value for Apply; units are per-helper and documented at the
// decide_*() site. Reject/Clear always set value=0 so callers can read
// it unconditionally.
struct Decision {
  Action   action;
  uint16_t value;
};

// Decide how the firmware should react to an incoming /Set/CurrentOverride
// payload. Mirrors esp32.cpp:627-635.
//
//   requested_dA     : payload.toInt() — deci-amps (160 = 16A)
//   min_current_dA   : MinCurrent * 10 (settable, usually 60)
//   max_current_dA   : MaxCurrent * 10 (settable, usually 160)
//   load_balance     : LoadBl  (0=standalone, 1=master, 2+=slave)
//   mode             : MODE_*  (0=Normal, 1=Smart, 2=Solar, 255=Off)
//
// Returns Reject for any disallowed combination, Clear when the request
// is an explicit "0" (regardless of role/mode — matches the firmware),
// Apply otherwise. On Apply, .value is the accepted deci-amps.
Decision decide_current_override(
    uint16_t requested_dA,
    uint16_t min_current_dA,
    uint16_t max_current_dA,
    uint8_t  load_balance,
    uint8_t  mode);

// Decide how the firmware should react to an incoming
// /Set/CurrentMaxSumMains payload. Mirrors esp32.cpp (currently around
// line 636) — note the firmware does NOT publish an echo on this topic.
//
//   requested_a   : payload.toInt() — whole amps (not deci-amps here)
//   load_balance  : LoadBl — slaves (>=2) reject all requests
//
// Accepts 0 (disable the limit entirely → Clear) or any value in the
// range [10, 600] A (Apply). Out-of-range and slave → Reject. On Apply,
// .value is the accepted amps.
Decision decide_max_sum_mains(
    uint16_t requested_a,
    uint8_t  load_balance);

// Decide how the firmware should react to an incoming /Set/CPPWMOverride
// payload. Mirrors esp32.cpp:647-661. Input is signed because `-1`
// carries special meaning (release the override).
//
//   -1       → Clear. Release override; firmware restores SetCPDuty(1024)
//              and PILOT_CONNECTED; CPDutyOverride := false.
//   0        → Apply with value=0. Force pilot disconnected
//              (SetCPDuty(0), PILOT_DISCONNECTED, CPDutyOverride := true).
//   1..1024  → Apply with value=requested. Raw 1024-scale duty written
//              directly; PILOT_CONNECTED; CPDutyOverride := true.
//   else     → Reject. Firmware silently drops. The legacy inline code
//              actually UB'd on pwm<-1 (negative int cast to uint32_t
//              passed through to ledcWrite). The helper fixes that by
//              treating anything in {pwm != -1, pwm != 0, pwm > 1024}
//              as Reject. See FEEDBACK §8.
//
// On Apply, caller derives `pilot_connected = (decision.value > 0)`.
Decision decide_cp_pwm_override(int32_t requested);

// ---------------------------------------------------------------------
// /Set/Mode has a richer action space than the 3-way Action enum: five
// accepted payload strings dispatch to two distinct firmware functions
// (setMode vs setAccess), with Solar also triggering a side-effect
// override-clear at the call site. Give it its own Decision type so the
// intent stays readable.
// ---------------------------------------------------------------------
enum class ModeAction : uint8_t {
  Reject      = 0,
  ApplyMode   = 1,  // caller: setMode(target)
  ApplyAccess = 2,  // caller: setAccess(target)
};

// Target constants — bit-identical to main.h. Duplicated here so
// evse_limits doesn't have to pull in main.h (and its enum-member/macro
// collision surface, see firmware_core_shim.h's preamble).
//   main.h :: MODE_NORMAL=0, MODE_SMART=1, MODE_SOLAR=2
//   main.h :: AccessStatus_t { OFF=0, ON=1, PAUSE=2 }
constexpr uint8_t kModeNormal  = 0;
constexpr uint8_t kModeSmart   = 1;
constexpr uint8_t kModeSolar   = 2;
constexpr uint8_t kAccessOff   = 0;
constexpr uint8_t kAccessPause = 2;

struct ModeDecision {
  ModeAction action;
  uint8_t    target;
};

// Decide how the firmware should react to an incoming /Set/Mode payload.
// Mirrors esp32.cpp:606-621.
//
//   "Off"    → ApplyAccess(OFF)    — caller must then setAccess(OFF).
//                                    v4 firmware also resets modem timers.
//   "Normal" → ApplyMode(NORMAL)
//   "Solar"  → ApplyMode(SOLAR)    — caller must ALSO setOverrideCurrent(0)
//                                    before (or after) setMode. This is the
//                                    only accept arm with a side effect
//                                    outside the helper's contract.
//   "Smart"  → ApplyMode(SMART)
//   "Pause"  → ApplyAccess(PAUSE)
//   else     → Reject. Firmware drops silently.
//
// Payload is passed as (ptr, len) so the C++ side can use string_view-like
// semantics without requiring std::string in the C ABI bridge.
ModeDecision decide_set_mode(const char* payload, uint32_t len);

// Decide how the firmware should react to an incoming /Set/EnableC2
// payload. Mirrors esp32.cpp:798-816. Accepts EITHER:
//   - a digit "0".."4"  → Apply, value = parsed digit
//   - one of the 5 strings "Not present", "Always Off", "Solar Off",
//     "Always On", "Auto" → Apply, value = index in that list
// Anything else → Reject.
//
// Target enum values for main.h::EnableC2_t:
//   0 = NOT_PRESENT, 1 = ALWAYS_OFF, 2 = SOLAR_OFF,
//   3 = ALWAYS_ON,   4 = AUTO
//
// The firmware's legacy inline code ALWAYS called
// `request_write_settings()` after hitting this branch — even on
// rejected input. The extracted helper only encodes admission; the
// call site keeps the unconditional write-settings behavior to avoid a
// behavior change, but see FEEDBACK §9 for the concern that this lets
// MQTT-reachable attackers trigger flash writes on garbage payloads.
Decision decide_enable_c2(const char* payload, uint32_t len);

// ---------------------------------------------------------------------
// Multi-value meter payloads — /Set/MainsMeter, /Set/EVMeter, etc. —
// need a richer return type than the single-value Action/Decision.
// ---------------------------------------------------------------------
struct MeterDecision {
  Action  action;
  int16_t l1;       // phase 1 current, deci-amps (signed: mains can be negative)
  int16_t l2;
  int16_t l3;
};

// Decide how the firmware should react to an incoming /Set/MainsMeter
// payload. Mirrors esp32.cpp:672-692.
//
//   payload         : expected "L1:L2:L3" of three decimal ints.
//   mains_is_em_api : true iff MainsMeter.Type == EM_API (meter.h=9).
//                     The firmware requires an API-configured mains
//                     meter to accept externally-fed readings.
//   load_balance    : LoadBl — slaves (>=2) reject all payloads.
//
// Accept conditions: EM_API, master/standalone, parse yields exactly
// three ints, each in the open interval (-2000, 2000) dA (= -200 A to
// +200 A exclusive). On Apply, {l1, l2, l3} are the parsed values; the
// call site is responsible for writing them to MainsMeter.Irms[] and
// calling CalcIsum() (v3) or forwarding to Serial1 (v4) — that
// downstream is a side effect, not an admission decision.
//
// No Clear action: there is no "disable the mains meter" payload. Any
// invalid input silently rejects.
MeterDecision decide_mains_meter(
    const char* payload,
    uint32_t    len,
    bool        mains_is_em_api,
    uint8_t     load_balance);

// ---------------------------------------------------------------------
// /Set/EVMeter is richer than /Set/MainsMeter: five fields
// (L1:L2:L3:W:WH) with three *independent* partial-apply arms — the
// caller can update currents, power, and energy separately by sending
// `-1` on the fields it wants left untouched. Give it its own Decision
// type to model the tri-arm admit/skip pattern cleanly.
// ---------------------------------------------------------------------
struct EvMeterDecision {
  Action  action;           // Reject (parse/role failed) or Apply (n==5).
  bool    apply_currents;   // True iff all three currents in [0, 999] dA.
  int16_t l1;               // dA; 0..999 when apply_currents.
  int16_t l2;
  int16_t l3;
  bool    apply_power;      // True iff W > -1.
  int32_t power_w;          // Watts; valid when apply_power.
  bool    apply_energy;     // True iff WH > -1.
  int32_t energy_wh;        // Cumulative import Wh; valid when apply_energy.
};

// Decide how the firmware should react to an incoming /Set/EVMeter
// payload. Mirrors esp32.cpp:695-732.
//
//   payload       : expected "L1:L2:L3:W:WH" of five decimal ints.
//   ev_is_em_api  : true iff EVMeter.Type == EM_API. Unlike MainsMeter,
//                   EVMeter has NO LoadBl gate — any role can feed.
//
// Accept conditions:
//   parse yields exactly five ints          → Apply (else Reject).
//   currents: each L{1,2,3} in open-interval (-1, 1000) dA
//             → apply_currents=true, l1/l2/l3 populated.
//             Any negative L*, or any >= 1000, gates the whole currents
//             block off (not just that field). -1 is the documented
//             "skip currents this message" sentinel.
//   power:   W > -1 → apply_power=true, power_w=W.
//   energy:  WH > -1 → apply_energy=true, energy_wh=WH. Firmware's
//             Export_active_energy is zeroed on Apply — a side effect
//             the call site keeps, not the helper.
//
// On Reject, all apply_* flags are false and all values are zero.
// No Clear action: there is no "disable the EV meter" payload.
EvMeterDecision decide_ev_meter(
    const char* payload,
    uint32_t    len,
    bool        ev_is_em_api);

// Decide how the firmware should react to an incoming /Set/CableLock
// payload. Mirrors esp32.cpp:798-804. Legacy contract:
//   "1"    → CableLock := 1
//   other  → CableLock := 0   (including "0", empty, "garbage")
//   always calls request_write_settings() — even on unknown input.
//
// The helper tightens admission to exactly { "0", "1" }: unknown input
// is Reject. The call site keeps the unconditional
// request_write_settings() call to preserve the flash-write contract
// (see FEEDBACK §11 — this is the second instance of the pattern flagged
// in §9 for EnableC2). Callers should not treat Reject as a no-op at
// the call site; the flash write still fires.
Decision decide_cable_lock(const char* payload, uint32_t len);

// ---------------------------------------------------------------------
// /Set/HomeBatteryCurrent stores a *signed* int16 (negative = battery
// discharging / exporting to EV). The existing Decision type holds
// uint16_t, so give this helper its own return struct rather than
// encode sign via two's-complement.
// ---------------------------------------------------------------------
struct SignedDecision {
  Action  action;
  int16_t value;  // valid when action==Apply; int16_t matches main.cpp
};

// Decide how the firmware should react to an incoming
// /Set/HomeBatteryCurrent payload. Mirrors esp32.cpp:734-741.
//
//   payload        : signed decimal int (deci-amps per main.cpp's %d.%d log).
//   load_balance   : LoadBl — slaves (>=2) reject all payloads.
//
// Accept conditions: master/standalone + parse yields exactly one
// signed int that fits in int16 (helper saturates on overflow via
// parse_signed_int). The legacy inline code did `payload.toInt()` with
// no range check, silently writing 0 on non-numeric input (e.g.
// "garbage" → 0) AND advancing the homeBatteryLastUpdate timestamp
// either way. See FEEDBACK §10.
//
// The helper tightens that: non-numeric or empty payloads now Reject.
// The timestamp update at the call site is gated on Apply, matching
// the tightened contract.
SignedDecision decide_home_battery_current(
    const char* payload,
    uint32_t    len,
    uint8_t     load_balance);

// ---------------------------------------------------------------------
// /Set/Color{Off,Normal,Smart,Solar,Custom} all share the same admission
// shape: sscanf("%d,%d,%d", &R, &G, &B) + range check 0..255 per channel.
// One helper serves all five call sites; the caller routes the accepted
// {r,g,b} to the correct ColorXxx[3] global.
// ---------------------------------------------------------------------
struct ColorDecision {
  Action  action;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Decide how the firmware should react to an incoming /Set/Color*
// payload. Mirrors esp32.cpp:757-806 (five copy-pasted branches).
//
//   payload : expected "R,G,B" of three decimal ints.
//
// Accept conditions: parse yields exactly three ints (comma-separated),
// each in [0, 256). Anything else → Reject, all channels zeroed. No
// LoadBl gate, no EM_API gate, no role check — the RGB knobs are
// cosmetic and always settable.
//
// No Clear action: there is no "disable color" payload — the firmware
// treats any color array as always-valid and rendered.
ColorDecision decide_color_rgb(const char* payload, uint32_t len);

// Decide how the firmware should react to an incoming /Set/CustomButton
// payload. Mirrors esp32.cpp::/Set/CustomButton branch (two lines):
//   payload == "On" → CustomButton := true
//   else            → CustomButton := false
//
// This is a pure two-state toggle with no Reject arm — any non-"On"
// string is treated as "Off". The helper preserves that contract
// bit-identically (no tightening); .value is 1 on "On", 0 on anything
// else. Always returns Apply.
Decision decide_custom_button(const char* payload, uint32_t len);

// ---------------------------------------------------------------------
// /Set/RFID admission: parse a hex-encoded UID, validate length, and
// emit a ready-to-compare 8-byte RFID record. Returns the full record
// (incl. family code and Dallas CRC-8) so the caller just copies it
// into the RFID[8] global and triggers CheckRFID().
// ---------------------------------------------------------------------
struct RfidDecision {
  Action  action;
  uint8_t uid[8];  // family code byte, 6-7 UID bytes, CRC at [7].
};

// Decide how the firmware should react to an incoming /Set/RFID
// payload. Mirrors esp32.cpp:836-887.
//
//   payload         : hex string of 12 or 14 chars (6 or 7 UID bytes).
//                     Whitespace is trimmed from both ends before
//                     validation, matching the legacy String::trim().
//   reader_enabled  : true iff getItemValue(MENU_RFIDREADER) != 0.
//                     The firmware silently drops the request (with a
//                     log line) when the RFID reader is disabled.
//
// Accept conditions: reader enabled + payload is all hex digits after
// trim + length ∈ {12, 14}. Length 12 encodes a legacy 6-byte UID with
// family code 0x01 placed at uid[0] and the 6 payload bytes at uid[1..6];
// length 14 encodes a modern 7-byte UID at uid[0..6] with no family
// prefix. Either way, uid[7] is the Dallas CRC-8 of uid[0..6]
// (polynomial 0x8C, identical to utils.cpp::crc8).
//
// Reject on: reader disabled, empty payload after trim, non-hex chars,
// length neither 12 nor 14. On Reject, uid[] is zeroed.
RfidDecision decide_rfid_uid(const char* payload, uint32_t len, bool reader_enabled);

}  // namespace evse::limits
