// evse_limits.cpp — see evse_limits.h for the source-of-truth map.

#include "evse_limits.h"
#include "firmware_core_shim.h"

#include <cstring>

namespace evse::limits {

// Mirrors esp32.cpp::mqtt_receive_callback /Set/CurrentOverride branch
// verbatim. The only transformation is refactoring the condition into
// the three-way Action enum so the harness can observe reject vs accept
// without parsing output state.
Decision decide_current_override(
    uint16_t requested_dA,
    uint16_t min_current_dA,
    uint16_t max_current_dA,
    uint8_t  load_balance,
    uint8_t  mode) {
  if (requested_dA == 0) {
    return {Action::Clear, 0};
  }
  // LoadBl >= 2 means this unit is a slave — override is master-only.
  if (load_balance >= 2) {
    return {Action::Reject, 0};
  }
  // main.h MODE_NORMAL=0, MODE_SMART=1. Solar (2) and Off (255) reject.
  constexpr uint8_t kNormal = 0;
  constexpr uint8_t kSmart  = 1;
  if (mode != kNormal && mode != kSmart) {
    return {Action::Reject, 0};
  }
  if (requested_dA < min_current_dA || requested_dA > max_current_dA) {
    return {Action::Reject, 0};
  }
  return {Action::Apply, requested_dA};
}

// Mirrors esp32.cpp /Set/CurrentMaxSumMains. The firmware had:
//
//   if (RequestedCurrent == 0)          MaxSumMains = 0;
//   else if (RequestedCurrent == 0 ||
//            (RequestedCurrent >= 10 && RequestedCurrent <= 600))
//                                         MaxSumMains = RequestedCurrent;
//
// The second `== 0` is dead (outer if already handled it — FEEDBACK §4).
// This helper collapses it to a clean three-way decision.
Decision decide_max_sum_mains(uint16_t requested_a, uint8_t load_balance) {
  if (load_balance >= 2) {
    return {Action::Reject, 0};
  }
  if (requested_a == 0) {
    return {Action::Clear, 0};
  }
  if (requested_a < 10 || requested_a > 600) {
    return {Action::Reject, 0};
  }
  return {Action::Apply, requested_a};
}

// Mirrors esp32.cpp /Set/CPPWMOverride. Legacy inline code:
//
//   int pwm = payload.toInt();
//   if (pwm == -1)       { SetCPDuty(1024); PILOT_CONNECTED;    CPDutyOverride = false; }
//   else if (pwm == 0)   { SetCPDuty(0);    PILOT_DISCONNECTED; CPDutyOverride = true;  }
//   else if (pwm <= 1024){ SetCPDuty(pwm);  PILOT_CONNECTED;    CPDutyOverride = true;  }
//
// The original chain only constrained pwm from above (pwm <= 1024); it
// did NOT reject pwm < -1. That means e.g. pwm = -2 fell through the
// final `else if (pwm <= 1024)` (true — -2 <= 1024) and got passed to
// SetCPDuty, which takes uint32_t — implicit conversion of -2 to
// uint32_t is 0xFFFFFFFE, passed through to ledcWrite as a duty value.
// Whether that wraps, asserts, or silently clamps is platform-defined
// (ledc accepts up to 2^duty_res - 1, typically 1023 at 10-bit). Either
// way, "negative duty" is nonsense input that the legacy code accepted.
//
// This helper tightens the contract: only {-1, 0, 1..1024} are accepted;
// anything else is a silent Reject. See FEEDBACK §8.
Decision decide_cp_pwm_override(int32_t requested) {
  if (requested == -1) {
    return {Action::Clear, 0};
  }
  if (requested == 0) {
    return {Action::Apply, 0};
  }
  if (requested >= 1 && requested <= 1024) {
    return {Action::Apply, static_cast<uint16_t>(requested)};
  }
  return {Action::Reject, 0};
}

// Mirrors esp32.cpp /Set/Mode dispatch. Exact string match, case
// sensitive (matches the firmware's `payload == "Off"` comparisons).
// Anything not on the accept list silently rejects. Per-arm semantics
// are documented in evse_limits.h.
//
// Payload length is passed in so we can reject strings that merely
// share a prefix with an accept string (e.g. "Offline" must not become
// "Off"). The firmware's String::operator== is exact-match so that's
// the contract to preserve.
// Mirrors esp32.cpp /Set/EnableC2. Two accept paths:
//   digit branch : payload[0] is '0'..'4' AND the parsed int is <=4.
//                  The parse is equivalent to Arduino String::toInt(),
//                  which stops at the first non-digit and returns 0 on
//                  empty input. We replicate via a small hand-rolled
//                  parser to avoid atoi/strtol locale surprises and to
//                  reject extra trailing garbage explicitly.
//   string branch: payload exactly matches one of the five
//                  main.cpp::StrEnableC2 entries (case-sensitive).
//
// Pinned StrEnableC2 values, from SmartEVSE-3/src/main.cpp:329
//   { "Not present", "Always Off", "Solar Off", "Always On", "Auto" }
namespace {

// Parse a signed decimal int from (payload+i) up to the first non-digit
// (or end-of-buffer), matching `sscanf("%d", ...)`. Returns true if at
// least one digit was consumed. On success, writes the value to
// *out_value and advances *inout_i past the last digit (not past the
// separator). An optional leading '-' is supported; '+' is too (sscanf
// accepts it).
bool parse_signed_int(const char* payload, uint32_t len, uint32_t* inout_i, int32_t* out_value) {
  uint32_t i = *inout_i;
  if (i >= len) return false;
  bool negative = false;
  if (payload[i] == '-') { negative = true; ++i; }
  else if (payload[i] == '+') { ++i; }
  if (i >= len) return false;
  if (payload[i] < '0' || payload[i] > '9') return false;
  int64_t acc = 0;
  while (i < len && payload[i] >= '0' && payload[i] <= '9') {
    acc = acc * 10 + (payload[i] - '0');
    // Saturate at int32 to avoid overflow when callers pass huge payloads.
    if (acc > 0x7FFFFFFF) { acc = 0x7FFFFFFF; while (i < len && payload[i] >= '0' && payload[i] <= '9') ++i; break; }
    ++i;
  }
  *inout_i = i;
  *out_value = static_cast<int32_t>(negative ? -acc : acc);
  return true;
}

}  // namespace

// Mirrors esp32.cpp /Set/MainsMeter admission. The legacy code used
// sscanf("%d:%d:%d"); we replicate with a small hand-rolled parser so
// the helper has no libc dependency at host-test time beyond <cstring>
// and so the parse rules are inspectable. Acceptance is: exactly three
// signed ints separated by single ':' characters, nothing after the
// third number (save for trailing whitespace to match sscanf's lenient
// behavior).
MeterDecision decide_mains_meter(
    const char* payload,
    uint32_t    len,
    bool        mains_is_em_api,
    uint8_t     load_balance) {
  MeterDecision reject{Action::Reject, 0, 0, 0};
  if (!mains_is_em_api) return reject;
  if (load_balance >= 2) return reject;
  if (payload == nullptr || len == 0) return reject;

  uint32_t i = 0;
  int32_t v[3] = {0, 0, 0};
  for (int k = 0; k < 3; ++k) {
    if (!parse_signed_int(payload, len, &i, &v[k])) return reject;
    if (k < 2) {
      // Require a ':' separator between values. sscanf treated ':' as a
      // literal match token, so exactly one colon is required here too.
      if (i >= len || payload[i] != ':') return reject;
      ++i;
    }
  }
  // Match the firmware's open-interval check: (-2000, 2000) = -199.9..199.9 A.
  for (int k = 0; k < 3; ++k) {
    if (v[k] <= -2000 || v[k] >= 2000) return reject;
  }
  return {Action::Apply,
          static_cast<int16_t>(v[0]),
          static_cast<int16_t>(v[1]),
          static_cast<int16_t>(v[2])};
}

// Mirrors esp32.cpp /Set/EVMeter admission. Shares the hand-rolled
// `parse_signed_int` with /Set/MainsMeter; the two topics differ only
// in field count (3 vs 5), range (mains signed ±2000 dA vs EV
// unsigned 0..999 dA with `-1` skip sentinels on every field), and
// role gating (mains rejects slaves; EV does not).
EvMeterDecision decide_ev_meter(
    const char* payload,
    uint32_t    len,
    bool        ev_is_em_api) {
  EvMeterDecision reject{Action::Reject, false, 0, 0, 0, false, 0, false, 0};
  if (!ev_is_em_api) return reject;
  if (payload == nullptr || len == 0) return reject;

  uint32_t i = 0;
  int32_t v[5] = {0, 0, 0, 0, 0};
  for (int k = 0; k < 5; ++k) {
    if (!parse_signed_int(payload, len, &i, &v[k])) return reject;
    if (k < 4) {
      if (i >= len || payload[i] != ':') return reject;
      ++i;
    }
  }
  // Parse succeeded → Action::Apply. Each of the three arms has its own
  // admission gate; an Apply with all three `apply_*` flags false is a
  // legal (if useless) outcome — matches firmware sending "-1:-1:-1:-1:-1".
  EvMeterDecision d{Action::Apply, false, 0, 0, 0, false, 0, false, 0};
  const bool currents_ok =
      v[0] > -1 && v[0] < 1000 &&
      v[1] > -1 && v[1] < 1000 &&
      v[2] > -1 && v[2] < 1000;
  if (currents_ok) {
    d.apply_currents = true;
    d.l1 = static_cast<int16_t>(v[0]);
    d.l2 = static_cast<int16_t>(v[1]);
    d.l3 = static_cast<int16_t>(v[2]);
  }
  if (v[3] > -1) {
    d.apply_power = true;
    d.power_w = v[3];
  }
  if (v[4] > -1) {
    d.apply_energy = true;
    d.energy_wh = v[4];
  }
  return d;
}

// Mirrors esp32.cpp /Set/CableLock. Legacy: `payload == "1"` → 1, else
// → 0; request_write_settings() called unconditionally. The helper
// tightens admission to exactly { "0", "1" }; the call site keeps the
// unconditional flash write for bit-identity (FEEDBACK §11).
Decision decide_cable_lock(const char* payload, uint32_t len) {
  if (payload == nullptr || len == 0) {
    return {Action::Reject, 0};
  }
  if (len == 1 && payload[0] == '1') return {Action::Apply, 1};
  if (len == 1 && payload[0] == '0') return {Action::Apply, 0};
  return {Action::Reject, 0};
}

// Mirrors esp32.cpp /Set/HomeBatteryCurrent. Legacy code had no range
// check and silently wrote 0 on any non-numeric payload (String::toInt
// returns 0). Helper tightens: empty/non-numeric → Reject; valid
// int → Apply with int16 narrowing at the call site. See FEEDBACK §10.
SignedDecision decide_home_battery_current(
    const char* payload,
    uint32_t    len,
    uint8_t     load_balance) {
  SignedDecision reject{Action::Reject, 0};
  if (load_balance >= 2) return reject;
  if (payload == nullptr || len == 0) return reject;
  uint32_t i = 0;
  int32_t v = 0;
  if (!parse_signed_int(payload, len, &i, &v)) return reject;
  // Saturate at int16 bounds — homeBatteryCurrent is int16_t.
  if (v > 32767)  v = 32767;
  if (v < -32768) v = -32768;
  return {Action::Apply, static_cast<int16_t>(v)};
}

// Mirrors esp32.cpp /Set/CustomButton. Always Apply — legacy code was:
//   if (payload == "On") CustomButton = true; else CustomButton = false;
// so a 2-state always-accept toggle with no Reject arm. The helper
// preserves that bit-identically; .value is 1 on exact-match "On", 0
// on anything else (case-sensitive, matches String::operator==).
Decision decide_custom_button(const char* payload, uint32_t len) {
  if (payload != nullptr && len == 2 &&
      payload[0] == 'O' && payload[1] == 'n') {
    return {Action::Apply, 1};
  }
  return {Action::Apply, 0};
}

namespace {

// Dallas CRC-8 (polynomial 0x8C reversed, init 0). Matches utils.cpp::crc8
// byte-for-byte — duplicated here so evse_limits has no dependency on
// the firmware's utility module.
uint8_t rfid_crc8(const uint8_t* buf, uint8_t n) {
  uint8_t crc = 0;
  for (uint8_t k = 0; k < n; ++k) {
    uint8_t inbyte = buf[k];
    for (uint8_t j = 0; j < 8; ++j) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

bool is_hex_digit(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}

uint8_t hex_nybble(char c) {
  if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
  if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(10 + (c - 'a'));
  return static_cast<uint8_t>(10 + (c - 'A'));
}

}  // namespace

// Mirrors esp32.cpp /Set/RFID. Trims whitespace, validates hex, dispatches
// on length (12 → 6-byte UID with 0x01 family prefix, 14 → 7-byte UID),
// fills uid[0..6] with parsed bytes, and computes CRC into uid[7].
RfidDecision decide_rfid_uid(const char* payload, uint32_t len, bool reader_enabled) {
  RfidDecision reject{Action::Reject, {0, 0, 0, 0, 0, 0, 0, 0}};
  if (!reader_enabled) return reject;
  if (payload == nullptr) return reject;

  // Trim leading/trailing whitespace to match Arduino String::trim().
  uint32_t start = 0, end = len;
  while (start < end && (payload[start] == ' '  || payload[start] == '\t' ||
                         payload[start] == '\n' || payload[start] == '\r')) ++start;
  while (end > start && (payload[end - 1] == ' '  || payload[end - 1] == '\t' ||
                         payload[end - 1] == '\n' || payload[end - 1] == '\r')) --end;
  uint32_t tlen = end - start;
  if (tlen != 12 && tlen != 14) return reject;

  for (uint32_t k = 0; k < tlen; ++k) {
    if (!is_hex_digit(payload[start + k])) return reject;
  }

  RfidDecision d{Action::Apply, {0, 0, 0, 0, 0, 0, 0, 0}};
  if (tlen == 12) {
    // 6-byte UID (old reader). Family code 0x01 at uid[0]; payload bytes at uid[1..6].
    d.uid[0] = 0x01;
    for (uint32_t k = 0; k < 6; ++k) {
      uint8_t hi = hex_nybble(payload[start + k * 2]);
      uint8_t lo = hex_nybble(payload[start + k * 2 + 1]);
      d.uid[k + 1] = static_cast<uint8_t>((hi << 4) | lo);
    }
  } else {
    // 7-byte UID (new reader). All 7 payload bytes at uid[0..6].
    for (uint32_t k = 0; k < 7; ++k) {
      uint8_t hi = hex_nybble(payload[start + k * 2]);
      uint8_t lo = hex_nybble(payload[start + k * 2 + 1]);
      d.uid[k] = static_cast<uint8_t>((hi << 4) | lo);
    }
  }
  d.uid[7] = rfid_crc8(d.uid, 7);
  return d;
}

// Mirrors the five /Set/Color* branches in esp32.cpp. Legacy code did
// `sscanf("%d,%d,%d")` and applied on n==3 with each channel in [0, 256).
// The hand-rolled parser replaces sscanf for consistency with the other
// meter helpers; strict ',' separator, no whitespace tolerance. Trailing
// bytes after the third number are ignored, matching sscanf's behavior.
ColorDecision decide_color_rgb(const char* payload, uint32_t len) {
  ColorDecision reject{Action::Reject, 0, 0, 0};
  if (payload == nullptr || len == 0) return reject;
  uint32_t i = 0;
  int32_t v[3] = {0, 0, 0};
  for (int k = 0; k < 3; ++k) {
    if (!parse_signed_int(payload, len, &i, &v[k])) return reject;
    if (k < 2) {
      if (i >= len || payload[i] != ',') return reject;
      ++i;
    }
  }
  for (int k = 0; k < 3; ++k) {
    if (v[k] < 0 || v[k] >= 256) return reject;
  }
  return {Action::Apply,
          static_cast<uint8_t>(v[0]),
          static_cast<uint8_t>(v[1]),
          static_cast<uint8_t>(v[2])};
}

Decision decide_enable_c2(const char* payload, uint32_t len) {
  if (len == 0 || payload == nullptr) {
    return {Action::Reject, 0};
  }
  // Digit branch. Firmware: if (isdigit(payload[0])) { value = payload.toInt(); if (value <= 4) ... }
  // We tighten it to only accept exactly one digit (the legacy code
  // accepted "3abc" because String::toInt stops at non-digits).
  // Intentionally looser-acceptance matching: to preserve bit-identity,
  // accept the digit prefix and ignore trailing garbage just like
  // String::toInt. The first char is the gate.
  if (payload[0] >= '0' && payload[0] <= '9') {
    // Match String::toInt semantics: digits up to first non-digit.
    uint32_t acc = 0;
    for (uint32_t i = 0; i < len; ++i) {
      char c = payload[i];
      if (c < '0' || c > '9') break;
      acc = acc * 10 + static_cast<uint32_t>(c - '0');
      if (acc > 255) { acc = 255; break; }  // saturate (uint8_t overflow sentinel)
    }
    if (acc <= 4) {
      return {Action::Apply, static_cast<uint16_t>(acc)};
    }
    // Digit parsed but out of range — legacy code dropped it, no
    // fallback to string match (the outer if/else gated that).
    return {Action::Reject, 0};
  }
  // String branch. Legacy code walks the table and accepts on exact match.
  static const char* kStrEnableC2[5] = {
    "Not present", "Always Off", "Solar Off", "Always On", "Auto"
  };
  for (uint8_t i = 0; i < 5; ++i) {
    const uint32_t tlen = static_cast<uint32_t>(std::strlen(kStrEnableC2[i]));
    if (len == tlen && std::memcmp(payload, kStrEnableC2[i], tlen) == 0) {
      return {Action::Apply, i};
    }
  }
  return {Action::Reject, 0};
}

ModeDecision decide_set_mode(const char* payload, uint32_t len) {
  auto eq = [&](const char* target) {
    const uint32_t tlen = static_cast<uint32_t>(std::strlen(target));
    return len == tlen && std::memcmp(payload, target, tlen) == 0;
  };
  if (eq("Off"))    return {ModeAction::ApplyAccess, kAccessOff};
  if (eq("Normal")) return {ModeAction::ApplyMode,   kModeNormal};
  if (eq("Solar"))  return {ModeAction::ApplyMode,   kModeSolar};
  if (eq("Smart"))  return {ModeAction::ApplyMode,   kModeSmart};
  if (eq("Pause"))  return {ModeAction::ApplyAccess, kAccessPause};
  return {ModeAction::Reject, 0};
}

}  // namespace evse::limits

// Flat C ABI bridges. Each returns 0=Reject, 1=Clear, 2=Apply and
// writes the accepted value (or 0 for Reject/Clear) to *out_value.

extern "C" uint8_t evse_limits_decide_current_override(
    uint16_t requested_dA,
    uint16_t min_current_dA,
    uint16_t max_current_dA,
    uint8_t  load_balance,
    uint8_t  mode,
    uint16_t* out_value_dA) {
  auto d = evse::limits::decide_current_override(
      requested_dA, min_current_dA, max_current_dA, load_balance, mode);
  if (out_value_dA) *out_value_dA = d.value;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_max_sum_mains(
    uint16_t requested_a,
    uint8_t  load_balance,
    uint16_t* out_value_a) {
  auto d = evse::limits::decide_max_sum_mains(requested_a, load_balance);
  if (out_value_a) *out_value_a = d.value;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_cp_pwm_override(
    int32_t requested,
    uint16_t* out_value) {
  auto d = evse::limits::decide_cp_pwm_override(requested);
  if (out_value) *out_value = d.value;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_set_mode(
    const char* payload,
    uint32_t len,
    uint8_t* out_target) {
  auto d = evse::limits::decide_set_mode(payload, len);
  if (out_target) *out_target = d.target;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_enable_c2(
    const char* payload,
    uint32_t len,
    uint16_t* out_value) {
  auto d = evse::limits::decide_enable_c2(payload, len);
  if (out_value) *out_value = d.value;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_mains_meter(
    const char* payload,
    uint32_t len,
    uint8_t mains_is_em_api,
    uint8_t load_balance,
    int16_t* out_l1,
    int16_t* out_l2,
    int16_t* out_l3) {
  auto d = evse::limits::decide_mains_meter(
      payload, len, mains_is_em_api != 0, load_balance);
  if (out_l1) *out_l1 = d.l1;
  if (out_l2) *out_l2 = d.l2;
  if (out_l3) *out_l3 = d.l3;
  return static_cast<uint8_t>(d.action);
}

// /Set/EVMeter has three partial-apply arms. The flat-C bridge returns
// the overall Action (0=Reject, 2=Apply) plus a bitmask in *out_flags
// marking which arms are live on Apply:
//   bit 0 = currents valid (l1/l2/l3 populated)
//   bit 1 = power valid    (power_w populated)
//   bit 2 = energy valid   (energy_wh populated)
// On Reject, *out_flags is 0 and all out-values are 0.
extern "C" uint8_t evse_limits_decide_cable_lock(
    const char* payload,
    uint32_t len,
    uint8_t* out_value) {
  auto d = evse::limits::decide_cable_lock(payload, len);
  if (out_value) *out_value = static_cast<uint8_t>(d.value);
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_home_battery_current(
    const char* payload,
    uint32_t len,
    uint8_t load_balance,
    int16_t* out_value) {
  auto d = evse::limits::decide_home_battery_current(payload, len, load_balance);
  if (out_value) *out_value = d.value;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_ev_meter(
    const char* payload,
    uint32_t len,
    uint8_t ev_is_em_api,
    uint8_t* out_flags,
    int16_t* out_l1,
    int16_t* out_l2,
    int16_t* out_l3,
    int32_t* out_power_w,
    int32_t* out_energy_wh) {
  auto d = evse::limits::decide_ev_meter(payload, len, ev_is_em_api != 0);
  uint8_t flags = 0;
  if (d.apply_currents) flags |= 0x1;
  if (d.apply_power)    flags |= 0x2;
  if (d.apply_energy)   flags |= 0x4;
  if (out_flags)     *out_flags = flags;
  if (out_l1)        *out_l1 = d.l1;
  if (out_l2)        *out_l2 = d.l2;
  if (out_l3)        *out_l3 = d.l3;
  if (out_power_w)   *out_power_w = d.power_w;
  if (out_energy_wh) *out_energy_wh = d.energy_wh;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_color_rgb(
    const char* payload,
    uint32_t len,
    uint8_t* out_r,
    uint8_t* out_g,
    uint8_t* out_b) {
  auto d = evse::limits::decide_color_rgb(payload, len);
  if (out_r) *out_r = d.r;
  if (out_g) *out_g = d.g;
  if (out_b) *out_b = d.b;
  return static_cast<uint8_t>(d.action);
}

extern "C" uint8_t evse_limits_decide_custom_button(
    const char* payload,
    uint32_t len,
    uint8_t* out_value) {
  auto d = evse::limits::decide_custom_button(payload, len);
  if (out_value) *out_value = static_cast<uint8_t>(d.value);
  return static_cast<uint8_t>(d.action);
}

// Writes 8 bytes (family code / UID / CRC) into out_uid on Apply;
// zeroes them on Reject. Return matches Action enum (0/2).
extern "C" uint8_t evse_limits_decide_rfid_uid(
    const char* payload,
    uint32_t len,
    uint8_t reader_enabled,
    uint8_t* out_uid) {
  auto d = evse::limits::decide_rfid_uid(payload, len, reader_enabled != 0);
  if (out_uid) {
    for (int k = 0; k < 8; ++k) out_uid[k] = d.uid[k];
  }
  return static_cast<uint8_t>(d.action);
}
