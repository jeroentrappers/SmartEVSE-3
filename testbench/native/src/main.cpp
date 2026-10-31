// main.cpp — stdio JSON front-end for DeviceUnderTest.
//
// Protocol matches testbench/README.md:
//   stdin:  one JSON command object per line
//   stdout: one JSON event  object per line
// Virtual time advances only on `tick` commands; the process never sleeps.

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

#include "harness/DeviceUnderTest.h"

namespace {

using evse::DeviceUnderTest;
using evse::EventSink;
using evse::Mode;

// ---------------------------------------------------------------------
// Very small JSON parser — only handles the flat object shape used by the
// command protocol: {"cmd":"x","key":value,...} with string, number,
// boolean, and null values. Nesting and arrays are not needed on the
// command side.

struct JsonValue {
  enum Kind { kString, kNumber, kBool, kNull };
  Kind kind = kNull;
  std::string str;
  double num = 0.0;
  bool boolean = false;
};

class JsonParser {
 public:
  explicit JsonParser(const std::string& s) : s_(s) {}

  bool parse_object(std::unordered_map<std::string, JsonValue>& out) {
    skip_ws();
    if (!consume('{')) return false;
    skip_ws();
    if (peek() == '}') { ++i_; return true; }
    for (;;) {
      skip_ws();
      std::string key;
      if (!parse_string(key)) return false;
      skip_ws();
      if (!consume(':')) return false;
      skip_ws();
      JsonValue v;
      if (!parse_value(v)) return false;
      out[key] = std::move(v);
      skip_ws();
      if (peek() == ',') { ++i_; continue; }
      if (peek() == '}') { ++i_; return true; }
      return false;
    }
  }

 private:
  bool parse_value(JsonValue& v) {
    skip_ws();
    char c = peek();
    if (c == '"') { v.kind = JsonValue::kString; return parse_string(v.str); }
    if (c == 't' || c == 'f') {
      if (s_.compare(i_, 4, "true") == 0)  { i_ += 4; v.kind = JsonValue::kBool; v.boolean = true;  return true; }
      if (s_.compare(i_, 5, "false") == 0) { i_ += 5; v.kind = JsonValue::kBool; v.boolean = false; return true; }
      return false;
    }
    if (c == 'n') {
      if (s_.compare(i_, 4, "null") == 0) { i_ += 4; v.kind = JsonValue::kNull; return true; }
      return false;
    }
    // number
    size_t start = i_;
    if (c == '-' || c == '+') ++i_;
    while (i_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[i_])) ||
                              s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E' ||
                              s_[i_] == '+' || s_[i_] == '-')) ++i_;
    if (i_ == start) return false;
    try { v.num = std::stod(s_.substr(start, i_ - start)); }
    catch (...) { return false; }
    v.kind = JsonValue::kNumber;
    return true;
  }

  bool parse_string(std::string& out) {
    if (!consume('"')) return false;
    while (i_ < s_.size()) {
      char c = s_[i_++];
      if (c == '"') return true;
      if (c == '\\' && i_ < s_.size()) {
        char e = s_[i_++];
        switch (e) {
          case '"':  out += '"';  break;
          case '\\': out += '\\'; break;
          case '/':  out += '/';  break;
          case 'n':  out += '\n'; break;
          case 'r':  out += '\r'; break;
          case 't':  out += '\t'; break;
          default:   out += e;    break;  // tolerant
        }
      } else {
        out += c;
      }
    }
    return false;
  }

  char peek() const { return i_ < s_.size() ? s_[i_] : '\0'; }
  bool consume(char c) { if (peek() == c) { ++i_; return true; } return false; }
  void skip_ws() { while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_; }

  const std::string& s_;
  size_t i_ = 0;
};

// ---------------------------------------------------------------------
// Stdout event sink. Owns the virtual clock.

class StdoutSink : public EventSink {
 public:
  uint64_t now_ms() const override { return t_ms_; }
  void advance(uint32_t ms) { t_ms_ += ms; }

  void emit(const char* kind, const std::string& payload) override {
    std::ostringstream os;
    os << "{\"t\":" << t_ms_ << ",\"kind\":\"" << kind << "\"";
    if (!payload.empty()) os << "," << payload;
    os << "}\n";
    std::fwrite(os.str().data(), 1, os.str().size(), stdout);
    std::fflush(stdout);
  }

 private:
  uint64_t t_ms_ = 0;
};

Mode mode_from_string(const std::string& s) {
  if (s == "Off")    return evse::MODE_OFF;
  if (s == "Normal") return evse::MODE_NORMAL;
  if (s == "Smart")  return evse::MODE_SMART;
  if (s == "Solar")  return evse::MODE_SOLAR;
  return evse::MODE_SMART;
}

}  // namespace

int main() {
  StdoutSink sink;
  DeviceUnderTest dut(&sink);
  dut.boot();

  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    std::unordered_map<std::string, JsonValue> cmd;
    JsonParser p(line);
    if (!p.parse_object(cmd)) {
      std::fprintf(stderr, "parse error: %s\n", line.c_str());
      continue;
    }
    auto it = cmd.find("cmd");
    if (it == cmd.end() || it->second.kind != JsonValue::kString) continue;
    const std::string& name = it->second.str;

    auto num = [&](const char* k, double def = 0.0) -> double {
      auto j = cmd.find(k);
      if (j == cmd.end() || j->second.kind != JsonValue::kNumber) return def;
      return j->second.num;
    };
    auto str = [&](const char* k, const std::string& def = "") -> std::string {
      auto j = cmd.find(k);
      if (j == cmd.end() || j->second.kind != JsonValue::kString) return def;
      return j->second.str;
    };
    auto boolean = [&](const char* k, bool def = false) -> bool {
      auto j = cmd.find(k);
      if (j == cmd.end() || j->second.kind != JsonValue::kBool) return def;
      return j->second.boolean;
    };

    if (name == "tick") {
      uint32_t ms = static_cast<uint32_t>(num("ms", 0));
      // Advance the clock first so emissions during the tick carry the
      // correct t value.
      if (ms > 0) {
        // Emit tick events at 10ms granularity so scenarios can see
        // exactly when transitions fired.
        uint32_t remaining = ms;
        while (remaining >= 10) {
          sink.advance(10);
          dut.tick(10);
          remaining -= 10;
        }
        if (remaining > 0) sink.advance(remaining);
      }
    } else if (name == "set_pilot_mv") {
      dut.set_pilot_mv(static_cast<uint32_t>(num("mv", 12000)));
    } else if (name == "plug_ev") {
      dut.plug_ev(boolean("connected", false));
    } else if (name == "set_mode") {
      dut.set_mode(mode_from_string(str("mode", "Smart")));
    } else if (name == "rcm_trip") {
      dut.rcm_trip();
    } else if (name == "rcm_clear") {
      dut.rcm_clear();
    } else if (name == "set_temp_c") {
      dut.set_temp_c(static_cast<float>(num("temp", 25.0)));
    } else if (name == "set_mains") {
      dut.set_mains(static_cast<float>(num("l1")),
                    static_cast<float>(num("l2")),
                    static_cast<float>(num("l3")));
    } else if (name == "ext_switch") {
      dut.ext_switch(boolean("on", false));
    } else if (name == "set_load_balance") {
      dut.set_load_balance(static_cast<uint8_t>(num("role", 0)));
    } else if (name == "set_mains_meter_em_api") {
      dut.set_mains_meter_em_api(boolean("on", true));
    } else if (name == "set_ev_meter_em_api") {
      dut.set_ev_meter_em_api(boolean("on", true));
    } else if (name == "mqtt_set") {
      dut.mqtt_set(str("topic"), str("payload"));
    } else if (name == "reset") {
      dut.reset();
    } else if (name == "snapshot") {
      dut.snapshot();
    } else {
      std::fprintf(stderr, "unknown cmd: %s\n", name.c_str());
    }
  }
  return 0;
}
