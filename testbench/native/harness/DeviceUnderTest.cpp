#include "harness/DeviceUnderTest.h"

#include <cstdio>
#include <sstream>

#include "evse_limits.h"

namespace evse {

namespace {

// Minimal JSON string escaper. Covers the subset we actually emit
// (topics, payloads, reasons) — all printable ASCII, no UTF-16 escapes.
std::string jesc(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 2);
  for (char c : s) {
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

}  // namespace

DeviceUnderTest::DeviceUnderTest(EventSink* sink) : sink_(sink) {
  core_.set_observer(this);
}

void DeviceUnderTest::boot() {
  {
    std::ostringstream os;
    os << "\"version\":\"native-0.1.0\",\"prefix\":\"" << mqtt_prefix_ << "\"";
    sink_->emit("boot", os.str());
  }
  publish_snapshot();
}

void DeviceUnderTest::reset() {
  core_.reset();
  in_ = Inputs{};
  pilot_mv_ = 12000;
  ev_plugged_ = false;
  cp_pwm_override_ = -1;
  enable_c2_ = 0;
  mains_meter_em_api_ = true;
  ev_meter_em_api_ = true;
  ev_l1_dA_ = 0; ev_l2_dA_ = 0; ev_l3_dA_ = 0;
  ev_power_w_ = 0; ev_energy_wh_ = 0;
  cable_lock_ = 0;
  home_battery_current_dA_ = 0;
  for (int i = 0; i < 3; ++i) {
    color_off_[i] = color_normal_[i] = color_smart_[i] =
        color_solar_[i] = color_custom_[i] = 0;
  }
  custom_button_ = 0;
  for (int i = 0; i < 8; ++i) rfid_uid_[i] = 0;
  rfid_reader_enabled_ = true;
  sink_->emit("reset", "");
  publish_snapshot();
}

void DeviceUnderTest::tick(uint32_t ms) {
  uint64_t start = sink_->now_ms();
  uint32_t remaining = ms;
  // Advance in 10ms chunks. Every 1000ms, also fire tick_1s.
  while (remaining >= 10) {
    core_.tick_10ms(in_);
    uint64_t now = sink_->now_ms();
    // tick_1s fires at every second-boundary crossed during this tick.
    if ((now / 1000) != ((now - 10) / 1000)) {
      core_.tick_1s();
    }
    remaining -= 10;
  }
  (void)start;
}

void DeviceUnderTest::set_pilot_mv(uint32_t mv) {
  pilot_mv_ = mv;
  // If EV is unplugged, pilot is pulled to +12V regardless of knob.
  uint32_t effective = ev_plugged_ ? mv : 12000;
  in_.pilot = pilot_from_cp_mv(effective);
}

void DeviceUnderTest::plug_ev(bool connected) {
  ev_plugged_ = connected;
  // When disconnected, pilot is pulled to +12V regardless of knob. When
  // plugged in and the user hasn't dialed it down yet, nudge to +9V so
  // the state transitions A→B (matches the dashboard's plug-then-tune UX).
  uint32_t effective = connected
      ? (pilot_mv_ >= 11000 ? 9000 : pilot_mv_)
      : 12000;
  in_.pilot = pilot_from_cp_mv(effective);
}

void DeviceUnderTest::set_mode(Mode m) {
  in_.mode = m;
  in_.access_on = (m != MODE_OFF);
}

void DeviceUnderTest::rcm_trip() {
  in_.error_flags |= ERR_RCM_TRIPPED;
}

void DeviceUnderTest::rcm_clear() {
  in_.error_flags &= ~ERR_RCM_TRIPPED;
}

void DeviceUnderTest::set_temp_c(float t) {
  in_.temp_c = t;
  if (t >= 65.0f) in_.error_flags |= ERR_TEMP_HIGH;
  else            in_.error_flags &= ~ERR_TEMP_HIGH;
}

void DeviceUnderTest::set_mains(float l1, float l2, float l3) {
  in_.l1_a = l1;
  in_.l2_a = l2;
  in_.l3_a = l3;
}

void DeviceUnderTest::ext_switch(bool on) {
  in_.access_on = on;
}

void DeviceUnderTest::set_load_balance(uint8_t role) {
  in_.load_balance = role;
}

void DeviceUnderTest::set_mains_meter_em_api(bool on) {
  mains_meter_em_api_ = on;
}

void DeviceUnderTest::set_ev_meter_em_api(bool on) {
  ev_meter_em_api_ = on;
}

void DeviceUnderTest::mqtt_set(const std::string& topic, const std::string& payload) {
  std::ostringstream os;
  os << "\"topic\":\"" << jesc(topic) << "\",\"payload\":\"" << jesc(payload) << "\"";
  sink_->emit("mqtt_rx", os.str());

  // Very small routing table — matches the firmware's MQTT subscribe set.
  auto endswith = [](const std::string& s, const char* suf) {
    size_t n = 0; const char* p = suf; while (*p) { ++p; ++n; }
    return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
  };
  if (endswith(topic, "/Set/CurrentOverride")) {
    int dA = 0;
    try { dA = std::stoi(payload); } catch (...) { dA = 0; }
    auto decision = evse::limits::decide_current_override(
        static_cast<uint16_t>(dA < 0 ? 0 : dA),
        static_cast<uint16_t>(in_.min_current_a * 10),
        static_cast<uint16_t>(in_.max_current_a * 10),
        in_.load_balance,
        static_cast<uint8_t>(in_.mode));
    if (decision.action == evse::limits::Action::Clear) {
      in_.override_a = 0.0f;
      publish_mqtt(mqtt_prefix_ + "/CurrentOverride", "0", true);
    } else if (decision.action == evse::limits::Action::Apply) {
      in_.override_a = decision.value / 10.0f;
      publish_mqtt(mqtt_prefix_ + "/CurrentOverride",
                   std::to_string(decision.value), true);
    }
    // Reject → firmware drops silently, no echo publish.
  } else if (endswith(topic, "/Set/CurrentMaxSumMains")) {
    int req = 0;
    try { req = std::stoi(payload); } catch (...) { req = 0; }
    auto decision = evse::limits::decide_max_sum_mains(
        static_cast<uint16_t>(req < 0 ? 0 : req),
        in_.load_balance);
    if (decision.action == evse::limits::Action::Clear) {
      in_.max_sum_mains_a = 0;
    } else if (decision.action == evse::limits::Action::Apply) {
      in_.max_sum_mains_a = decision.value;
    }
    // Firmware does NOT publish an echo on this topic — intentional.
  } else if (endswith(topic, "/Set/CPPWMOverride")) {
    int32_t req = 0;
    try { req = std::stoi(payload); } catch (...) { req = 0; }
    uint16_t accepted = 0;
    auto decision = evse::limits::decide_cp_pwm_override(req);
    accepted = decision.value;
    if (decision.action == evse::limits::Action::Clear) {
      // Release override — firmware does SetCPDuty(1024) + PILOT_CONNECTED.
      // In the harness, the core's normal PWM now takes over; we surface
      // the override-release via two synthetic events so scenarios can
      // assert it without reading snapshots.
      cp_pwm_override_ = -1;
      std::ostringstream pwm_os;
      // 1024/1024 → 100% duty = pilot held high (no PWM swing).
      pwm_os << "\"duty\":100,\"advertised_a\":0";
      sink_->emit("pwm", pwm_os.str());
      sink_->emit("pilot", std::string("\"connected\":true"));
    } else if (decision.action == evse::limits::Action::Apply) {
      // Raw 1024-scale duty written. Convert to percent for the `duty`
      // field so the wire format stays aligned with the core's emitter.
      cp_pwm_override_ = static_cast<int16_t>(accepted);
      int percent = (accepted * 100) / 1024;
      std::ostringstream pwm_os;
      pwm_os << "\"duty\":" << percent << ",\"advertised_a\":0";
      sink_->emit("pwm", pwm_os.str());
      std::string pilot_payload = std::string("\"connected\":") +
          (accepted > 0 ? "true" : "false");
      sink_->emit("pilot", pilot_payload);
    }
    // Reject → silent. Firmware drops the payload. No echo publish.
  } else if (endswith(topic, "/Set/EnableC2")) {
    uint16_t accepted = 0;
    auto decision = evse::limits::decide_enable_c2(
        payload.data(), static_cast<uint32_t>(payload.size()));
    if (decision.action == evse::limits::Action::Apply) {
      accepted = decision.value;
      enable_c2_ = static_cast<uint8_t>(accepted);
    }
    // Reject → silent drop (harness does not emulate the firmware's
    // unconditional request_write_settings() side effect; that's a
    // flash-write concern, not an observable admission outcome).
  } else if (endswith(topic, "/Set/Mode")) {
    auto decision = evse::limits::decide_set_mode(
        payload.data(), static_cast<uint32_t>(payload.size()));
    if (decision.action == evse::limits::ModeAction::ApplyMode) {
      // Solar clears the override current as a side effect, matching
      // the firmware's legacy inline ordering (see esp32.cpp:614-616).
      if (decision.target == evse::limits::kModeSolar) {
        in_.override_a = 0.0f;
      }
      set_mode(static_cast<Mode>(decision.target));
      publish_mqtt(mqtt_prefix_ + "/Mode", payload, true);
    } else if (decision.action == evse::limits::ModeAction::ApplyAccess) {
      if (decision.target == evse::limits::kAccessOff) {
        // MODE_OFF is the harness's sentinel for "access off" in the
        // Core's view. Snapshot's mode_name() will show "Off".
        set_mode(MODE_OFF);
      } else {
        // Pause: no dedicated mode sentinel — just cut access_on; the
        // last mode value sticks in the snapshot.
        in_.access_on = false;
      }
      publish_mqtt(mqtt_prefix_ + "/Mode", payload, true);
    }
    // Reject → firmware drops silently, no echo publish.
  } else if (endswith(topic, "/Set/EVMeter")) {
    auto decision = evse::limits::decide_ev_meter(
        payload.data(),
        static_cast<uint32_t>(payload.size()),
        ev_meter_em_api_);
    if (decision.action == evse::limits::Action::Apply) {
      // Three independent arms — emit a single ev_meter event per
      // admission with a flags bitmask mirroring the C-ABI contract
      // (0x1 currents, 0x2 power, 0x4 energy). Surface the applied
      // values in snapshot too so regressions to any arm are visible.
      uint8_t flags = 0;
      if (decision.apply_currents) {
        flags |= 0x1;
        ev_l1_dA_ = decision.l1;
        ev_l2_dA_ = decision.l2;
        ev_l3_dA_ = decision.l3;
      }
      if (decision.apply_power) {
        flags |= 0x2;
        ev_power_w_ = decision.power_w;
      }
      if (decision.apply_energy) {
        flags |= 0x4;
        ev_energy_wh_ = decision.energy_wh;
      }
      std::ostringstream os;
      os << "\"flags\":" << static_cast<int>(flags)
         << ",\"l1_dA\":" << ev_l1_dA_
         << ",\"l2_dA\":" << ev_l2_dA_
         << ",\"l3_dA\":" << ev_l3_dA_
         << ",\"power_w\":" << ev_power_w_
         << ",\"energy_wh\":" << ev_energy_wh_;
      sink_->emit("ev_meter", os.str());
    }
    // Reject → silent. Firmware drops the payload.
  } else if (endswith(topic, "/Set/CableLock")) {
    auto decision = evse::limits::decide_cable_lock(
        payload.data(), static_cast<uint32_t>(payload.size()));
    if (decision.action == evse::limits::Action::Apply) {
      cable_lock_ = static_cast<uint8_t>(decision.value);
    }
    // Firmware keeps request_write_settings() unconditional here — the
    // harness emits a `flash_write` event so scenarios can pin the
    // flash-write-on-reject pattern called out in FEEDBACK §11.
    sink_->emit("flash_write", "\"topic\":\"CableLock\"");
  } else if (endswith(topic, "/Set/HomeBatteryCurrent")) {
    auto decision = evse::limits::decide_home_battery_current(
        payload.data(),
        static_cast<uint32_t>(payload.size()),
        in_.load_balance);
    if (decision.action == evse::limits::Action::Apply) {
      home_battery_current_dA_ = decision.value;
    }
    // Firmware sets homeBatteryLastUpdate only on the tightened Apply
    // path (legacy set it unconditionally on non-slave — see FEEDBACK §10).
  } else if (endswith(topic, "/Set/ColorOff")
          || endswith(topic, "/Set/ColorNormal")
          || endswith(topic, "/Set/ColorSmart")
          || endswith(topic, "/Set/ColorSolar")
          || endswith(topic, "/Set/ColorCustom")) {
    auto decision = evse::limits::decide_color_rgb(
        payload.data(), static_cast<uint32_t>(payload.size()));
    if (decision.action == evse::limits::Action::Apply) {
      uint8_t* target = nullptr;
      const char* name = "";
      if (endswith(topic, "/Set/ColorOff"))         { target = color_off_;    name = "Off"; }
      else if (endswith(topic, "/Set/ColorNormal")) { target = color_normal_; name = "Normal"; }
      else if (endswith(topic, "/Set/ColorSmart"))  { target = color_smart_;  name = "Smart"; }
      else if (endswith(topic, "/Set/ColorSolar"))  { target = color_solar_;  name = "Solar"; }
      else if (endswith(topic, "/Set/ColorCustom")) { target = color_custom_; name = "Custom"; }
      if (target) { target[0] = decision.r; target[1] = decision.g; target[2] = decision.b; }
      std::ostringstream os;
      os << "\"which\":\"" << name << "\""
         << ",\"r\":" << static_cast<int>(decision.r)
         << ",\"g\":" << static_cast<int>(decision.g)
         << ",\"b\":" << static_cast<int>(decision.b);
      sink_->emit("color_apply", os.str());
    }
    // Reject → silent drop, matches legacy firmware.
  } else if (endswith(topic, "/Set/CustomButton")) {
    auto decision = evse::limits::decide_custom_button(
        payload.data(), static_cast<uint32_t>(payload.size()));
    if (decision.action == evse::limits::Action::Apply) {
      custom_button_ = static_cast<uint8_t>(decision.value);
      std::ostringstream os;
      os << "\"value\":" << static_cast<int>(custom_button_);
      sink_->emit("custom_button", os.str());
    }
    // Helper always returns Apply — there is no Reject path here; legacy
    // firmware treats any payload other than exact "On" as "Off".
  } else if (endswith(topic, "/Set/RFID")) {
    auto decision = evse::limits::decide_rfid_uid(
        payload.data(),
        static_cast<uint32_t>(payload.size()),
        rfid_reader_enabled_);
    if (decision.action == evse::limits::Action::Apply) {
      for (int i = 0; i < 8; ++i) rfid_uid_[i] = decision.uid[i];
      std::ostringstream os;
      os << "\"uid\":\"";
      for (int i = 0; i < 8; ++i) {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02X", rfid_uid_[i]);
        os << buf;
      }
      os << "\"";
      sink_->emit("rfid_apply", os.str());
    }
    // Reject → silent drop (reader disabled, bad hex, or wrong length).
  } else if (endswith(topic, "/Set/MainsMeter")) {
    int16_t l1 = 0, l2 = 0, l3 = 0;
    auto decision = evse::limits::decide_mains_meter(
        payload.data(),
        static_cast<uint32_t>(payload.size()),
        mains_meter_em_api_,
        in_.load_balance);
    if (decision.action == evse::limits::Action::Apply) {
      l1 = decision.l1;
      l2 = decision.l2;
      l3 = decision.l3;
      // Helper returns deci-amps; harness mains are floats in amps.
      in_.l1_a = l1 / 10.0f;
      in_.l2_a = l2 / 10.0f;
      in_.l3_a = l3 / 10.0f;
      std::ostringstream os;
      os << "\"l1_dA\":" << l1 << ",\"l2_dA\":" << l2 << ",\"l3_dA\":" << l3;
      sink_->emit("mains_meter", os.str());
    }
    // Reject → firmware drops silently, no echo publish.
  }
}

void DeviceUnderTest::snapshot() { publish_snapshot(); }

void DeviceUnderTest::on_state(State from, State to, const char* reason) {
  std::ostringstream os;
  os << "\"from\":\"" << state_name(from) << "\",\"to\":\"" << state_name(to)
     << "\",\"reason\":\"" << jesc(reason ? reason : "") << "\"";
  sink_->emit("state", os.str());
  publish_mqtt(mqtt_prefix_ + "/State", state_name(to), true);
}

void DeviceUnderTest::on_contactor(uint8_t phase, bool closed) {
  std::ostringstream os;
  os << "\"phase\":" << static_cast<int>(phase)
     << ",\"closed\":" << (closed ? "true" : "false");
  sink_->emit("contactor", os.str());
}

void DeviceUnderTest::on_pilot(bool connected) {
  std::ostringstream os;
  os << "\"connected\":" << (connected ? "true" : "false");
  sink_->emit("pilot", os.str());
}

void DeviceUnderTest::on_pwm(uint16_t duty_permille, float advertised_a) {
  std::ostringstream os;
  os << "\"duty\":" << (duty_permille / 10)
     << ",\"advertised_a\":" << advertised_a;
  sink_->emit("pwm", os.str());
}

void DeviceUnderTest::on_error(uint8_t flag, bool set) {
  const char* name = "UNKNOWN";
  switch (flag) {
    case ERR_LESS_6A:     name = "LESS_6A"; break;
    case ERR_CT_NOCOMM:   name = "CT_NOCOMM"; break;
    case ERR_TEMP_HIGH:   name = "TEMP_HIGH"; break;
    case ERR_EV_NOCOMM:   name = "EV_NOCOMM"; break;
    case ERR_RCM_TRIPPED: name = "RCM_TRIPPED"; break;
  }
  std::ostringstream os;
  os << "\"flag\":\"" << name << "\",\"set\":" << (set ? "true" : "false");
  sink_->emit("error", os.str());
}

void DeviceUnderTest::publish_snapshot() {
  const Outputs& o = core_.outputs();
  std::ostringstream os;
  os << "\"state\":\"" << state_name(o.state) << "\","
     << "\"mode\":\""  << mode_name(in_.mode) << "\","
     << "\"pilot_mv\":" << pilot_mv_ << ","
     << "\"ev_connected\":" << (ev_plugged_ ? "true" : "false") << ","
     << "\"contactor\":[" << (o.contactor1 ? "true" : "false") << ","
                          << (o.contactor2 ? "true" : "false") << ",false],"
     << "\"pwm_duty\":" << (o.pwm_duty_permille / 10) << ","
     << "\"errors\":[],"
     << "\"charge_current_a\":" << o.charge_current_a << ","
     << "\"override_current_a\":" << in_.override_a << ","
     << "\"max_current_a\":" << in_.max_current_a << ","
     << "\"max_mains_a\":" << in_.max_mains_a << ","
     << "\"max_sum_mains_a\":" << in_.max_sum_mains_a << ","
     << "\"cp_pwm_override\":" << cp_pwm_override_ << ","
     << "\"enable_c2\":" << static_cast<int>(enable_c2_) << ","
     << "\"mains_meter_em_api\":" << (mains_meter_em_api_ ? "true" : "false") << ","
     << "\"ev_meter_em_api\":" << (ev_meter_em_api_ ? "true" : "false") << ","
     << "\"ev_meter\":{\"l1_dA\":" << ev_l1_dA_
           << ",\"l2_dA\":" << ev_l2_dA_
           << ",\"l3_dA\":" << ev_l3_dA_
           << ",\"power_w\":" << ev_power_w_
           << ",\"energy_wh\":" << ev_energy_wh_ << "},"
     << "\"cable_lock\":" << static_cast<int>(cable_lock_) << ","
     << "\"home_battery_current_dA\":" << home_battery_current_dA_ << ","
     << "\"colors\":{"
           << "\"off\":["    << static_cast<int>(color_off_[0])    << "," << static_cast<int>(color_off_[1])    << "," << static_cast<int>(color_off_[2])    << "],"
           << "\"normal\":[" << static_cast<int>(color_normal_[0]) << "," << static_cast<int>(color_normal_[1]) << "," << static_cast<int>(color_normal_[2]) << "],"
           << "\"smart\":["  << static_cast<int>(color_smart_[0])  << "," << static_cast<int>(color_smart_[1])  << "," << static_cast<int>(color_smart_[2])  << "],"
           << "\"solar\":["  << static_cast<int>(color_solar_[0])  << "," << static_cast<int>(color_solar_[1])  << "," << static_cast<int>(color_solar_[2])  << "],"
           << "\"custom\":[" << static_cast<int>(color_custom_[0]) << "," << static_cast<int>(color_custom_[1]) << "," << static_cast<int>(color_custom_[2]) << "]"
        << "},"
     << "\"custom_button\":" << static_cast<int>(custom_button_) << ","
     << "\"rfid_uid\":\"";
  for (int i = 0; i < 8; ++i) {
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02X", rfid_uid_[i]);
    os << buf;
  }
  os << "\","
     << "\"rfid_reader_enabled\":" << (rfid_reader_enabled_ ? "true" : "false") << ","
     << "\"temp_c\":" << in_.temp_c << ","
     << "\"mains\":[" << in_.l1_a << "," << in_.l2_a << "," << in_.l3_a << "]";
  sink_->emit("snapshot", os.str());
}

void DeviceUnderTest::publish_mqtt(const std::string& topic, const std::string& payload, bool retain) {
  std::ostringstream os;
  os << "\"topic\":\"" << jesc(topic) << "\","
     << "\"payload\":\"" << jesc(payload) << "\","
     << "\"retain\":" << (retain ? "true" : "false");
  sink_->emit("mqtt_publish", os.str());
}

}  // namespace evse
