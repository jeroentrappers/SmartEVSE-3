// DeviceUnderTest — owns the firmware Core + surrounding virtual hardware.
//
// The harness holds everything the firmware would touch on real hardware:
// the pilot voltage source, the mains meter, temperature, RCM latch.
// Commands from the dashboard / scenario runner mutate those inputs; the
// `tick` command advances virtual time by N ms, driving Core::tick_10ms
// and Core::tick_1s on schedule.

#pragma once

#include <cstdint>
#include <string>

// firmware_core lives under SmartEVSE-3/src/ — the firmware owns the
// module and the native harness links the same translation unit via
// the Makefile's FW_SRC include path.
#include "firmware_core.h"

namespace evse {

class EventSink {
 public:
  virtual ~EventSink() = default;
  // Emit one JSON event to stdout. `payload` is a pre-formatted JSON body
  // without the outer braces — the sink adds t_ms, kind, and wrapping.
  virtual void emit(const char* kind, const std::string& payload) = 0;
  virtual uint64_t now_ms() const = 0;
};

class DeviceUnderTest : public Observer {
 public:
  explicit DeviceUnderTest(EventSink* sink);

  void boot();
  void reset();

  // Virtual-time advance. Runs tick_10ms at 10 ms granularity and
  // tick_1s once per second of simulated time.
  void tick(uint32_t ms);

  // Input mutators — each matches a dashboard command in the JSON protocol.
  void set_pilot_mv(uint32_t mv);
  void plug_ev(bool connected);
  void set_mode(Mode m);
  void rcm_trip();
  void rcm_clear();
  void set_temp_c(float t);
  void set_mains(float l1, float l2, float l3);
  void ext_switch(bool on);
  void set_load_balance(uint8_t role);   // 0=standalone, 1=master, >=2=slave
  // Toggle whether the harness's virtual mains meter is configured as
  // EM_API (accepts externally-fed readings via MQTT /Set/MainsMeter) or
  // a direct-wired meter (rejects all MQTT mains payloads). Defaults true
  // so scenarios can exercise the admit path without extra setup.
  void set_mains_meter_em_api(bool on);
  // Same toggle for the EV-side meter (/Set/EVMeter). Defaults true.
  void set_ev_meter_em_api(bool on);
  void mqtt_set(const std::string& topic, const std::string& payload);
  void snapshot();

  // Observer hooks — forwards Core's events to the EventSink.
  void on_state(State from, State to, const char* reason) override;
  void on_contactor(uint8_t phase, bool closed) override;
  void on_pilot(bool connected) override;
  void on_pwm(uint16_t duty_permille, float advertised_a) override;
  void on_error(uint8_t flag, bool set) override;

 private:
  void publish_snapshot();
  void publish_mqtt(const std::string& topic, const std::string& payload, bool retain);

  EventSink* sink_;
  Core       core_;
  Inputs     in_;
  uint32_t   pilot_mv_  = 12000;
  bool       ev_plugged_ = false;
  // CP PWM override from MQTT /Set/CPPWMOverride. -1 = no override (core
  // drives PWM normally). 0..1024 = firmware wrote this raw duty to
  // ledcWrite and set CPDutyOverride=true; we surface the last-written
  // duty so scenarios can see the override via snapshot.
  int16_t    cp_pwm_override_ = -1;
  // EnableC2 setting from MQTT /Set/EnableC2. 0..4 matching main.h
  // EnableC2_t { NOT_PRESENT, ALWAYS_OFF, SOLAR_OFF, ALWAYS_ON, AUTO }.
  // Harness default matches firmware's power-on default of 0.
  uint8_t    enable_c2_ = 0;
  // Whether the virtual mains meter is configured as EM_API
  // (externally-fed via MQTT) vs directly wired. Default true: scenarios
  // that want to verify the non-EM_API rejection path flip to false.
  bool       mains_meter_em_api_ = true;
  bool       ev_meter_em_api_    = true;
  // Last EV-meter admitted values, surfaced in snapshot so scenarios
  // can observe which of the three partial-apply arms fired.
  int16_t    ev_l1_dA_ = 0;
  int16_t    ev_l2_dA_ = 0;
  int16_t    ev_l3_dA_ = 0;
  int32_t    ev_power_w_ = 0;
  int32_t    ev_energy_wh_ = 0;
  // Last CableLock value seen via /Set/CableLock (0 or 1). Legacy
  // firmware writes 0 on unknown input; the harness leaves the prior
  // value intact on Reject so the tightening vs bit-identity diff is
  // visible in scenarios.
  uint8_t    cable_lock_ = 0;
  // Last HomeBatteryCurrent from /Set/HomeBatteryCurrent, dA (signed).
  int16_t    home_battery_current_dA_ = 0;
  // Last /Set/Color* writes. Five 3-byte RGB globals in the firmware;
  // harness mirrors them so scenarios can snapshot per-topic apply.
  uint8_t    color_off_[3]    = {0, 0, 0};
  uint8_t    color_normal_[3] = {0, 0, 0};
  uint8_t    color_smart_[3]  = {0, 0, 0};
  uint8_t    color_solar_[3]  = {0, 0, 0};
  uint8_t    color_custom_[3] = {0, 0, 0};
  // Last /Set/CustomButton value (0 or 1). Legacy firmware treats anything
  // except the exact string "On" as Off, so there is no Reject path here.
  uint8_t    custom_button_ = 0;
  // Last /Set/RFID accepted UID (8 bytes: family / UID / CRC-8) and
  // whether the reader is enabled. The reader-enabled flag defaults true
  // so scenarios don't need to toggle it; flipping to false exercises
  // the disabled-reader reject path.
  uint8_t    rfid_uid_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bool       rfid_reader_enabled_ = true;
  std::string mqtt_prefix_ = "SmartEVSE-12345";
};

}  // namespace evse
