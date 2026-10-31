// Fine-grained unit tests for firmware_core pure helpers and the
// simplified Core state machine.
//
// Scope: exercises each helper's boundaries (threshold edges, truncation
// behavior, invalid-input fallbacks) and a couple of end-to-end Core
// transitions. Complements the scenario corpus in testbench/scenarios/,
// which tests the same code through the stdio JSON protocol and the
// harness's virtual hardware.
//
// Why this file doesn't use Unity: PlatformIO's `pio test -e native`
// wants sources under `src/` linked into the test image, but the
// harness's src/main.cpp defines its own main(), which collides with
// Unity's test runner main(). Rather than add conditional PIO
// plumbing, this test is a plain executable built via the Makefile's
// `test` target. Intent matches `pio test`: fine-grained asserts,
// CI-gated on every run of `make check`.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "firmware_core.h"
#include "evse_limits.h"

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;
const char* g_current_test = "<none>";
bool g_test_failed = false;

#define TEST_BEGIN(name) \
    do { g_current_test = name; g_test_failed = false; ++g_tests_run; } while (0)

#define TEST_END() \
    do { if (g_test_failed) { ++g_tests_failed; std::printf("FAIL  %s\n", g_current_test); } \
         else { std::printf("PASS  %s\n", g_current_test); } } while (0)

#define EXPECT_EQ(expected, actual) \
    do { \
        auto _e = (expected); auto _a = (actual); \
        if (!(_e == _a)) { \
            std::printf("  %s:%d: EXPECT_EQ(%s, %s) — expected=%lld actual=%lld\n", \
                        __FILE__, __LINE__, #expected, #actual, \
                        static_cast<long long>(_e), static_cast<long long>(_a)); \
            g_test_failed = true; \
        } \
    } while (0)

// ---------------------------------------------------------------------
// Pilot decoding — main.cpp::Pilot() threshold table at ADC scale.
// ---------------------------------------------------------------------

void test_decode_pilot_bands() {
    TEST_BEGIN("decode_pilot band edges");
    using namespace evse;
    // Each band keyed on the low edge of min_mv from firmware_core.cpp.
    EXPECT_EQ(PILOT_12V, decode_pilot(3055, 3200));
    EXPECT_EQ(PILOT_12V, decode_pilot(3500, 3600));
    EXPECT_EQ(PILOT_9V,  decode_pilot(2735, 3054));
    EXPECT_EQ(PILOT_6V,  decode_pilot(2400, 2734));
    EXPECT_EQ(PILOT_3V,  decode_pilot(2000, 2399));
    EXPECT_EQ(PILOT_SHORT, decode_pilot(1600, 1999));
    EXPECT_EQ(PILOT_DIODE, decode_pilot(150,  250));
    // Gap between SHORT (<2000) and DIODE (<300) plus <100 → NOK.
    EXPECT_EQ(PILOT_NOK,   decode_pilot(50, 80));
    EXPECT_EQ(PILOT_NOK,   decode_pilot(1500, 1800));  // min<1600
    TEST_END();
}

void test_pilot_from_cp_mv_knobs() {
    TEST_BEGIN("pilot_from_cp_mv dashboard knobs");
    using namespace evse;
    EXPECT_EQ(PILOT_12V,    pilot_from_cp_mv(12000));
    EXPECT_EQ(PILOT_9V,     pilot_from_cp_mv(9000));
    EXPECT_EQ(PILOT_6V,     pilot_from_cp_mv(6000));
    EXPECT_EQ(PILOT_3V,     pilot_from_cp_mv(3000));
    EXPECT_EQ(PILOT_SHORT,  pilot_from_cp_mv(0));
    // Tolerance bands: ±1000 mV around each nominal level.
    EXPECT_EQ(PILOT_9V,     pilot_from_cp_mv(8500));
    EXPECT_EQ(PILOT_6V,     pilot_from_cp_mv(7500));
    TEST_END();
}

// ---------------------------------------------------------------------
// Duty-cycle math — both the permille helper (native) and the v3
// bit-identical helper (firmware-side).
// ---------------------------------------------------------------------

void test_duty_for_current_boundaries() {
    TEST_BEGIN("duty_for_current boundaries");
    using namespace evse;
    EXPECT_EQ(1000u, duty_for_current(0.0f));   // <6A → pilot steady +12V
    EXPECT_EQ(1000u, duty_for_current(5.9f));
    EXPECT_EQ(100u,  duty_for_current(6.0f));   // 6A = 10% duty
    EXPECT_EQ(267u,  duty_for_current(16.0f));  // ~26.7%
    EXPECT_EQ(850u,  duty_for_current(51.0f));  // 51A = 85% (top of first band)
    EXPECT_EQ(844u,  duty_for_current(51.1f));  // jumps into second band
    EXPECT_EQ(960u,  duty_for_current(80.0f));  // 80A = 96% (top of second band)
    EXPECT_EQ(1000u, duty_for_current(80.1f));  // >80A → off
    EXPECT_EQ(1000u, duty_for_current(200.0f));
    TEST_END();
}

void test_current_for_duty_inverse() {
    TEST_BEGIN("current_for_duty inverse");
    using namespace evse;
    EXPECT_EQ(0, static_cast<int>(current_for_duty(1000)));
    EXPECT_EQ(6, static_cast<int>(current_for_duty(100)));   // 10% = 6A
    // Permille-to-amps is lossy, just check ballpark on a couple points.
    float sixteen = current_for_duty(267);
    if (!(sixteen >= 15.9f && sixteen <= 16.1f)) {
        std::printf("  current_for_duty(267)=%.3f, expected ~16\n", sixteen);
        g_test_failed = true;
    }
    TEST_END();
}

void test_cp_duty_v3_matches_firmware() {
    TEST_BEGIN("cp_duty_for_current_v3 firmware-identical");
    using namespace evse;
    // Hand-computed spot checks, pinned to main.cpp::SetCurrent()'s
    // v3 branch. MIN_CURRENT defaults to 6, so 60 dA is the floor.
    EXPECT_EQ(102u, cp_duty_for_current_v3(60,  60));   // 60/0.6=100 → ×1024/1000 = 102
    EXPECT_EQ(272u, cp_duty_for_current_v3(160, 60));   // 160/0.6=266 (truncated) → 272
    EXPECT_EQ(870u, cp_duty_for_current_v3(510, 60));   // 510/0.6=850 → 870
    EXPECT_EQ(864u, cp_duty_for_current_v3(511, 60));   // second branch: 204+640=844 → 864
    EXPECT_EQ(983u, cp_duty_for_current_v3(800, 60));   // 320+640=960 → 983
    EXPECT_EQ(102u, cp_duty_for_current_v3(59,  60));   // under min → fallback 100 → 102
    EXPECT_EQ(102u, cp_duty_for_current_v3(801, 60));   // over 800   → fallback
    EXPECT_EQ(102u, cp_duty_for_current_v3(0,   60));
    // MIN_CURRENT=5 compile override: 50 dA is now valid.
    EXPECT_EQ(84u,  cp_duty_for_current_v3(50,  50));   // 50/0.6=83 → ×1024/1000 = 84
    EXPECT_EQ(102u, cp_duty_for_current_v3(49,  50));   // still under floor
    TEST_END();
}

// ---------------------------------------------------------------------
// Core tick_10ms — microcosm of the scenario corpus, no harness.
// ---------------------------------------------------------------------

class RecordingObserver : public evse::Observer {
 public:
    struct StateChange { evse::State from, to; std::string reason; };
    std::vector<StateChange> states;
    std::vector<std::pair<uint8_t, bool>> contactors;

    void on_state(evse::State f, evse::State t, const char* r) override {
        states.push_back({f, t, r ? r : ""});
    }
    void on_contactor(uint8_t p, bool c) override { contactors.push_back({p, c}); }
};

void test_core_plug_in_smart_charges() {
    TEST_BEGIN("Core A→B→C on pilot 9V then 6V");
    using namespace evse;
    RecordingObserver obs;
    Core core(&obs);
    Inputs in;
    in.pilot = PILOT_9V;
    core.tick_10ms(in);
    EXPECT_EQ(1u, obs.states.size());  // A→B
    EXPECT_EQ(STATE_B, obs.states[0].to);
    in.pilot = PILOT_6V;
    core.tick_10ms(in);
    EXPECT_EQ(2u, obs.states.size());  // A→B then B→C
    EXPECT_EQ(STATE_C, obs.states[1].to);
    EXPECT_EQ(2u, obs.contactors.size());  // phases 1 and 2 closed
    EXPECT_EQ(1, obs.contactors[0].first);
    EXPECT_EQ(true, obs.contactors[0].second);
    TEST_END();
}

void test_core_mode_off_blocks_charge() {
    TEST_BEGIN("Core mode=OFF blocks B→C");
    using namespace evse;
    RecordingObserver obs;
    Core core(&obs);
    Inputs in;
    in.mode = MODE_OFF;
    in.access_on = false;
    in.pilot = PILOT_9V;
    core.tick_10ms(in);  // A→B still happens (car showed up)
    in.pilot = PILOT_6V;
    core.tick_10ms(in);  // B→C must NOT happen
    EXPECT_EQ(STATE_B, core.state());
    EXPECT_EQ(0u, obs.contactors.size());
    TEST_END();
}

void test_core_rcm_fault_bypasses_grace() {
    TEST_BEGIN("Core RCM fault opens contactors within 20 ms");
    using namespace evse;
    RecordingObserver obs;
    Core core(&obs);
    Inputs in;
    in.pilot = PILOT_9V; core.tick_10ms(in);
    in.pilot = PILOT_6V; core.tick_10ms(in);
    // We should now be in C with contactors closed.
    EXPECT_EQ(STATE_C, core.state());
    // Trip RCM. First tick: C→C1 (fault). Second tick: C1→B1 (fault).
    in.error_flags |= ERR_RCM_TRIPPED;
    core.tick_10ms(in);
    EXPECT_EQ(STATE_C1, core.state());
    core.tick_10ms(in);
    EXPECT_EQ(STATE_B1, core.state());
    // Contactors must be open — 2 opens were recorded in addition to the
    // 2 closes from B→C.
    int opens = 0;
    for (auto& c : obs.contactors) if (!c.second) ++opens;
    EXPECT_EQ(2, opens);
    TEST_END();
}

// ---------------------------------------------------------------------
// limits::decide_current_override — mirrors esp32.cpp::mqtt_receive_callback
// /Set/CurrentOverride branch. Each case maps to an observable firmware
// outcome (no echo = reject, echo with 0 = clear, echo with value = apply).
// ---------------------------------------------------------------------

void test_limits_clear_on_zero() {
    TEST_BEGIN("limits Clear on requested=0");
    using namespace evse::limits;
    // Even a slave with Mode=Off must still accept "clear" — matches the
    // firmware's `if (RequestedCurrent == 0) setOverrideCurrent(0)` that
    // runs before any guard.
    auto d = decide_current_override(/*req*/0, /*min*/60, /*max*/160,
                                     /*LoadBl*/3, /*Mode*/255);
    EXPECT_EQ(static_cast<int>(Action::Clear), static_cast<int>(d.action));
    EXPECT_EQ(0u, d.value);
    TEST_END();
}

void test_limits_apply_in_range() {
    TEST_BEGIN("limits Apply on in-range Normal/Smart");
    using namespace evse::limits;
    auto a = decide_current_override(100, 60, 160, 0, /*Normal*/0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(a.action));
    EXPECT_EQ(100u, a.value);
    auto b = decide_current_override(160, 60, 160, 1, /*Smart*/1);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(b.action));
    EXPECT_EQ(160u, b.value);
    auto c = decide_current_override(60, 60, 160, 0, 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(c.action));
    EXPECT_EQ(60u, c.value);
    TEST_END();
}

void test_limits_reject_out_of_range() {
    TEST_BEGIN("limits Reject on out-of-range");
    using namespace evse::limits;
    // Below MinCurrent — user can disable by sending 0, not an under-min value.
    auto below = decide_current_override(59, 60, 160, 0, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(below.action));
    // Above MaxCurrent — the 99999 payload from scenario 07.
    auto above = decide_current_override(99999, 60, 160, 0, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(above.action));
    // Just above max.
    auto edge = decide_current_override(161, 60, 160, 0, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(edge.action));
    TEST_END();
}

void test_limits_reject_slave() {
    TEST_BEGIN("limits Reject when LoadBl>=2 (slave)");
    using namespace evse::limits;
    auto r2 = decide_current_override(100, 60, 160, 2, /*Smart*/1);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(r2.action));
    auto r7 = decide_current_override(100, 60, 160, 7, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(r7.action));
    // Master (LoadBl=1) still accepts.
    auto m = decide_current_override(100, 60, 160, 1, 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(m.action));
    TEST_END();
}

void test_limits_reject_non_normal_smart_mode() {
    TEST_BEGIN("limits Reject when Mode is Solar/Off");
    using namespace evse::limits;
    auto solar = decide_current_override(100, 60, 160, 0, /*Solar*/2);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(solar.action));
    auto off = decide_current_override(100, 60, 160, 0, /*Off*/255);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(off.action));
    TEST_END();
}

void test_limits_max_sum_mains_boundaries() {
    TEST_BEGIN("limits decide_max_sum_mains boundaries");
    using namespace evse::limits;
    // Clear on zero (disables MaxSumMains).
    auto c = decide_max_sum_mains(0, 0);
    EXPECT_EQ(static_cast<int>(Action::Clear), static_cast<int>(c.action));
    EXPECT_EQ(0u, c.value);
    // In-range [10, 600] accepted.
    auto lo = decide_max_sum_mains(10, 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(lo.action));
    EXPECT_EQ(10u, lo.value);
    auto hi = decide_max_sum_mains(600, 1);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(hi.action));
    EXPECT_EQ(600u, hi.value);
    auto mid = decide_max_sum_mains(100, 1);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(mid.action));
    // Below 10 or above 600 → Reject (not Clear — only exact 0 clears).
    auto under = decide_max_sum_mains(9, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(under.action));
    auto over = decide_max_sum_mains(601, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(over.action));
    auto far = decide_max_sum_mains(65535, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(far.action));
    TEST_END();
}

void test_limits_max_sum_mains_slave_rejects_even_zero() {
    TEST_BEGIN("limits decide_max_sum_mains slave rejects everything (incl. 0)");
    using namespace evse::limits;
    // Unlike decide_current_override, the firmware guards the entire
    // CurrentMaxSumMains topic on `LoadBl < 2` — slaves can't clear or
    // apply. Pin that behavior here.
    auto z = decide_max_sum_mains(0, 2);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(z.action));
    auto v = decide_max_sum_mains(100, 2);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(v.action));
    auto bigrole = decide_max_sum_mains(100, 7);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(bigrole.action));
    TEST_END();
}

void test_limits_cp_pwm_override_release() {
    TEST_BEGIN("limits decide_cp_pwm_override Clear on -1");
    using namespace evse::limits;
    auto d = decide_cp_pwm_override(-1);
    EXPECT_EQ(static_cast<int>(Action::Clear), static_cast<int>(d.action));
    EXPECT_EQ(0u, d.value);
    TEST_END();
}

void test_limits_cp_pwm_override_disconnect() {
    TEST_BEGIN("limits decide_cp_pwm_override Apply(0) on 0");
    using namespace evse::limits;
    // 0 is Apply, not Clear — firmware distinguishes "release override"
    // (duty=1024, CPDutyOverride=false) from "override to zero duty"
    // (duty=0, CPDutyOverride=true). The harness-side pilot_connected
    // derivation is `value > 0`, so value=0 drives PILOT_DISCONNECTED.
    auto d = decide_cp_pwm_override(0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
    EXPECT_EQ(0u, d.value);
    TEST_END();
}

void test_limits_cp_pwm_override_apply() {
    TEST_BEGIN("limits decide_cp_pwm_override Apply on 1..1024");
    using namespace evse::limits;
    auto lo = decide_cp_pwm_override(1);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(lo.action));
    EXPECT_EQ(1u, lo.value);
    auto mid = decide_cp_pwm_override(512);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(mid.action));
    EXPECT_EQ(512u, mid.value);
    auto hi = decide_cp_pwm_override(1024);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(hi.action));
    EXPECT_EQ(1024u, hi.value);
    TEST_END();
}

void test_limits_cp_pwm_override_reject_out_of_range() {
    TEST_BEGIN("limits decide_cp_pwm_override Reject on >1024 and negative-beyond--1");
    using namespace evse::limits;
    auto above = decide_cp_pwm_override(1025);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(above.action));
    auto big = decide_cp_pwm_override(2000);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(big.action));
    // FEEDBACK §8: legacy inline chain accepted pwm=-2 and passed it to
    // SetCPDuty((uint32_t)-2) = 0xFFFFFFFE. Helper rejects.
    auto negtwo = decide_cp_pwm_override(-2);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(negtwo.action));
    auto big_neg = decide_cp_pwm_override(-9999);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(big_neg.action));
    TEST_END();
}

void test_limits_set_mode_accept_five_keywords() {
    TEST_BEGIN("limits decide_set_mode accepts Off/Normal/Smart/Solar/Pause");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_set_mode(s, static_cast<uint32_t>(std::strlen(s)));
    };
    auto off = run("Off");
    EXPECT_EQ(static_cast<int>(ModeAction::ApplyAccess), static_cast<int>(off.action));
    EXPECT_EQ(static_cast<int>(kAccessOff), static_cast<int>(off.target));
    auto normal = run("Normal");
    EXPECT_EQ(static_cast<int>(ModeAction::ApplyMode), static_cast<int>(normal.action));
    EXPECT_EQ(static_cast<int>(kModeNormal), static_cast<int>(normal.target));
    auto smart = run("Smart");
    EXPECT_EQ(static_cast<int>(ModeAction::ApplyMode), static_cast<int>(smart.action));
    EXPECT_EQ(static_cast<int>(kModeSmart), static_cast<int>(smart.target));
    auto solar = run("Solar");
    EXPECT_EQ(static_cast<int>(ModeAction::ApplyMode), static_cast<int>(solar.action));
    EXPECT_EQ(static_cast<int>(kModeSolar), static_cast<int>(solar.target));
    auto pause = run("Pause");
    EXPECT_EQ(static_cast<int>(ModeAction::ApplyAccess), static_cast<int>(pause.action));
    EXPECT_EQ(static_cast<int>(kAccessPause), static_cast<int>(pause.target));
    TEST_END();
}

void test_limits_enable_c2_digit_accepts_0_to_4() {
    TEST_BEGIN("limits decide_enable_c2 accepts digits 0..4");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_enable_c2(s, static_cast<uint32_t>(std::strlen(s)));
    };
    for (int i = 0; i <= 4; ++i) {
        char buf[2] = { static_cast<char>('0' + i), 0 };
        auto d = run(buf);
        EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
        EXPECT_EQ(static_cast<int>(i), static_cast<int>(d.value));
    }
    // Legacy String::toInt-style digit prefix acceptance: "3abc" → 3.
    auto d = run("3abc");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
    EXPECT_EQ(3u, d.value);
    TEST_END();
}

void test_limits_enable_c2_digit_rejects_out_of_range() {
    TEST_BEGIN("limits decide_enable_c2 rejects digits >4");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_enable_c2(s, static_cast<uint32_t>(std::strlen(s)));
    };
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("5").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("99").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("100").action));
    TEST_END();
}

void test_limits_enable_c2_string_accepts_five_names() {
    TEST_BEGIN("limits decide_enable_c2 accepts five string names");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_enable_c2(s, static_cast<uint32_t>(std::strlen(s)));
    };
    struct { const char* name; uint16_t idx; } table[] = {
        {"Not present", 0}, {"Always Off", 1}, {"Solar Off", 2},
        {"Always On", 3},   {"Auto", 4},
    };
    for (auto& t : table) {
        auto d = run(t.name);
        EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
        EXPECT_EQ(static_cast<int>(t.idx), static_cast<int>(d.value));
    }
    TEST_END();
}

void test_limits_enable_c2_rejects_junk_and_case() {
    TEST_BEGIN("limits decide_enable_c2 rejects junk and wrong case");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_enable_c2(s, static_cast<uint32_t>(std::strlen(s)));
    };
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("auto").action));     // lowercase
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("AUTO").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("Auto ").action));    // trailing space
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("Something else").action));
    // Leading non-digit non-'A'/'N'/'S': rejected (starts neither digit
    // branch nor any table entry).
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("-1").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("+2").action));
    TEST_END();
}

void test_limits_mains_meter_accept_triple() {
    TEST_BEGIN("limits decide_mains_meter accepts valid L1:L2:L3");
    using namespace evse::limits;
    auto run = [](const char* s, bool em_api, uint8_t lb) {
        return decide_mains_meter(s, static_cast<uint32_t>(std::strlen(s)), em_api, lb);
    };
    auto d = run("100:-50:200", /*em_api*/true, /*LoadBl*/1);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
    EXPECT_EQ(100, static_cast<int>(d.l1));
    EXPECT_EQ(-50, static_cast<int>(d.l2));
    EXPECT_EQ(200, static_cast<int>(d.l3));
    // Standalone (LoadBl=0) and open-interval boundaries just below 2000.
    auto e = run("1999:-1999:0", true, 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(e.action));
    EXPECT_EQ(1999, static_cast<int>(e.l1));
    EXPECT_EQ(-1999, static_cast<int>(e.l2));
    EXPECT_EQ(0, static_cast<int>(e.l3));
    // Leading '+' and '0' handled like sscanf("%d").
    auto f = run("+10:0:-0", true, 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(f.action));
    EXPECT_EQ(10, static_cast<int>(f.l1));
    TEST_END();
}

void test_limits_mains_meter_reject_role_and_type() {
    TEST_BEGIN("limits decide_mains_meter rejects slave and non-EM_API");
    using namespace evse::limits;
    auto run = [](const char* s, bool em_api, uint8_t lb) {
        return decide_mains_meter(s, static_cast<uint32_t>(std::strlen(s)), em_api, lb);
    };
    // Slave rejects even a perfectly valid payload.
    auto slave = run("100:100:100", true, 2);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(slave.action));
    auto bigrole = run("100:100:100", true, 9);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(bigrole.action));
    // Non-EM_API meter (firmware-configured, not API-fed) rejects.
    auto notapi = run("100:100:100", false, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(notapi.action));
    TEST_END();
}

void test_limits_mains_meter_reject_parse() {
    TEST_BEGIN("limits decide_mains_meter rejects malformed payloads");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_mains_meter(s, static_cast<uint32_t>(std::strlen(s)), true, 0);
    };
    // Empty / null.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("").action));
    auto nullp = decide_mains_meter(nullptr, 0, true, 0);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(nullp.action));
    // Missing separators.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("100").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("100:200").action));
    // Wrong separator.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("100,200,300").action));
    // Trailing garbage after third value — sscanf("%d:%d:%d") tolerates it
    // silently, and so do we (the third parse just stops at the first
    // non-digit). If the stricter "no trailing garbage" rule is wanted
    // later, tighten the helper — for now match legacy behavior.
    auto trailing = run("100:200:300abc");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(trailing.action));
    EXPECT_EQ(300, static_cast<int>(trailing.l3));
    // Non-numeric first field.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("abc:100:100").action));
    // Double colon (empty middle field).
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("100::200").action));
    // Lone sign with no digits.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("-:100:100").action));
    TEST_END();
}

void test_limits_mains_meter_reject_range() {
    TEST_BEGIN("limits decide_mains_meter rejects out-of-range (open interval)");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_mains_meter(s, static_cast<uint32_t>(std::strlen(s)), true, 0);
    };
    // Exactly 2000 / -2000 is rejected — firmware uses strict <, >.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("2000:0:0").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("-2000:0:0").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("0:2000:0").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("0:0:-2000").action));
    // Big positive / negative.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("99999:0:0").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("0:0:-99999").action));
    TEST_END();
}

void test_limits_ev_meter_accept_all_fields() {
    TEST_BEGIN("limits decide_ev_meter accepts full L1:L2:L3:W:WH");
    using namespace evse::limits;
    auto run = [](const char* s, bool em_api) {
        return decide_ev_meter(s, static_cast<uint32_t>(std::strlen(s)), em_api);
    };
    // 100 dA, 200 dA, 300 dA currents; 3500 W; 42000 Wh.
    auto d = run("100:200:300:3500:42000", true);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
    EXPECT_EQ(1, d.apply_currents ? 1 : 0);
    EXPECT_EQ(100, static_cast<int>(d.l1));
    EXPECT_EQ(200, static_cast<int>(d.l2));
    EXPECT_EQ(300, static_cast<int>(d.l3));
    EXPECT_EQ(1, d.apply_power ? 1 : 0);
    EXPECT_EQ(3500, static_cast<int>(d.power_w));
    EXPECT_EQ(1, d.apply_energy ? 1 : 0);
    EXPECT_EQ(42000, static_cast<int>(d.energy_wh));
    // Boundary: 999 dA is the max accepted current (1000 excluded).
    auto edge = run("999:0:0:0:0", true);
    EXPECT_EQ(1, edge.apply_currents ? 1 : 0);
    EXPECT_EQ(999, static_cast<int>(edge.l1));
    TEST_END();
}

void test_limits_ev_meter_partial_apply() {
    TEST_BEGIN("limits decide_ev_meter partial-apply arms gate independently");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_ev_meter(s, static_cast<uint32_t>(std::strlen(s)), true);
    };
    // Only currents — caller sends -1 for power+energy to skip them.
    auto c = run("50:60:70:-1:-1");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(c.action));
    EXPECT_EQ(1, c.apply_currents ? 1 : 0);
    EXPECT_EQ(0, c.apply_power ? 1 : 0);
    EXPECT_EQ(0, c.apply_energy ? 1 : 0);
    // Only power.
    auto p = run("-1:-1:-1:2500:-1");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(p.action));
    EXPECT_EQ(0, p.apply_currents ? 1 : 0);
    EXPECT_EQ(1, p.apply_power ? 1 : 0);
    EXPECT_EQ(2500, static_cast<int>(p.power_w));
    EXPECT_EQ(0, p.apply_energy ? 1 : 0);
    // Only energy.
    auto e = run("-1:-1:-1:-1:12345");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(e.action));
    EXPECT_EQ(0, e.apply_currents ? 1 : 0);
    EXPECT_EQ(0, e.apply_power ? 1 : 0);
    EXPECT_EQ(1, e.apply_energy ? 1 : 0);
    EXPECT_EQ(12345, static_cast<int>(e.energy_wh));
    // One negative current gates ALL currents (firmware's &&-chain).
    auto single_neg = run("-1:50:60:2000:5000");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(single_neg.action));
    EXPECT_EQ(0, single_neg.apply_currents ? 1 : 0);  // whole block off
    EXPECT_EQ(1, single_neg.apply_power ? 1 : 0);     // power still admitted
    EXPECT_EQ(1, single_neg.apply_energy ? 1 : 0);
    // Currents >= 1000 dA also gate the block off.
    auto over = run("1000:50:60:-1:-1");
    EXPECT_EQ(0, over.apply_currents ? 1 : 0);
    TEST_END();
}

void test_limits_ev_meter_reject_role_and_parse() {
    TEST_BEGIN("limits decide_ev_meter rejects non-EM_API and parse failures");
    using namespace evse::limits;
    auto run = [](const char* s, bool em_api) {
        return decide_ev_meter(s, static_cast<uint32_t>(std::strlen(s)), em_api);
    };
    // Not EM_API — payload ignored wholesale.
    auto notapi = run("100:100:100:100:100", false);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(notapi.action));
    // Fewer than 5 fields.
    EXPECT_EQ(static_cast<int>(Action::Reject),
              static_cast<int>(run("1:2:3:4", true).action));
    EXPECT_EQ(static_cast<int>(Action::Reject),
              static_cast<int>(run("1:2:3", true).action));
    // Empty.
    EXPECT_EQ(static_cast<int>(Action::Reject),
              static_cast<int>(run("", true).action));
    // Non-numeric field.
    EXPECT_EQ(static_cast<int>(Action::Reject),
              static_cast<int>(run("1:2:x:4:5", true).action));
    // Wrong separator.
    EXPECT_EQ(static_cast<int>(Action::Reject),
              static_cast<int>(run("1,2,3,4,5", true).action));
    // All-skip (all -1) still parses successfully → Apply with no arms
    // live. Firmware's n==5 gate accepts; the three sub-gates all fail.
    auto allskip = run("-1:-1:-1:-1:-1", true);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(allskip.action));
    EXPECT_EQ(0, allskip.apply_currents ? 1 : 0);
    EXPECT_EQ(0, allskip.apply_power ? 1 : 0);
    EXPECT_EQ(0, allskip.apply_energy ? 1 : 0);
    TEST_END();
}

void test_limits_cable_lock_accept_zero_and_one() {
    TEST_BEGIN("limits decide_cable_lock accepts exactly \"0\" and \"1\"");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_cable_lock(s, static_cast<uint32_t>(std::strlen(s)));
    };
    auto one = run("1");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(one.action));
    EXPECT_EQ(1u, one.value);
    auto zero = run("0");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(zero.action));
    EXPECT_EQ(0u, zero.value);
    TEST_END();
}

void test_limits_cable_lock_rejects_rest() {
    TEST_BEGIN("limits decide_cable_lock rejects everything else (tightening)");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_cable_lock(s, static_cast<uint32_t>(std::strlen(s)));
    };
    // Legacy: all these silently set CableLock=0 and called
    // request_write_settings(). Tightened helper rejects; call site
    // keeps the flash write for bit-identity (FEEDBACK §11).
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("2").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("01").action));  // "01" ≠ "1"
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("true").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("yes").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run(" 1").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("1 ").action));
    TEST_END();
}

void test_limits_home_battery_current_accepts_signed_int() {
    TEST_BEGIN("limits decide_home_battery_current accepts signed int on master");
    using namespace evse::limits;
    auto run = [](const char* s, uint8_t lb) {
        return decide_home_battery_current(s, static_cast<uint32_t>(std::strlen(s)), lb);
    };
    auto pos = run("100", 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(pos.action));
    EXPECT_EQ(100, static_cast<int>(pos.value));
    auto neg = run("-50", 1);  // master (LoadBl=1) accepts
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(neg.action));
    EXPECT_EQ(-50, static_cast<int>(neg.value));
    auto zero = run("0", 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(zero.action));
    EXPECT_EQ(0, static_cast<int>(zero.value));
    // Saturate at int16 bounds.
    auto overflow = run("99999", 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(overflow.action));
    EXPECT_EQ(32767, static_cast<int>(overflow.value));
    auto underflow = run("-99999", 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(underflow.action));
    EXPECT_EQ(-32768, static_cast<int>(underflow.value));
    TEST_END();
}

void test_limits_home_battery_current_rejects_slave_and_garbage() {
    TEST_BEGIN("limits decide_home_battery_current rejects slave and non-numeric");
    using namespace evse::limits;
    auto run = [](const char* s, uint8_t lb) {
        return decide_home_battery_current(s, static_cast<uint32_t>(std::strlen(s)), lb);
    };
    // Slave rejects regardless of payload.
    auto slave = run("100", 2);
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(slave.action));
    // Non-numeric — FEEDBACK §10 tightening. Legacy silently wrote 0.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("", 0).action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("garbage", 0).action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("-", 0).action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run(" 100", 0).action));
    // Digit-prefix tolerated (matches String::toInt semantics).
    auto prefix = run("123abc", 0);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(prefix.action));
    EXPECT_EQ(123, static_cast<int>(prefix.value));
    TEST_END();
}

void test_limits_set_mode_reject_case_and_prefix() {
    TEST_BEGIN("limits decide_set_mode rejects wrong case, prefixes, and junk");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_set_mode(s, static_cast<uint32_t>(std::strlen(s)));
    };
    // Case sensitivity — firmware uses String::operator== which is exact.
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run("off").action));
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run("SMART").action));
    // Prefix / suffix — "Offline" must not match "Off".
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run("Offline").action));
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run("SmartX").action));
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run("XSmart").action));
    // Empty + whitespace + junk.
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run("").action));
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run(" Off ").action));
    EXPECT_EQ(static_cast<int>(ModeAction::Reject), static_cast<int>(run("garbage").action));
    TEST_END();
}

void test_limits_color_rgb_accepts_valid_triples() {
    TEST_BEGIN("limits decide_color_rgb accepts canonical R,G,B in [0, 256)");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_color_rgb(s, static_cast<uint32_t>(std::strlen(s)));
    };
    auto mid = run("128,64,200");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(mid.action));
    EXPECT_EQ(128, static_cast<int>(mid.r));
    EXPECT_EQ(64,  static_cast<int>(mid.g));
    EXPECT_EQ(200, static_cast<int>(mid.b));
    // Boundaries: 0 and 255 both accepted.
    auto zero = run("0,0,0");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(zero.action));
    EXPECT_EQ(0, static_cast<int>(zero.r));
    auto maxv = run("255,255,255");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(maxv.action));
    EXPECT_EQ(255, static_cast<int>(maxv.b));
    // Trailing garbage is ignored (matches sscanf).
    auto trailing = run("10,20,30 trailing");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(trailing.action));
    EXPECT_EQ(30, static_cast<int>(trailing.b));
    TEST_END();
}

void test_limits_custom_button_on_vs_off() {
    TEST_BEGIN("limits decide_custom_button Apply(1) on \"On\", Apply(0) otherwise");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_custom_button(s, static_cast<uint32_t>(std::strlen(s)));
    };
    // Exact "On" → 1.
    auto on = run("On");
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(on.action));
    EXPECT_EQ(1, static_cast<int>(on.value));
    // Everything else → Apply(0). Legacy had no Reject arm.
    EXPECT_EQ(0, static_cast<int>(run("Off").value));
    EXPECT_EQ(0, static_cast<int>(run("ON").value));    // case-sensitive
    EXPECT_EQ(0, static_cast<int>(run("on").value));
    EXPECT_EQ(0, static_cast<int>(run("Online").value));  // prefix != match
    EXPECT_EQ(0, static_cast<int>(run("").value));
    EXPECT_EQ(0, static_cast<int>(run("garbage").value));
    // All return Apply, never Reject.
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(run("").action));
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(run("Off").action));
    TEST_END();
}

void test_limits_rfid_uid_6_byte() {
    TEST_BEGIN("limits decide_rfid_uid 12-hex → 6-byte UID with 0x01 family prefix and CRC");
    using namespace evse::limits;
    auto run = [](const char* s, bool enabled) {
        return decide_rfid_uid(s, static_cast<uint32_t>(std::strlen(s)), enabled);
    };
    // "010203040506" → uid = [0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, CRC].
    auto d = run("010203040506", true);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
    EXPECT_EQ(0x01, static_cast<int>(d.uid[0]));  // family prefix
    EXPECT_EQ(0x01, static_cast<int>(d.uid[1]));
    EXPECT_EQ(0x02, static_cast<int>(d.uid[2]));
    EXPECT_EQ(0x06, static_cast<int>(d.uid[6]));
    // CRC-8 Dallas of {0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06} = 0xbd
    // (verified with an independent Python impl against polynomial 0x8C).
    EXPECT_EQ(0xbd, static_cast<int>(d.uid[7]));
    // Lowercase hex accepted.
    auto lower = run("abcdef123456", true);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(lower.action));
    EXPECT_EQ(0xab, static_cast<int>(lower.uid[1]));
    // Whitespace trimmed.
    auto trimmed = run("  010203040506  ", true);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(trimmed.action));
    EXPECT_EQ(0x06, static_cast<int>(trimmed.uid[6]));
    TEST_END();
}

void test_limits_rfid_uid_7_byte() {
    TEST_BEGIN("limits decide_rfid_uid 14-hex → 7-byte UID with CRC, no family prefix");
    using namespace evse::limits;
    auto d = decide_rfid_uid("04A1B2C3D4E5F6", 14, true);
    EXPECT_EQ(static_cast<int>(Action::Apply), static_cast<int>(d.action));
    EXPECT_EQ(0x04, static_cast<int>(d.uid[0]));  // no family prefix
    EXPECT_EQ(0xa1, static_cast<int>(d.uid[1]));
    EXPECT_EQ(0xf6, static_cast<int>(d.uid[6]));
    // CRC-8 Dallas of {0x04,0xA1,0xB2,0xC3,0xD4,0xE5,0xF6} = 0x46.
    EXPECT_EQ(0x46, static_cast<int>(d.uid[7]));
    TEST_END();
}

void test_limits_rfid_uid_rejects() {
    TEST_BEGIN("limits decide_rfid_uid rejects disabled / bad hex / wrong length");
    using namespace evse::limits;
    auto run = [](const char* s, bool enabled) {
        return decide_rfid_uid(s, static_cast<uint32_t>(std::strlen(s)), enabled);
    };
    // Reader disabled — reject regardless of payload.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("010203040506", false).action));
    // Non-hex character.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("01020304050Z", true).action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("ghijklmnopqr", true).action));
    // Wrong lengths (10, 13, 16 — 13 has odd length; none match {12, 14}).
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("0102030405", true).action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("01020304050607A", true).action));  // 15
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("0102030405060708", true).action)); // 16
    // Empty / whitespace-only.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("", true).action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("   ", true).action));
    TEST_END();
}

void test_limits_color_rgb_rejects_out_of_range_and_parse() {
    TEST_BEGIN("limits decide_color_rgb rejects >=256, negative, parse fail");
    using namespace evse::limits;
    auto run = [](const char* s) {
        return decide_color_rgb(s, static_cast<uint32_t>(std::strlen(s)));
    };
    // Range: 256 is exclusive upper bound.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("256,0,0").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("0,256,0").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("0,0,999").action));
    // Negative — sscanf("%d") accepted it, helper rejects on range.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("-1,0,0").action));
    // Parse fail: too few fields.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("10,20").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("10").action));
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("").action));
    // Wrong separator — colon instead of comma.
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run("10:20:30").action));
    // Leading whitespace — strict parser (tightening from sscanf).
    EXPECT_EQ(static_cast<int>(Action::Reject), static_cast<int>(run(" 10,20,30").action));
    TEST_END();
}

void test_check_switching_phases_all_reasons() {
    TEST_BEGIN("check_switching_phases covers all nine PhaseSwitchReason branches");
    using evse::check_switching_phases;
    using evse::MODE_NORMAL; using evse::MODE_SMART; using evse::MODE_SOLAR; using evse::MODE_OFF;
    using evse::C2_NOT_PRESENT; using evse::C2_ALWAYS_OFF; using evse::C2_SOLAR_OFF;
    using evse::C2_AUTO; using evse::C2_ALWAYS_ON;
    using evse::STATE_A; using evse::STATE_B; using evse::STATE_C;
    // Switch_Phase_t values from main.h:325 — { NO_SWITCH=0, GOING_TO_SWITCH_1P=1, GOING_TO_SWITCH_3P=2 }
    constexpr uint8_t NO_SW   = 0;
    constexpr uint8_t TO_1P   = 1;
    constexpr uint8_t TO_3P   = 2;

    // C2_ALWAYS_OFF forces 1P. Currently 3P and in STATE_C → STAGE_1P.
    EXPECT_EQ(static_cast<int>(evse::PSR_STAGE_1P),
              static_cast<int>(check_switching_phases(C2_ALWAYS_OFF, MODE_NORMAL, STATE_C, 3, NO_SW)));
    // Currently 3P and in STATE_A → SET_1P_IMMEDIATE (no _LOG_A branch).
    EXPECT_EQ(static_cast<int>(evse::PSR_SET_1P_IMMEDIATE),
              static_cast<int>(check_switching_phases(C2_ALWAYS_OFF, MODE_NORMAL, STATE_A, 3, NO_SW)));
    // Currently 1P → ALREADY_1P (writes NO_SWITCH, logs "No need…").
    EXPECT_EQ(static_cast<int>(evse::PSR_ALREADY_1P),
              static_cast<int>(check_switching_phases(C2_ALWAYS_OFF, MODE_NORMAL, STATE_C, 1, NO_SW)));

    // C2_ALWAYS_ON forces 3P. Currently 1P and in STATE_B → STAGE_3P.
    EXPECT_EQ(static_cast<int>(evse::PSR_STAGE_3P),
              static_cast<int>(check_switching_phases(C2_ALWAYS_ON, MODE_NORMAL, STATE_B, 1, NO_SW)));
    // Currently 1P and in STATE_A → SET_3P_IMMEDIATE.
    EXPECT_EQ(static_cast<int>(evse::PSR_SET_3P_IMMEDIATE),
              static_cast<int>(check_switching_phases(C2_ALWAYS_ON, MODE_NORMAL, STATE_A, 1, NO_SW)));
    // Currently 3P → ALREADY_3P.
    EXPECT_EQ(static_cast<int>(evse::PSR_ALREADY_3P),
              static_cast<int>(check_switching_phases(C2_ALWAYS_ON, MODE_NORMAL, STATE_C, 3, NO_SW)));

    // C2_SOLAR_OFF in MODE_SOLAR forces 1P (same as ALWAYS_OFF in solar).
    EXPECT_EQ(static_cast<int>(evse::PSR_STAGE_1P),
              static_cast<int>(check_switching_phases(C2_SOLAR_OFF, MODE_SOLAR, STATE_C, 3, NO_SW)));

    // C2_AUTO with MODE_SOLAR — obey_c2 is true because mode==SOLAR, and
    // force_1P returns (nr_phases==1). With nr_phases=1 → ALREADY_1P.
    EXPECT_EQ(static_cast<int>(evse::PSR_ALREADY_1P),
              static_cast<int>(check_switching_phases(C2_AUTO, MODE_SOLAR, STATE_C, 1, NO_SW)));

    // C2_AUTO + MODE_SMART: falls into the SMART arm.
    //   nr_phases != 3 → AUTO_STAGE_3P.
    EXPECT_EQ(static_cast<int>(evse::PSR_AUTO_STAGE_3P),
              static_cast<int>(check_switching_phases(C2_AUTO, MODE_SMART, STATE_C, 1, NO_SW)));
    //   nr_phases == 3 and a pending switch → AUTO_CANCEL.
    EXPECT_EQ(static_cast<int>(evse::PSR_AUTO_CANCEL),
              static_cast<int>(check_switching_phases(C2_AUTO, MODE_SMART, STATE_C, 3, TO_1P)));
    EXPECT_EQ(static_cast<int>(evse::PSR_AUTO_CANCEL),
              static_cast<int>(check_switching_phases(C2_AUTO, MODE_SMART, STATE_C, 3, TO_3P)));
    //   nr_phases == 3 and no pending switch → NOOP (no write, no log).
    EXPECT_EQ(static_cast<int>(evse::PSR_NOOP),
              static_cast<int>(check_switching_phases(C2_AUTO, MODE_SMART, STATE_C, 3, NO_SW)));

    // C2_AUTO + MODE_NORMAL or MODE_OFF: neither arm fires → NOOP.
    EXPECT_EQ(static_cast<int>(evse::PSR_NOOP),
              static_cast<int>(check_switching_phases(C2_AUTO, MODE_NORMAL, STATE_C, 1, NO_SW)));
    EXPECT_EQ(static_cast<int>(evse::PSR_NOOP),
              static_cast<int>(check_switching_phases(C2_AUTO, MODE_OFF, STATE_C, 1, TO_1P)));

    // C2_NOT_PRESENT always returns 3P — currently 1P → STAGE_3P.
    EXPECT_EQ(static_cast<int>(evse::PSR_STAGE_3P),
              static_cast<int>(check_switching_phases(C2_NOT_PRESENT, MODE_NORMAL, STATE_C, 1, NO_SW)));
    TEST_END();
}

void test_decide_state_c_contactor2() {
    TEST_BEGIN("decide_state_c_contactor2 mirrors STATE_C entry block");
    using evse::decide_state_c_contactor2;
    using evse::MODE_NORMAL; using evse::MODE_SMART; using evse::MODE_SOLAR;
    using evse::C2_NOT_PRESENT; using evse::C2_ALWAYS_OFF; using evse::C2_SOLAR_OFF;
    using evse::C2_AUTO; using evse::C2_ALWAYS_ON;
    // Switch_Phase_t values from main.h:325 — { NO_SWITCH=0, GOING_TO_SWITCH_1P=1, GOING_TO_SWITCH_3P=2 }
    constexpr uint8_t NO_SW = 0;
    constexpr uint8_t TO_1P = 1;
    constexpr uint8_t TO_3P = 2;

    // ALWAYS_OFF → always 1P → C2 off, regardless of switching pre-adjustment.
    EXPECT_EQ(false, decide_state_c_contactor2(C2_ALWAYS_OFF, MODE_NORMAL, 3, NO_SW));
    EXPECT_EQ(false, decide_state_c_contactor2(C2_ALWAYS_OFF, MODE_NORMAL, 3, TO_3P));

    // ALWAYS_ON → always 3P → C2 on.
    EXPECT_EQ(true, decide_state_c_contactor2(C2_ALWAYS_ON, MODE_NORMAL, 1, NO_SW));
    EXPECT_EQ(true, decide_state_c_contactor2(C2_ALWAYS_ON, MODE_NORMAL, 1, TO_1P));

    // NOT_PRESENT → always 3P → C2 on.
    EXPECT_EQ(true, decide_state_c_contactor2(C2_NOT_PRESENT, MODE_SOLAR, 1, NO_SW));

    // SOLAR_OFF + MODE_SOLAR → 1P → C2 off.
    EXPECT_EQ(false, decide_state_c_contactor2(C2_SOLAR_OFF, MODE_SOLAR, 3, NO_SW));
    // SOLAR_OFF + MODE_NORMAL → 3P → C2 on.
    EXPECT_EQ(true, decide_state_c_contactor2(C2_SOLAR_OFF, MODE_NORMAL, 3, NO_SW));

    // AUTO → follows effective nr_of_phases_charging after pre-adjustment.
    // Pending GOING_TO_SWITCH_1P overrides current 3 → effective=1 → 1P → C2 off.
    EXPECT_EQ(false, decide_state_c_contactor2(C2_AUTO, MODE_NORMAL, 3, TO_1P));
    // Pending GOING_TO_SWITCH_3P overrides current 1 → effective=3 → 3P → C2 on.
    EXPECT_EQ(true, decide_state_c_contactor2(C2_AUTO, MODE_NORMAL, 1, TO_3P));
    // No pending switch — current 1 stays 1 → 1P → C2 off.
    EXPECT_EQ(false, decide_state_c_contactor2(C2_AUTO, MODE_NORMAL, 1, NO_SW));
    // No pending switch — current 3 stays 3 → 3P → C2 on.
    EXPECT_EQ(true, decide_state_c_contactor2(C2_AUTO, MODE_NORMAL, 3, NO_SW));

    // AUTO + unusual effective phase count (e.g. 0 or 2) → force_1P returns 0
    // (AUTO only treats ==1 as 1P), so 3P / C2 on.
    EXPECT_EQ(true, decide_state_c_contactor2(C2_AUTO, MODE_SMART, 0, NO_SW));
    EXPECT_EQ(true, decide_state_c_contactor2(C2_AUTO, MODE_SMART, 2, NO_SW));
    TEST_END();
}

void test_decide_state_power_unavailable() {
    TEST_BEGIN("decide_state_power_unavailable fixpoints match setStatePowerUnavailable");
    using evse::decide_state_power_unavailable;
    using evse::STATE_A; using evse::STATE_B; using evse::STATE_C;
    using evse::STATE_D; using evse::STATE_B1; using evse::STATE_C1;

    // STATE_A is a fixpoint (no-op — already disconnected).
    EXPECT_EQ((int)STATE_A, (int)decide_state_power_unavailable(STATE_A));
    // STATE_C → STATE_C1 (charging: tell EV to stop).
    EXPECT_EQ((int)STATE_C1, (int)decide_state_power_unavailable(STATE_C));
    // STATE_C1 and STATE_B1 are fixpoints (already stopping / stopped).
    EXPECT_EQ((int)STATE_C1, (int)decide_state_power_unavailable(STATE_C1));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_power_unavailable(STATE_B1));
    // Everything else (B, D, modem states) → STATE_B1.
    EXPECT_EQ((int)STATE_B1, (int)decide_state_power_unavailable(STATE_B));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_power_unavailable(STATE_D));
    // Unknown states (e.g. STATE_MODEM_* which live in main.h, not evse::State)
    // fall through to STATE_B1.
    EXPECT_EQ((int)STATE_B1, (int)decide_state_power_unavailable(50));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_power_unavailable(255));
    TEST_END();
}

void test_decide_state_on_access_change() {
    TEST_BEGIN("decide_state_on_access_change mirrors setAccess's OFF/PAUSE transitions");
    using evse::decide_state_on_access_change;
    using evse::STATE_A; using evse::STATE_B; using evse::STATE_C;
    using evse::STATE_D; using evse::STATE_B1; using evse::STATE_C1;
    constexpr uint8_t ACCESS_OFF = 0;
    constexpr uint8_t ACCESS_ON  = 1;
    constexpr uint8_t ACCESS_PAUSE = 2;
    // STATE_MODEM_* from main_c.h (not in evse::State enum).
    constexpr uint8_t STATE_MODEM_REQUEST = 11;
    constexpr uint8_t STATE_MODEM_WAIT    = 12;
    constexpr uint8_t STATE_MODEM_DONE    = 13;
    constexpr uint8_t STATE_MODEM_DENIED  = 14;

    // access == ON: every state is a fixpoint (no transition).
    for (uint8_t s : {(uint8_t)STATE_A, (uint8_t)STATE_B, (uint8_t)STATE_C,
                      (uint8_t)STATE_D, (uint8_t)STATE_B1, (uint8_t)STATE_C1,
                      STATE_MODEM_REQUEST, STATE_MODEM_WAIT,
                      STATE_MODEM_DONE,    STATE_MODEM_DENIED}) {
        EXPECT_EQ((int)s, (int)decide_state_on_access_change(ACCESS_ON, s));
    }

    // access == OFF: STATE_C → STATE_C1.
    EXPECT_EQ((int)STATE_C1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_C));
    EXPECT_EQ((int)STATE_C1, (int)decide_state_on_access_change(ACCESS_PAUSE, STATE_C));

    // access == OFF: STATE_C1 is a fixpoint (legacy excluded via `State != STATE_C1`).
    EXPECT_EQ((int)STATE_C1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_C1));

    // access == OFF: STATE_B and modem states → STATE_B1.
    EXPECT_EQ((int)STATE_B1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_B));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_MODEM_REQUEST));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_MODEM_WAIT));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_MODEM_DONE));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_MODEM_DENIED));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_on_access_change(ACCESS_PAUSE, STATE_B));

    // access == OFF: STATE_A, STATE_B1, STATE_D are fixpoints (no-op).
    EXPECT_EQ((int)STATE_A,  (int)decide_state_on_access_change(ACCESS_OFF, STATE_A));
    EXPECT_EQ((int)STATE_B1, (int)decide_state_on_access_change(ACCESS_OFF, STATE_B1));
    EXPECT_EQ((int)STATE_D,  (int)decide_state_on_access_change(ACCESS_OFF, STATE_D));

    // Unknown access values (outside {OFF, ON, PAUSE}): treated as no-op
    // because the legacy check is `Access == OFF || Access == PAUSE`.
    EXPECT_EQ((int)STATE_C, (int)decide_state_on_access_change(99, STATE_C));
    TEST_END();
}

void test_decide_mode_switch_cp_reason() {
    TEST_BEGIN("decide_mode_switch_cp_reason fires on SOLAR_OFF boundary and AUTO Solar-exit 1P");
    using evse::decide_mode_switch_cp_reason;
    using evse::decide_cp_disconnect_target_on_mode_switch;
    using evse::MODE_NORMAL; using evse::MODE_SMART; using evse::MODE_SOLAR; using evse::MODE_OFF;
    using evse::C2_NOT_PRESENT; using evse::C2_ALWAYS_OFF; using evse::C2_SOLAR_OFF;
    using evse::C2_AUTO; using evse::C2_ALWAYS_ON;
    using evse::STATE_A; using evse::STATE_B; using evse::STATE_C;
    using evse::STATE_D; using evse::STATE_B1; using evse::STATE_C1;
    using evse::MSR_NOOP; using evse::MSR_SOLAR_OFF_C2_BOUNDARY; using evse::MSR_AUTO_SOLAR_EXIT_1P;

    // SOLAR_OFF + entering SOLAR → reason 1.
    EXPECT_EQ((int)MSR_SOLAR_OFF_C2_BOUNDARY,
              (int)decide_mode_switch_cp_reason(C2_SOLAR_OFF, MODE_NORMAL, MODE_SOLAR, 3));
    EXPECT_EQ((int)MSR_SOLAR_OFF_C2_BOUNDARY,
              (int)decide_mode_switch_cp_reason(C2_SOLAR_OFF, MODE_SMART,  MODE_SOLAR, 1));
    // SOLAR_OFF + leaving SOLAR → reason 1.
    EXPECT_EQ((int)MSR_SOLAR_OFF_C2_BOUNDARY,
              (int)decide_mode_switch_cp_reason(C2_SOLAR_OFF, MODE_SOLAR, MODE_NORMAL, 1));
    // SOLAR_OFF + same mode (no crossing) → NOOP.
    EXPECT_EQ((int)MSR_NOOP,
              (int)decide_mode_switch_cp_reason(C2_SOLAR_OFF, MODE_SOLAR, MODE_SOLAR, 1));
    EXPECT_EQ((int)MSR_NOOP,
              (int)decide_mode_switch_cp_reason(C2_SOLAR_OFF, MODE_NORMAL, MODE_SMART, 3));

    // AUTO + leaving SOLAR while 1P → reason 2.
    EXPECT_EQ((int)MSR_AUTO_SOLAR_EXIT_1P,
              (int)decide_mode_switch_cp_reason(C2_AUTO, MODE_SOLAR, MODE_NORMAL, 1));
    EXPECT_EQ((int)MSR_AUTO_SOLAR_EXIT_1P,
              (int)decide_mode_switch_cp_reason(C2_AUTO, MODE_SOLAR, MODE_SMART,  1));
    // AUTO + leaving SOLAR but already 3P → NOOP (no phase switch needed).
    EXPECT_EQ((int)MSR_NOOP,
              (int)decide_mode_switch_cp_reason(C2_AUTO, MODE_SOLAR, MODE_NORMAL, 3));
    // AUTO + entering SOLAR → NOOP (not covered by legacy block 2).
    EXPECT_EQ((int)MSR_NOOP,
              (int)decide_mode_switch_cp_reason(C2_AUTO, MODE_NORMAL, MODE_SOLAR, 1));
    // AUTO + SOLAR→SOLAR no-op → NOOP.
    EXPECT_EQ((int)MSR_NOOP,
              (int)decide_mode_switch_cp_reason(C2_AUTO, MODE_SOLAR, MODE_SOLAR, 1));

    // Other C2 values never fire.
    for (auto c2 : {C2_NOT_PRESENT, C2_ALWAYS_OFF, C2_ALWAYS_ON}) {
        EXPECT_EQ((int)MSR_NOOP,
                  (int)decide_mode_switch_cp_reason(c2, MODE_NORMAL, MODE_SOLAR, 1));
        EXPECT_EQ((int)MSR_NOOP,
                  (int)decide_mode_switch_cp_reason(c2, MODE_SOLAR,  MODE_NORMAL, 1));
    }

    // State-map helper: C→C1, B→B1, everything else is a fixpoint.
    EXPECT_EQ((int)STATE_C1, (int)decide_cp_disconnect_target_on_mode_switch(STATE_C));
    EXPECT_EQ((int)STATE_B1, (int)decide_cp_disconnect_target_on_mode_switch(STATE_B));
    EXPECT_EQ((int)STATE_A,  (int)decide_cp_disconnect_target_on_mode_switch(STATE_A));
    EXPECT_EQ((int)STATE_D,  (int)decide_cp_disconnect_target_on_mode_switch(STATE_D));
    EXPECT_EQ((int)STATE_B1, (int)decide_cp_disconnect_target_on_mode_switch(STATE_B1));
    EXPECT_EQ((int)STATE_C1, (int)decide_cp_disconnect_target_on_mode_switch(STATE_C1));
    // Modem state values (main_c.h 11..14) — not in evse::State, stay fixpoint.
    EXPECT_EQ(11, (int)decide_cp_disconnect_target_on_mode_switch(11));
    EXPECT_EQ(14, (int)decide_cp_disconnect_target_on_mode_switch(14));
    TEST_END();
}

void test_decide_solar_start_phase() {
    TEST_BEGIN("decide_solar_start_phase staging decision on Isum threshold");
    using evse::decide_solar_start_phase;
    using evse::MODE_NORMAL; using evse::MODE_SMART; using evse::MODE_SOLAR; using evse::MODE_OFF;
    using evse::C2_NOT_PRESENT; using evse::C2_ALWAYS_OFF; using evse::C2_SOLAR_OFF;
    using evse::C2_AUTO; using evse::C2_ALWAYS_ON;
    using evse::STATE_A; using evse::STATE_B; using evse::STATE_C;
    using evse::SSPR_NOOP; using evse::SSPR_STAGE_3P; using evse::SSPR_STAGE_1P;

    // Gating: only fires in MODE_SOLAR.
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_NORMAL, STATE_B, C2_AUTO, -10000, 6, 1));
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_SMART,  STATE_B, C2_AUTO, -10000, 6, 1));
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_OFF,    STATE_B, C2_AUTO, -10000, 6, 1));

    // Gating: only fires in STATE_B.
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_A, C2_AUTO, -10000, 6, 1));
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_C, C2_AUTO, -10000, 6, 1));

    // Gating: only fires for EnableC2 == AUTO.
    for (auto c2 : {C2_NOT_PRESENT, C2_ALWAYS_OFF, C2_SOLAR_OFF, C2_ALWAYS_ON}) {
        EXPECT_EQ((int)SSPR_NOOP,
                  (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, c2, -10000, 6, 1));
    }

    // Threshold math: three_phase_threshold = 30 * MinCurrent + 30.
    // MinCurrent=6A → threshold = 210 dA. Negative Isum means surplus.
    // -Isum >= threshold → stage 3P (if not already 3P).
    EXPECT_EQ((int)SSPR_STAGE_3P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -210, 6, 1));
    EXPECT_EQ((int)SSPR_STAGE_3P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -1000, 6, 1));

    // Exactly at threshold → STAGE_3P (>= test).
    EXPECT_EQ((int)SSPR_STAGE_3P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -210, 6, 1));

    // One dA below threshold → STAGE_1P.
    EXPECT_EQ((int)SSPR_STAGE_1P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -209, 6, 3));

    // Surplus above threshold but already 3P → NOOP.
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -1000, 6, 3));

    // Below threshold and already 1P → NOOP.
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, 0, 6, 1));
    EXPECT_EQ((int)SSPR_NOOP,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -100, 6, 1));

    // Import (positive Isum) → never above threshold, stage 1P if not already.
    EXPECT_EQ((int)SSPR_STAGE_1P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, 500, 6, 3));

    // MinCurrent scaling: 16A → threshold = 30*16+30 = 510 dA.
    EXPECT_EQ((int)SSPR_STAGE_3P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -510, 16, 1));
    EXPECT_EQ((int)SSPR_STAGE_1P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -509, 16, 3));

    // nr_of_phases_charging == 2 treated as "not 1" and "not 3" → stage in either direction.
    EXPECT_EQ((int)SSPR_STAGE_3P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, -1000, 6, 2));
    EXPECT_EQ((int)SSPR_STAGE_1P,
              (int)decide_solar_start_phase(MODE_SOLAR, STATE_B, C2_AUTO, 0, 6, 2));
    TEST_END();
}

void test_decide_solar_admission() {
    TEST_BEGIN("decide_solar_admission mirrors the three Solar guards in IsCurrentAvailable");
    using evse::decide_solar_admission;
    using evse::MODE_NORMAL; using evse::MODE_SMART; using evse::MODE_SOLAR; using evse::MODE_OFF;
    using evse::SAR_ALLOW; using evse::SAR_DENY_NO_SURPLUS;
    using evse::SAR_DENY_CANT_SPLIT; using evse::SAR_DENY_OVER_IMPORT;

    // Non-Solar modes always ALLOW — matches legacy "block inside if (Mode==SOLAR)".
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_NORMAL, 0, 0, 2, 6, 6, 0));
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SMART,  0, 0, 2, 6, 6, 0));
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_OFF,    0, 0, 2, 6, 6, 0));

    // Guard 1 — no active EVSE, not enough surplus.
    // StartCurrent=2A → threshold = -20 dA. Isum >= -20 denies.
    EXPECT_EQ((int)SAR_DENY_NO_SURPLUS,
              (int)decide_solar_admission(MODE_SOLAR, 0, -20, 2, 6, 6, 0));
    EXPECT_EQ((int)SAR_DENY_NO_SURPLUS,
              (int)decide_solar_admission(MODE_SOLAR, 0, 0,   2, 6, 6, 0));
    EXPECT_EQ((int)SAR_DENY_NO_SURPLUS,
              (int)decide_solar_admission(MODE_SOLAR, 0, 100, 2, 6, 6, 0));
    // One dA below threshold (-21 dA < -20 dA) → ALLOW.
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 0, -21, 2, 6, 6, 0));
    // StartCurrent=0 → threshold = 0 dA. Any Isum >= 0 denies.
    EXPECT_EQ((int)SAR_DENY_NO_SURPLUS,
              (int)decide_solar_admission(MODE_SOLAR, 0, 0, 0, 6, 6, 0));
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 0, -1, 0, 6, 6, 0));

    // Guard 2 — can't split. ActiveEVSE=1, MinCurrent=6 → min_slot=60 dA.
    // If TotalCurrent < 60 dA → DENY_CANT_SPLIT.
    EXPECT_EQ((int)SAR_DENY_CANT_SPLIT,
              (int)decide_solar_admission(MODE_SOLAR, 1, -1000, 2, 6, 6, 59));
    EXPECT_EQ((int)SAR_DENY_CANT_SPLIT,
              (int)decide_solar_admission(MODE_SOLAR, 1, -1000, 2, 6, 6, 0));
    // Exactly at threshold (60 dA) → guard 2 fails (>= not >), fall through.
    // Guard 3: Isum=-1000, budget = 6*10 + 60 - 60 = 60 dA, -1000 <= 60 → ALLOW.
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 1, -1000, 2, 6, 6, 60));
    // Two active EVSEs → min_slot=120 dA.
    EXPECT_EQ((int)SAR_DENY_CANT_SPLIT,
              (int)decide_solar_admission(MODE_SOLAR, 2, -1000, 2, 6, 6, 119));

    // Guard 3 — over import. ActiveEVSE=1, MinCurrent=6, ImportCurrent=0,
    // TotalCurrent=100 dA. Budget = 0 + 100 - 60 = 40 dA.
    // Isum > 40 dA → DENY_OVER_IMPORT.
    EXPECT_EQ((int)SAR_DENY_OVER_IMPORT,
              (int)decide_solar_admission(MODE_SOLAR, 1, 41, 2, 6, 0, 100));
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 1, 40, 2, 6, 0, 100));
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 1, -100, 2, 6, 0, 100));
    // With ImportCurrent=3A, budget = 30 + 100 - 60 = 70 dA. Isum=71 denies.
    EXPECT_EQ((int)SAR_DENY_OVER_IMPORT,
              (int)decide_solar_admission(MODE_SOLAR, 1, 71, 2, 6, 3, 100));
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 1, 70, 2, 6, 3, 100));

    // Guard 3 only fires when active_evse > 0 — with 0 active, we can
    // only trip guard 1 (surplus) above. Here we force guard 1 to pass
    // with strong surplus, guard 2 to pass (min_slot_total=0), and
    // confirm guard 3 is skipped.
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 0, -1000, 2, 6, 0, 0));

    // Full-allow happy path: strong surplus, split feasible, import OK.
    EXPECT_EQ((int)SAR_ALLOW,
              (int)decide_solar_admission(MODE_SOLAR, 1, -500, 2, 6, 10, 100));
    TEST_END();
}

void test_decide_circuit_admission() {
    TEST_BEGIN("decide_circuit_admission mirrors MaxMains/MaxCircuit/MaxSumMains guards");
    using evse::decide_circuit_admission;
    using evse::MODE_NORMAL; using evse::MODE_SMART; using evse::MODE_SOLAR; using evse::MODE_OFF;
    using evse::CAR_ALLOW; using evse::CAR_DENY_MAX_MAINS;
    using evse::CAR_DENY_MAX_CIRCUIT; using evse::CAR_DENY_MAX_SUM_MAINS;

    // --- MaxMains guard: Mode != NORMAL and ActiveEVSE*MinCurrent*10 + Baseload > MaxMains*10 ---
    // ActiveEVSE=1, MinCurrent=6, MaxMains=10. Threshold: 60 + Baseload > 100 → Baseload > 40.
    EXPECT_EQ((int)CAR_DENY_MAX_MAINS,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 41, 0, 0, 6, 10, 16, 0, 0, 0, 1));
    EXPECT_EQ((int)CAR_DENY_MAX_MAINS,
              (int)decide_circuit_admission(MODE_SMART, 1, 41, 0, 0, 6, 10, 16, 0, 0, 0, 1));
    // At threshold → allow (`>` not `>=`).
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 40, 0, 0, 6, 10, 16, 0, 0, 0, 1));
    // MODE_NORMAL skips MaxMains entirely.
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_NORMAL, 1, 1000, 0, 0, 6, 10, 16, 0, 0, 0, 1));

    // --- MaxCircuit guard: gated on (LoadBl==0 && ev_meter && Mode!=NORMAL) OR LoadBl==1 ---
    // Setup: ActiveEVSE=1, MinCurrent=6, MaxCircuit=10 → Baseload_EV threshold = 40.
    // LoadBl=0, ev_meter=1, SOLAR, Baseload=0, Baseload_EV=41 → DENY.
    EXPECT_EQ((int)CAR_DENY_MAX_CIRCUIT,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 41, 0, 6, 99, 10, 0, 0, 1, 1));
    // Same but ev_meter=0 → MaxCircuit not checked, ALLOW.
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 41, 0, 6, 99, 10, 0, 0, 0, 1));
    // LoadBl=0, ev_meter=1, NORMAL → MaxCircuit not checked.
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_NORMAL, 1, 0, 41, 0, 6, 99, 10, 0, 0, 1, 1));
    // LoadBl=1 (master): always checks MaxCircuit, including NORMAL.
    EXPECT_EQ((int)CAR_DENY_MAX_CIRCUIT,
              (int)decide_circuit_admission(MODE_NORMAL, 1, 0, 41, 0, 6, 99, 10, 0, 1, 0, 1));
    EXPECT_EQ((int)CAR_DENY_MAX_CIRCUIT,
              (int)decide_circuit_admission(MODE_SOLAR,  1, 0, 41, 0, 6, 99, 10, 0, 1, 0, 1));
    // LoadBl=2 (slave): no MaxCircuit check.
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 1000, 0, 6, 99, 10, 0, 2, 1, 1));
    // MaxCircuit threshold boundary.
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 40, 0, 6, 99, 10, 0, 0, 1, 1));

    // --- MaxSumMains guard: Mode != NORMAL and max_sum_mains != 0 ---
    // ActiveEVSE=1, MinCurrent=6, Phases=3 → scaled=180. MaxSumMains=20 → threshold 200.
    // scaled + Isum > 200 → Isum > 20.
    EXPECT_EQ((int)CAR_DENY_MAX_SUM_MAINS,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 0, 21, 6, 99, 99, 20, 0, 0, 3));
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 0, 20, 6, 99, 99, 20, 0, 0, 3));
    // Phases=1 halves scaled to 60. Threshold 200. Isum > 140 denies.
    EXPECT_EQ((int)CAR_DENY_MAX_SUM_MAINS,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 0, 141, 6, 99, 99, 20, 0, 0, 1));
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 0, 140, 6, 99, 99, 20, 0, 0, 1));
    // MaxSumMains=0 disables the check entirely.
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 0, 100000, 6, 99, 99, 0, 0, 0, 3));
    // MODE_NORMAL skips MaxSumMains.
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_NORMAL, 1, 0, 0, 100000, 6, 99, 99, 20, 0, 0, 3));

    // --- Guard ordering: MaxMains fires first ---
    // Craft a case where MaxMains, MaxCircuit, and MaxSumMains would all deny.
    EXPECT_EQ((int)CAR_DENY_MAX_MAINS,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 1000, 1000, 1000, 6, 10, 10, 10, 0, 1, 3));
    // Remove the MaxMains trigger → next is MaxCircuit.
    EXPECT_EQ((int)CAR_DENY_MAX_CIRCUIT,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 1000, 1000, 6, 99, 10, 10, 0, 1, 3));
    // Remove MaxCircuit trigger → MaxSumMains.
    EXPECT_EQ((int)CAR_DENY_MAX_SUM_MAINS,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 0, 1000, 6, 99, 99, 10, 0, 1, 3));

    // --- Happy path ---
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_SOLAR, 1, 0, 0, 0, 6, 16, 16, 16, 0, 1, 1));
    EXPECT_EQ((int)CAR_ALLOW,
              (int)decide_circuit_admission(MODE_NORMAL, 1, 0, 0, 0, 6, 16, 16, 16, 0, 1, 1));

    // --- Multi-EVSE scaling: two active EVSEs doubles min_slot_total. ---
    // MaxMains=10, ActiveEVSE=2, MinCurrent=6 → 120 + Baseload > 100 → always denies for Baseload>=0.
    EXPECT_EQ((int)CAR_DENY_MAX_MAINS,
              (int)decide_circuit_admission(MODE_SOLAR, 2, 0, 0, 0, 6, 10, 99, 0, 0, 0, 1));
    TEST_END();
}

void test_decide_solar_iset_balanced_delta_dA() {
    TEST_BEGIN("decide_solar_iset_balanced_delta_dA covers ramp/trim table");
    using evse::decide_solar_iset_balanced_delta_dA;

    // --- Gating: all three conditions must hold ---
    // ActiveEVSE == 0 → 0
    EXPECT_EQ(0, decide_solar_iset_balanced_delta_dA(0, 100, -50, true));
    // Idifference <= 0 → 0
    EXPECT_EQ(0, decide_solar_iset_balanced_delta_dA(1, 0, -50, true));
    EXPECT_EQ(0, decide_solar_iset_balanced_delta_dA(1, -10, -50, true));
    // phases_last_update_flag == false → 0
    EXPECT_EQ(0, decide_solar_iset_balanced_delta_dA(1, 100, -50, false));

    // --- Surplus (IsumImport < 0) ---
    // Strong surplus + strong headroom → +5 dA
    EXPECT_EQ(5, decide_solar_iset_balanced_delta_dA(1, 20, -20, true));
    EXPECT_EQ(5, decide_solar_iset_balanced_delta_dA(1, 11, -11, true));
    // IsumImport <= -10 fails inner test → trickle +1
    EXPECT_EQ(1, decide_solar_iset_balanced_delta_dA(1, 20, -10, true));
    // Idifference <= 10 fails inner test → trickle +1
    EXPECT_EQ(1, decide_solar_iset_balanced_delta_dA(1, 10, -20, true));
    // Just a tiny negative → trickle +1
    EXPECT_EQ(1, decide_solar_iset_balanced_delta_dA(1, 20, -1, true));

    // --- Over-draw (IsumImport >= 0) ---
    // Within 0.3 A dead band
    EXPECT_EQ(0, decide_solar_iset_balanced_delta_dA(1, 20, 0, true));
    EXPECT_EQ(0, decide_solar_iset_balanced_delta_dA(1, 20, 3, true));
    // > 3, <= 10 → -1
    EXPECT_EQ(-1, decide_solar_iset_balanced_delta_dA(1, 20, 4, true));
    EXPECT_EQ(-1, decide_solar_iset_balanced_delta_dA(1, 20, 10, true));
    // > 10, <= 20 → -5
    EXPECT_EQ(-5, decide_solar_iset_balanced_delta_dA(1, 20, 11, true));
    EXPECT_EQ(-5, decide_solar_iset_balanced_delta_dA(1, 20, 20, true));
    // > 20 → -(IsumImport / 2)
    EXPECT_EQ(-10, decide_solar_iset_balanced_delta_dA(1, 20, 21, true));
    EXPECT_EQ(-25, decide_solar_iset_balanced_delta_dA(1, 20, 50, true));
    EXPECT_EQ(-100, decide_solar_iset_balanced_delta_dA(1, 20, 200, true));

    // --- Multi-EVSE gate passes once active_evse > 0 (no scaling dependency) ---
    EXPECT_EQ(5, decide_solar_iset_balanced_delta_dA(8, 20, -20, true));
    EXPECT_EQ(-5, decide_solar_iset_balanced_delta_dA(4, 20, 11, true));
    TEST_END();
}

void test_force_single_phase_charging_all_branches() {
    TEST_BEGIN("force_single_phase_charging covers all EnableC2 × Mode × Nr_Of_Phases");
    using evse::force_single_phase_charging;
    using evse::MODE_NORMAL; using evse::MODE_SMART; using evse::MODE_SOLAR; using evse::MODE_OFF;
    using evse::C2_NOT_PRESENT; using evse::C2_ALWAYS_OFF; using evse::C2_SOLAR_OFF;
    using evse::C2_AUTO; using evse::C2_ALWAYS_ON;

    // NOT_PRESENT → always 3P (0), regardless of mode or phase count.
    EXPECT_EQ(0, force_single_phase_charging(C2_NOT_PRESENT, MODE_NORMAL, 3));
    EXPECT_EQ(0, force_single_phase_charging(C2_NOT_PRESENT, MODE_SOLAR,  1));
    EXPECT_EQ(0, force_single_phase_charging(C2_NOT_PRESENT, MODE_OFF,    1));

    // ALWAYS_OFF → always 1P (1).
    EXPECT_EQ(1, force_single_phase_charging(C2_ALWAYS_OFF, MODE_NORMAL, 3));
    EXPECT_EQ(1, force_single_phase_charging(C2_ALWAYS_OFF, MODE_SOLAR,  3));

    // SOLAR_OFF → 1P only in MODE_SOLAR.
    EXPECT_EQ(1, force_single_phase_charging(C2_SOLAR_OFF, MODE_SOLAR,  3));
    EXPECT_EQ(0, force_single_phase_charging(C2_SOLAR_OFF, MODE_NORMAL, 3));
    EXPECT_EQ(0, force_single_phase_charging(C2_SOLAR_OFF, MODE_SMART,  3));
    EXPECT_EQ(0, force_single_phase_charging(C2_SOLAR_OFF, MODE_OFF,    3));

    // AUTO → follows Nr_Of_Phases_Charging (only ==1 triggers 1P).
    EXPECT_EQ(1, force_single_phase_charging(C2_AUTO, MODE_NORMAL, 1));
    EXPECT_EQ(0, force_single_phase_charging(C2_AUTO, MODE_NORMAL, 3));
    EXPECT_EQ(0, force_single_phase_charging(C2_AUTO, MODE_NORMAL, 2));  // not 1 → 3P
    EXPECT_EQ(0, force_single_phase_charging(C2_AUTO, MODE_NORMAL, 0));  // not 1 → 3P

    // ALWAYS_ON → always 3P (0).
    EXPECT_EQ(0, force_single_phase_charging(C2_ALWAYS_ON, MODE_SOLAR, 1));

    // Out-of-range enum → default 3P (matches legacy `return 0` fallback).
    EXPECT_EQ(0, force_single_phase_charging(static_cast<evse::EnableC2Config>(99), MODE_NORMAL, 1));
    TEST_END();
}

}  // namespace

int main() {
    test_decode_pilot_bands();
    test_pilot_from_cp_mv_knobs();
    test_duty_for_current_boundaries();
    test_current_for_duty_inverse();
    test_cp_duty_v3_matches_firmware();
    test_core_plug_in_smart_charges();
    test_core_mode_off_blocks_charge();
    test_core_rcm_fault_bypasses_grace();
    test_limits_clear_on_zero();
    test_limits_apply_in_range();
    test_limits_reject_out_of_range();
    test_limits_reject_slave();
    test_limits_reject_non_normal_smart_mode();
    test_limits_max_sum_mains_boundaries();
    test_limits_max_sum_mains_slave_rejects_even_zero();
    test_limits_cp_pwm_override_release();
    test_limits_cp_pwm_override_disconnect();
    test_limits_cp_pwm_override_apply();
    test_limits_cp_pwm_override_reject_out_of_range();
    test_limits_set_mode_accept_five_keywords();
    test_limits_set_mode_reject_case_and_prefix();
    test_limits_enable_c2_digit_accepts_0_to_4();
    test_limits_enable_c2_digit_rejects_out_of_range();
    test_limits_enable_c2_string_accepts_five_names();
    test_limits_enable_c2_rejects_junk_and_case();
    test_limits_mains_meter_accept_triple();
    test_limits_mains_meter_reject_role_and_type();
    test_limits_mains_meter_reject_parse();
    test_limits_mains_meter_reject_range();
    test_limits_ev_meter_accept_all_fields();
    test_limits_ev_meter_partial_apply();
    test_limits_ev_meter_reject_role_and_parse();
    test_limits_cable_lock_accept_zero_and_one();
    test_limits_cable_lock_rejects_rest();
    test_limits_home_battery_current_accepts_signed_int();
    test_limits_home_battery_current_rejects_slave_and_garbage();
    test_limits_color_rgb_accepts_valid_triples();
    test_limits_color_rgb_rejects_out_of_range_and_parse();
    test_limits_custom_button_on_vs_off();
    test_limits_rfid_uid_6_byte();
    test_limits_rfid_uid_7_byte();
    test_limits_rfid_uid_rejects();
    test_force_single_phase_charging_all_branches();
    test_check_switching_phases_all_reasons();
    test_decide_state_c_contactor2();
    test_decide_state_power_unavailable();
    test_decide_state_on_access_change();
    test_decide_mode_switch_cp_reason();
    test_decide_solar_start_phase();
    test_decide_solar_admission();
    test_decide_circuit_admission();
    test_decide_solar_iset_balanced_delta_dA();

    std::printf("\n%d/%d passed\n", g_tests_run - g_tests_failed, g_tests_run);
    return g_tests_failed ? 1 : 0;
}
