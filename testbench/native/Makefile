# Host-native build for the SmartEVSE firmware test harness.
#
# Produces build/smartevse-sim which speaks the stdio JSON protocol
# documented in testbench/README.md. Used by the dashboard as a drop-in
# replacement for mock_device.py, and by the scenario runner in CI.
#
# PlatformIO users can also `pio run -e native` — see platformio.ini.

CXX       ?= c++
CXXFLAGS  ?= -std=c++17 -Wall -Wextra -Wpedantic -O2

# firmware_core now lives under the firmware tree so the firmware's
# platformio build (src glob) and the native harness share a single
# source file. The relative path is stable as long as both repos stay
# siblings under SmartEVSE-3/.
FW_SRC := ../../SmartEVSE-3/src

CPPFLAGS  += -I. -I$(FW_SRC)
BUILD     := build
BIN       := $(BUILD)/smartevse-sim
TEST_BIN  := $(BUILD)/test_firmware_core

SRCS := \
    $(FW_SRC)/firmware_core.cpp \
    $(FW_SRC)/evse_limits.cpp \
    harness/DeviceUnderTest.cpp \
    src/main.cpp

TEST_SRCS := \
    $(FW_SRC)/firmware_core.cpp \
    $(FW_SRC)/evse_limits.cpp \
    test/test_firmware_core.cpp

# Strip leading ../ from object paths so everything lands under build/
# without escaping the build dir.
OBJS      := $(patsubst %.cpp,$(BUILD)/%.o,$(patsubst ../../%,__firmware__/%,$(SRCS)))
TEST_OBJS := $(patsubst %.cpp,$(BUILD)/%.o,$(patsubst ../../%,__firmware__/%,$(TEST_SRCS)))

all: $(BIN)

$(BIN): $(OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(TEST_BIN): $(TEST_OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Objects for sources inside the testbench tree.
$(BUILD)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

# Objects for sources pulled from the firmware tree (../../SmartEVSE-3/...).
$(BUILD)/__firmware__/%.o: ../../%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD)

# Run the scenario corpus against the built binary and diff against
# committed goldens. Requires `uv` (https://docs.astral.sh/uv/).
check: test $(BIN)
	cd .. && uv run --with pyyaml python3 run_scenarios.py --binary native/$(BIN)

# Fine-grained unit tests on firmware_core helpers. Complements the
# scenario corpus; runs as part of `make check`.
test: $(TEST_BIN)
	./$(TEST_BIN)

# Regenerate the goldens. Review the resulting diff carefully — this
# updates the contract every refactor has to honor.
update-goldens: $(BIN)
	cd .. && uv run --with pyyaml python3 run_scenarios.py --binary native/$(BIN) --update-goldens

.PHONY: all clean check test update-goldens
