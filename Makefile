# Motor wrapper build.
#
#   make test            hardware-free unit tests (any OS, no NanoLib needed)
#   make check-nanolib   type-check the NanoLib glue against stub headers
#   make demo            build bin/motor_demo against the real NanoLib SDK (Linux x86_64)
#   make run-demo ARGS="100 5"
#
# Point NANOLIB_DIR at the SDK if it lives elsewhere; adjust NANOLIB_INC if the
# headers are not in $(NANOLIB_DIR)/include.

CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
NANOLIB_DIR ?= ../vendor/nanolib
NANOLIB_INC ?= $(NANOLIB_DIR)/include
NANOLIB_LIB ?= $(NANOLIB_DIR)/lib

INCLUDES := -Iinclude
BIN := bin

.PHONY: all test check-nanolib demo run-demo clean

all: test

$(BIN):
	mkdir -p $(BIN)

$(BIN)/motor_tests: tests/motor_tests.cpp tests/fake_drive.hpp src/motor.cpp include/motor/*.hpp | $(BIN)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/motor_tests.cpp src/motor.cpp -o $@

test: $(BIN)/motor_tests
	./$(BIN)/motor_tests

check-nanolib:
	$(CXX) $(CXXFLAGS) $(INCLUDES) -Itests/nanolib_stubs -fsyntax-only src/nanolib_session.cpp examples/motor_demo.cpp

$(BIN)/motor_demo: examples/motor_demo.cpp src/motor.cpp src/nanolib_session.cpp include/motor/*.hpp | $(BIN)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -I$(NANOLIB_INC) examples/motor_demo.cpp src/motor.cpp src/nanolib_session.cpp \
		-L$(NANOLIB_LIB) -lnanolib -o $@

demo: $(BIN)/motor_demo

run-demo: $(BIN)/motor_demo
	LD_LIBRARY_PATH=$(NANOLIB_LIB):$$LD_LIBRARY_PATH ./$(BIN)/motor_demo $(ARGS)

clean:
	rm -rf $(BIN)
