.PHONY: build flash clean

SKETCH_FOLDER = dronie
FQBN := arduino:avr:uno
# ARDUINO_CLI_OPTIONS += -v  # Enable arduino-cli verbose mode
PORT ?= /dev/ttyACM0

CWD := $(shell pwd)
build:
	arduino-cli compile $(ARDUINO_CLI_OPTIONS) -b $(FQBN) --warnings all --libraries "$(CWD)/src/libraries" "$(CWD)/src/$(SKETCH_FOLDER)"

flash: build
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) "$(CWD)/src/$(SKETCH_FOLDER)"
