This software is meant to be built for an ATMEL Atmega328p microcontroller.

## Prerequisite
Install [arduino-cli](https://arduino.github.io/arduino-cli/0.23/installation/) and add the Arduino CLI installation path to your PATH environment variable.

## Installation
Clone this repository with the `--recursive` option in order to initialize submodules (`git clone --recursive [repository]`)

If you already cloned the repository and forgot to use the `--recursive` option, you can initialize submodules by running `git submodule update --init --recursive`

## Build
Run `make` to build the flashable hex file.

## Flash
If the Atmega328p has a regular bootloader, run the following command (replace `/dev/ttyACM0` with the USB port of a USB-to-RS232 device wired to the Atmega328p's UART pins) to flash the hex file onto the Atmega328p through USB via its bootloader:

`make flash PORT=/dev/ttyACM0`
