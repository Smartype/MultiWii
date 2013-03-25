.PHONY: all build clean upload size upload_using_programmer burn_bootloader

all: build

build: clean build_msg ./build/MultiWii.hex size

build_msg:
	@echo "Building..."

size: ./build/MultiWii.hex
	"avr-size" -A "./build/MultiWii.hex"
	"avr-size"  "./build/MultiWii.elf"

clean:
	@echo "Cleaning..."
	"rm" -rf "./build/MultiWii.hex" "./build/MultiWii.elf" "./build/MultiWii.eep" "./build/Alarms.ino.o" "./build/EEPROM.ino.o" "./build/GPS.ino.o" "./build/IMU.ino.o" "./build/LCD.ino.o" "./build/LED.ino.o" "./build/Output.ino.o" "./build/RX.ino.o" "./build/Sensors.ino.o" "./build/Serial.ino.o" "./build/MultiWii.ino.cpp.o"

./build/MultiWii.hex: ./build/MultiWii.elf ./build/MultiWii.eep
	@echo "Creating ./build/MultiWii.hex..."
	"avr-objcopy" -O ihex -R .eeprom "./build/MultiWii.elf" "./build/MultiWii.hex"

./build/MultiWii.eep: ./build/MultiWii.elf
	@echo "Creating ./build/MultiWii.eep..."
	"avr-objcopy" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 "./build/MultiWii.elf" "./build/MultiWii.eep"

./build/MultiWii.elf: ./build/core.a ./build/Alarms.ino.o ./build/EEPROM.ino.o ./build/GPS.ino.o ./build/IMU.ino.o ./build/LCD.ino.o ./build/LED.ino.o ./build/Output.ino.o ./build/RX.ino.o ./build/Sensors.ino.o ./build/Serial.ino.o ./build/MultiWii.ino.cpp.o
	@echo "Creating ./build/MultiWii.elf..."
	"avr-gcc" -Os -Wl,--gc-sections -mmcu=atmega328p -o "./build/MultiWii.elf"  "./build/Alarms.ino.o" "./build/EEPROM.ino.o" "./build/GPS.ino.o" "./build/IMU.ino.o" "./build/LCD.ino.o" "./build/LED.ino.o" "./build/Output.ino.o" "./build/RX.ino.o" "./build/Sensors.ino.o" "./build/Serial.ino.o" "./build/MultiWii.ino.cpp.o" "./build/core.a" "-L./build" -lm

./build/core.a: ./build/CDC.cpp.o ./build/HardwareSerial.cpp.o ./build/HID.cpp.o ./build/IPAddress.cpp.o ./build/main.cpp.o ./build/new.cpp.o ./build/Print.cpp.o ./build/Stream.cpp.o ./build/Tone.cpp.o ./build/USBCore.cpp.o ./build/WInterrupts.c.o ./build/wiring.c.o ./build/wiring_analog.c.o ./build/wiring_digital.c.o ./build/wiring_pulse.c.o ./build/wiring_shift.c.o ./build/WMath.cpp.o ./build/WString.cpp.o
	@echo "Creating ./build/core.a..."
	"avr-ar" rcs "./build/core.a"  "./build/CDC.cpp.o" "./build/HardwareSerial.cpp.o" "./build/HID.cpp.o" "./build/IPAddress.cpp.o" "./build/main.cpp.o" "./build/new.cpp.o" "./build/Print.cpp.o" "./build/Stream.cpp.o" "./build/Tone.cpp.o" "./build/USBCore.cpp.o" "./build/WInterrupts.c.o" "./build/wiring.c.o" "./build/wiring_analog.c.o" "./build/wiring_digital.c.o" "./build/wiring_pulse.c.o" "./build/wiring_shift.c.o" "./build/WMath.cpp.o" "./build/WString.cpp.o"

./build/Alarms.ino.o: ./Alarms.ino
	@echo "Creating ./build/Alarms.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/EEPROM.ino.o: ./EEPROM.ino
	@echo "Creating ./build/EEPROM.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/GPS.ino.o: ./GPS.ino
	@echo "Creating ./build/GPS.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/IMU.ino.o: ./IMU.ino
	@echo "Creating ./build/IMU.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/LCD.ino.o: ./LCD.ino
	@echo "Creating ./build/LCD.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/LED.ino.o: ./LED.ino
	@echo "Creating ./build/LED.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/Output.ino.o: ./Output.ino
	@echo "Creating ./build/Output.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/RX.ino.o: ./RX.ino
	@echo "Creating ./build/RX.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/Sensors.ino.o: ./Sensors.ino
	@echo "Creating ./build/Sensors.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/Serial.ino.o: ./Serial.ino
	@echo "Creating ./build/Serial.ino.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  -x c++ "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/MultiWii.ino.cpp.o: ./build/MultiWii.ino.cpp
	@echo "Creating ./build/MultiWii.ino.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/CDC.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/CDC.cpp
	@echo "Creating ./build/CDC.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/HardwareSerial.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/HardwareSerial.cpp
	@echo "Creating ./build/HardwareSerial.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/HID.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/HID.cpp
	@echo "Creating ./build/HID.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/IPAddress.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/IPAddress.cpp
	@echo "Creating ./build/IPAddress.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/main.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/main.cpp
	@echo "Creating ./build/main.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/new.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/new.cpp
	@echo "Creating ./build/new.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/Print.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/Print.cpp
	@echo "Creating ./build/Print.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/Stream.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/Stream.cpp
	@echo "Creating ./build/Stream.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/Tone.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/Tone.cpp
	@echo "Creating ./build/Tone.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/USBCore.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/USBCore.cpp
	@echo "Creating ./build/USBCore.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/WInterrupts.c.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/WInterrupts.c
	@echo "Creating ./build/WInterrupts.c.o..."
	"avr-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/wiring.c.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/wiring.c
	@echo "Creating ./build/wiring.c.o..."
	"avr-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/wiring_analog.c.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/wiring_analog.c
	@echo "Creating ./build/wiring_analog.c.o..."
	"avr-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/wiring_digital.c.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/wiring_digital.c
	@echo "Creating ./build/wiring_digital.c.o..."
	"avr-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/wiring_pulse.c.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/wiring_pulse.c
	@echo "Creating ./build/wiring_pulse.c.o..."
	"avr-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/wiring_shift.c.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/wiring_shift.c
	@echo "Creating ./build/wiring_shift.c.o..."
	"avr-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/WMath.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/WMath.cpp
	@echo "Creating ./build/WMath.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

./build/WString.cpp.o: /Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino/WString.cpp
	@echo "Creating ./build/WString.cpp.o..."
	"avr-g++" -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=103  "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/cores/arduino" "-I/Applications/Arduino.app/Contents/Resources/JAVA/hardware/arduino/variants/standard" "-I."  "$<" -o "$@"

