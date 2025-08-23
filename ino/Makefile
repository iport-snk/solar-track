SKETCH=/home/solar/solar-motors/solar-motors.ino
FQBN=arduino:megaavr:nona4809
PORT=/dev/ttyACM0

all: compile upload

compile:
	arduino-cli compile --fqbn $(FQBN) $(SKETCH)

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) $(SKETCH) --verbose
