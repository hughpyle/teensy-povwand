TEENSY3 Arduino project for LED "levitation wand"

	- Run a main interrupt timer (PIT1) at ~10kHz.
	- Wire the analog converter sampling to the PIT1 timer.
	- At about ~200Hz, refresh the LEDs.
		- Something related to the ambient audio, beats, etc.
		- Some background colors, etc.
		- Some text for persistence-of-vision display.


Electronics:
	- Electret microphone & preamp.
	- Strip of LEDs.
	- Teensy 3.0 (http://www.pjrc.com/teensy/)
	- Power from a single LIPO plus 5v booster.
	- USB Battery charger (https://www.sparkfun.com/products/10401)
