# Teensy3 sound-reactive LED Levitation Wand

This is an Arduino project using Teensy 3.0 hardware (ARM Cortex M4).  


## Software

- Run a main interrupt timer (PIT1) at ~10kHz.
- Wire the analog converter sampling to the PIT1 timer.
- At about ~200Hz, refresh the LEDs.

	- Something related to the ambient audio, beats, etc.
	- Some background colors, etc.
	- Some text for persistence-of-vision display.


## Electronics

- Teensy 3.0  
The incredibly small and amazingly powerful (ARM Cortex M4 at 96 MHz) [Teensy 3.0 from PJRC](http://www.pjrc.com/teensy/).

- Electret microphone & preamp.  
I used a [Transound TSB140](http://www.jlielectronics.com/transsound/electrets/tsb-140a.htm) microphone, which has an internal FET and three terminals.
See the [schematic](/hughpyle/teensy-povwand/blob/master/schematic.png) for details of the mic preamp.

- Strip of addressable RGB LEDs.  
I used a [LPD8806 strip from Adafruit](https://www.adafruit.com/products/306), with its waterproof (silicone) sheath removed.  
This is driven using the Teensy's hardware SPI interface, programmed with the very nice [FastSPI library](https://code.google.com/p/fastspi/).


Electric power for this project comes from a single-cell Lithium Polymer battery, boosted to 5V.  All the electronics runs from the 5V line.  
Teensy has its own onboard regulator.
My LED strip was split into two pieces, powered independently -- each fed via a 1 ohm resistor and having a 100u reservoir capacitor.  This gives a reasonable amount of isolation when the lights are driven hard.
The microphone preamp uses a series diode and a 100u reservoir capacitor, to prevent it being affected by dropouts when the LEDs suddenly suck all the power.

- Power from a single-cell LIPO plus 5v booster.  
I found [these 1S LIPO cells](http://www.hobbyking.com/mobile/viewproduct.asp?idproduct=23318&type=&idparentcat=238) at HobbyKing, which are fairly powerful and quite narrow -- they're only 12mm diameter.  Let me know if you find smaller (narrower) cells of similar capacity!  
The 5V booster is also from HobbyKing.  [This](http://www.hobbyking.com/hobbyking/store/__11784__TURNIGY_Voltage_Booster_for_Servo_Rx_1S_to_5v_1A_.html) is the tiniest one I could find, and it's small enough, and will supply up to 1.5 amps (not quite enough for a meter of LED at continuous full power, but we won't be at continuous full power).

- LIPO cell charger.  
I used [this one](http://www.sparkfun.com/products/10401) from Sparkfun, which is small enough to fit into the tube alongside the Teensy.  
Alternatively, you could surface a two-pin socket connected to the LIPO cell, and use an external charger.  But internal charger is safer & also easier to use.


## TODO

Fix the FFT; implement windowing

Monitor the LIPO cell voltage directly using one of the Teensy ADCs.  Warning flash then power down on undervoltage.

Build a longer-tube version suitable for rotation games.

Use nunchuck or other accelerometer.  Detect rotation.  Trigger rotation-mode POV patterns.

