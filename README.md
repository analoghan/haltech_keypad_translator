# haltech_keypad_translator
Translator to snoop Haltech Canbus Keypad and Output to other systems

This sketch is deisgned to run on an Macchina M2 and manage input from a Haltech 3x5 keypad.   Other controllers and keypads can work, but will require alteration to the relevant CAN IDs.   This sketch does NOT initialize the keypad for standalone function and assumes that a Haltech ECU (Nexus or Elite) is connected as well.

### The following functions are performed:
* Automatically determine CANbus speed on 1 or 2 busses
* Snoop CANbus 0 for Haltech keypad press and/or status light messages
* Output CANbus frames based on keypad press/status light messages
* Track state of "brake light indicactor (used for electric Tesla parking brake status) via SD card
* Control output (ground) to turn indicator on/off on Haltech dash

## Dependencies:
* M2_12VIO - https://github.com/TDoust/M2_12VIO
* due_can - https://github.com/collin80/due_can

Current design utilizes the status of the lights for Hazards and Horn functions.   This is done to remove the need to track a variable for the state of theses functions.  Keypad MUST be connected to a Haltech Nexus or Elite ECU and configured.   Keypad is configured in Haltech software to latch on "hazard" presses and momentary for "horn" presses.   These are using keys 1 and 6 respectively.

Outputs are designed to simulate a standard Grayhill for ECUMaster functions on PMU16 for horns/hazards and standard x773 CAN output to control an ECUMaster Half-Bridge driver for parking brake control.
