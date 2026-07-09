# ESP32_2Servo_2Frog_2TOTI_WiFi

This is a program to create an OpenLCB/LCC node. It was developed using PlatformIO to run on an Arduino Nano ESP32. The node is designed to connect over WiFi to the LCC hub provided by JMRI.

## General functionality

1. Allows two servos to be connected.
2. Provides frog switching for both servos.
3. Provides TOTI functionality for both frogs.
4. Allows the two servos to operate as a crossover.

## Detailed functionality

1. Respond to consumed events and/or send produced events.
2. Send initial states when connecting to the LCC hub to initialise JMRI.
3. Respond to queries from JMRI.
1. Provides 2 servos.
2. Allows the two servos to be operated as a single crossover.
3. Provides switching for 2 frogs.
4. Provides a TOTI for each frog.

To be completed.

## Software components
This software uses the following components;-
- OpenLCB_Single_Thread. See https://github.com/openlcb/OpenLCB_Single_Thread
- ESP32WiFiGC. See https://github.com/JohnCallingham/ESP32WiFiGC
- LCC_Servo. See https://github.com/JohnCallingham/LCC_SERVO
- LCC_Crossover. See https://github.com/JohnCallingham/LCC_CROSSOVER
- LCC_Frog. See https://github.com/JohnCallingham/LCC_FROG
- LCC_TOTI. See https://github.com/JohnCallingham/LCC_TOTI
- LCC_CONFIGURATION. See https://github.com/JohnCallingham/LCC_CONFIGURATION
