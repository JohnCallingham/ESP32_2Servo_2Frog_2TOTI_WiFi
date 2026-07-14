# ESP32_2Servo_2Frog_2TOTI_WiFi

This is a program to create an OpenLCB/LCC node. It was developed using PlatformIO to run on an Arduino Nano ESP32. The node is designed to connect over WiFi to the LCC hub provided by JMRI.

## General functionality

1. Allows two servos to be connected.
2. Provides frog switching for both servos.
3. Provides TOTI (Train On Track Indication) functionality for both frogs.
4. Allows the two servos to operate as a crossover.
5. Allows the onboard RGB LED to be configured to indicate various events.
6. Allows for testing cycles to be started by the user.
7. Allows for remote configuration and remote software updates.

## Detailed functionality

1. Respond to consumed events and/or send produced events.
2. Send initial states when connecting to the LCC hub to initialise JMRI.
3. Respond to queries from JMRI.
4. Provides 2 servos.
5. Allows the two servos to be operated as a single crossover.
6. Provides switching for 2 frogs.
7. Provides a TOTI for each frog.
8. Allows each colour of the on board RGB LED to be user configured to indicate one of;-
    - the status of the connection to the JMRI hub
    - the state of each of the servos (Thrown or Closed)
    - the state of the crossover (Thrown or Closed)
    - the state of each frog (connected to J or K)
    - the state of each TOTI (occupied or not occupied)
9. Allows a user configurable speed which applies to both servos. This is specified as the number of milli seconds delay between each degree of servo movement.
10. Allows for three positions for each servo (Thrown, Mid and Closed).
10. Allows the user to configure one of the three positions to be the initial position.
11. Allows for a user to configure each servo to be locked.
12. For each of the three positions for each servo;-
    - allows a user configurable angle for that position
    - responds to an event which will move the servo to that position regardless of the locked state of the servo (unconditional move)
    - responds to an event which will move the servo to that position only if the servo is not locked (conditional move)
    - sends an event when that position has been reached
    - sends an event when that position has been left
13. Each servo responds to an event which will move the servo to the other end position
14. Each servo responds to an event which will unlock the servo.
15. An event is sent if the servo is locked and a conditional move event is received.
16. Allows for two positions for the crossover (Thrown and Closed).
17. For each of the two positions for the crossover;-
    - responds to an event which will move both servos to that position
    - sends an event when both servos have reached that position
    - sends an event when the first servo leaves that position
18. The crossover responds to an event which will move both servos to the other end position.
19. Each frog responds to the follwowing events;-
    - connect frog to the J wire
    - disconnect frog from the J wire
    - connect frog to the K wire
    - disconnect frog from the K wire
20. The software ensures that the frog can never be connected to both J and K wires at the same time.
21. Send produced events when each TOTI becomes occupied or not occupied.
22. Allows user configurable delays for occupied and not occupied.
23. The following test cycles can be started by the user sending a specific event to the node;-
    - servo 1 continously moves between Thrown and Closed
    - servo 2 continously moves between Thrown and Closed
    - crossover continously moves both servos between Thrown and Closed
    - frog 1 continously connects frog 1 to J, then disconnects from J, connects to K, then disconnects from K
    - frog 2 continously connects frog 2 to J, then disconnects from J, connects to K, then disconnects from K
    - continously toggles the TOTI 1 test pin (D3) high, then low. If the test pin is connected to the TOTI 1 input pin (D0) then occupied and not occupied events will be continously sent.
    - continously toggles the TOTI 2 test pin (D2) high, then low. If the test pin is connected to the TOTI 2 input pin (D1) then occupied and not occupied events will be continously sent.
24. All the test cycles can be stopped by the user sending a specific event to the node.





## Software components
This program uses the following software components;-
- OpenLCB_Single_Thread. See https://github.com/openlcb/OpenLCB_Single_Thread
- ESP32WiFiGC. See https://github.com/JohnCallingham/ESP32WiFiGC
- LCC_Servo. See https://github.com/JohnCallingham/LCC_SERVO
- LCC_Crossover. See https://github.com/JohnCallingham/LCC_CROSSOVER
- LCC_Frog. See https://github.com/JohnCallingham/LCC_FROG
- LCC_TOTI. See https://github.com/JohnCallingham/LCC_TOTI
- LCC_CONFIGURATION. See https://github.com/JohnCallingham/LCC_CONFIGURATION

The following software components are dependencies of one or more of the above components;-
 - ArduinoJson. See [bblanchon/ArduinoJson@^7.4.1](https://github.com/bblanchon/ArduinoJson)
 - DEBOUNCE. See https://github.com/JohnCallingham/DEBOUNCE
 - HW_MUTEX. See https://github.com/JohnCallingham/HW_MUTEX
 - LCC_NODE_COMPONENT_BASE. See https://github.com/JohnCallingham/LCC_NODE_COMPONENT_BASE
 - SERVO_EASING. See https://github.com/JohnCallingham/SERVO_EASING

 The PlatformIO Library Dependency Finder handles downloading all dependencies.
