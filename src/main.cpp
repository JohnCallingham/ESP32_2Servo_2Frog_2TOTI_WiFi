//==============================================================
// ESP32_2Servo_2Frog_2TOTI_WiFi based on;-
// ESP32_2Frog_WiFi based on;
// ESP32_2TOTI_WiFi based on;-
// Pico_2TOTI_WiFi based on;-
// https://github.com/openlcb/OpenLCB_Single_Thread/tree/master/examples/Pico_8ServoWifiGC
//
// Modified John Callingham 2025, removed servo code and added;-
// - code to read input pins and send events when a change is detected.
// - code to respond to a JMRI query and return the current state of a TOTI.
// MOdified DPH 2024
// Copyright 2019 Alex Shepherd and David Harris
//==============================================================

// Debugging -- uncomment to activate debugging statements:
    // dP(x) prints x, 
    // dPH(x) prints x in hex, 
    // dPS(string,x) prints string and x
#include <Arduino.h>
#include <Wire.h>
#include "credentials.h"
#include "ESP32WiFiGC_V2.h"
#include "servo_lcc.h"
#include "frog.h"
#include "TOTI.h"
#include "crossover.h"
#include "tof_sensor.h"
#include "configurationOTA.h"
#include "configurationPreferences.h"

// I2C #defines.
#define I2C_SDA A4
#define I2C_SCL A5

// #define DISPLAY_MEMORY_USED
// #ifdef DISPLAY_MEMORY_USED
//   char displayMemory[100];
// #endif

#define DEBUG Serial
#define NOCAN

// Board definitions
#define MANU "J Callingham"  // The manufacturer of node
#define MODEL "ESP32_2Servo_2Frog_2TOTI_Wifi" // The model of the board
#define HWVERSION "0.1"   // Hardware version
#define SWVERSION "1.0.5"   // Software version

// To Reset the Node Number, Uncomment and edit the next line
// Need to do this at least once.  
// #define NODE_ADDRESS  5,1,1,1,0x91,0x07  // First servo node.
// #define NODE_ADDRESS  5,1,1,1,0x91,0x08  // Second servo node.
#define NODE_ADDRESS  5,1,1,1,0x91,0x09  // Third servo node.

// Set to 1 to Force Reset EEPROM to Factory Defaults 
// Need to do this at least once.  
#define RESET_TO_FACTORY_DEFAULTS 0

#define NUM_RGB_LED 3 // Red, green and blue.
#define RGB_LED_RED 0 // The index to the RGB_LEDs array.
#define RGB_LED_GREEN 1 // The index to the RGB_LEDs array.
#define RGB_LED_BLUE 2 // The index to the RGB_LEDs array.

#define NUM_SERVO 2
#define NUM_POS 3 // Servos have Thrown, Mid and Closed positions.
#define NUM_CR_POS 2 // Crossover has no Mid position.

#define SERVO_EVENT_BASE 0 // This is the first servo event.
#define NUM_EVENTS_PER_SERVO ((NUM_POS * 4) + 3) // 4 events per position + toggle event + unlock event + locked event
#define NUM_SERVO_EVENT (NUM_EVENTS_PER_SERVO * NUM_SERVO)

#define CROSSOVER_EVENT_BASE (SERVO_EVENT_BASE + NUM_SERVO_EVENT)
#define NUM_CROSSOVER_EVENT ((NUM_CR_POS * 3) + 1) // Add 1 event for the toggle eventID.

#define FROG_EVENT_BASE (CROSSOVER_EVENT_BASE + NUM_CROSSOVER_EVENT) // This is the first frog event.
#define NUM_FROG 2
#define NUM_FROG_EVENT (NUM_FROG * 4) // Each frog has connectJ, disconnectJ, connectK and disconnectK events.

#define TOTI_EVENT_BASE (FROG_EVENT_BASE + NUM_FROG_EVENT) // This is the first TOTI event.
#define NUM_TOTI 2
#define NUM_TOTI_EVENT (NUM_TOTI * 2) // Occupied event and Not Occupied event for each TOTI

#define TEST_EVENT_BASE   ((TOTI_EVENT_BASE) + (NUM_TOTI_EVENT))
#define TEST_EVENT_SERVO1 ((TEST_EVENT_BASE) + 0)
#define TEST_EVENT_SERVO2 ((TEST_EVENT_BASE) + 1)
#define TEST_EVENT_CROSSOVER ((TEST_EVENT_BASE) + 2)
#define TEST_EVENT_FROG1  ((TEST_EVENT_BASE) + 3)
#define TEST_EVENT_FROG2  ((TEST_EVENT_BASE) + 4)
#define TEST_EVENT_TOTI1  ((TEST_EVENT_BASE) + 5)
#define TEST_EVENT_TOTI2  ((TEST_EVENT_BASE) + 6)
#define TEST_EVENT_STOP   ((TEST_EVENT_BASE) + 7)
#define NUM_TEST_EVENT    8 // 7 x test on and 1 x test off.

#define NUM_EVENT (NUM_SERVO_EVENT) + (NUM_CROSSOVER_EVENT) + (NUM_FROG_EVENT) + (NUM_TOTI_EVENT) + (NUM_TEST_EVENT)

#define DESCRIPTION_LENGTH 16

/**
 * Definitions of the LED configuration property values.
 */
#define LED_CONFIG_NOT_CONFIGURED 0
#define LED_CONFIG_HUB_STATE 1
#define LED_CONFIG_SERVO_1_THROWN 2
#define LED_CONFIG_SERVO_1_CLOSED 3
#define LED_CONFIG_SERVO_2_THROWN 4
#define LED_CONFIG_SERVO_2_CLOSED 5
#define LED_CONFIG_CROSSOVER_THROWN 6
#define LED_CONFIG_CROSSOVER_CLOSED 7
#define LED_CONFIG_FROG_1_J_CONNECTED 8
#define LED_CONFIG_FROG_1_K_CONNECTED 9
#define LED_CONFIG_FROG_2_J_CONNECTED 10
#define LED_CONFIG_FROG_2_K_CONNECTED 11
#define LED_CONFIG_TOTI_1_STATE 12
#define LED_CONFIG_TOTI_2_STATE 13

// These have all been changed from 8 bit to 16 bit variables so that the following debounce delay variables, which are 16 bit, are aligned to a 16 bit boundary.
uint16_t ledRedConfiguration;
uint16_t ledGreenConfiguration;
uint16_t ledBlueConfiguration;
bool ledConfigHubConnected = false;

// Forward declarations
//void initialiseServos();
void updateServos();
uint8_t getLEDState(int switchInput);

/**
 * Configure servos.
 */
#define SERVO_1_PIN A0
#define SERVO_2_PIN A1

// Create two Servo_LCC objects and set their numbers and pins.
Servo_LCC servo0(0, SERVO_1_PIN);
Servo_LCC servo1(1, SERVO_2_PIN);

// Declare an array of pointers to the Servo objects.
Servo_LCC *servo[NUM_SERVO];

/**
 * Configure crossover.
 */
// Create the Crossover object and pass in the address of the servo objects.
Crossover crossover(&servo0, &servo1);

/**
 * Configure frogs.
 */
#define FROG_1_CONNECT_J_PIN D12 // physical pin 30.
#define FROG_1_CONNECT_K_PIN D11 // physical pin 29.
#define FROG_2_CONNECT_J_PIN D10 // physical pin 28.
#define FROG_2_CONNECT_K_PIN D9  // physical pin 27.

// Create two Frog objects and set their pins.
Frog frog0(0, FROG_1_CONNECT_J_PIN, FROG_1_CONNECT_K_PIN);
Frog frog1(1, FROG_2_CONNECT_J_PIN, FROG_2_CONNECT_K_PIN);

// Declare an array of pointers to the Frog objects.
Frog *frog[NUM_FROG];

/**
 * Configure TOTIs.
 */
#define TOTI_1_INPUT_PIN D0 // physical pin 17.
#define TOTI_1_TEST_PIN  D3 // physical pin 21.
#define TOTI_2_INPUT_PIN D1 // physical pin 16.
#define TOTI_2_TEST_PIN  D2 // physical pin 20.

// Create two TOTI objects and set their pins.
TOTI toti0(0, TOTI_1_INPUT_PIN, TOTI_1_TEST_PIN);
TOTI toti1(1, TOTI_2_INPUT_PIN, TOTI_2_TEST_PIN);

// Declare an array of pointers to the TOTI objects.
TOTI *toti[NUM_TOTI];

/**
 * Various #includes from the OpenLCB_Single_Thread library.
 */
#include "mdebugging.h"           // debugging
#include "processor.h"
#include "processCAN.h"
// #include "OpenLCBHeader.h"
#include "OpenLCBHeaderJC.h"

// CDI (Configuration Description Information) in xml, must match MemStruct
// See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.1-ConfigurationDescriptionInformation-2016-02-06.pdf
extern "C" {
    #define N(x) xN(x)     // allow the insertion of the value (x) ..
    #define xN(x) #x       // .. into the CDI string. 

const char configDefInfo[] PROGMEM =
// ===== Enter User definitions below =====
  CDIheader R"(

    <group replication=')" N(NUM_RGB_LED) R"('>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <name>LED Control</name>
      <repname>Red</repname><repname>Green</repname><repname>Blue</repname>

      <int size='2'>
        <name>LED function</name>
        <default>0</default>
        <map>
          <relation><property>0</property><value>Not configured</value></relation>
          <relation><property>1</property><value>Hub status</value></relation>
          <relation><property>2</property><value>Servo 1 Thrown</value></relation>
          <relation><property>3</property><value>Servo 1 Closed</value></relation>
          <relation><property>4</property><value>Servo 2 Thrown</value></relation>
          <relation><property>5</property><value>Servo 2 Closed</value></relation>
          <relation><property>6</property><value>Crossover Thrown</value></relation>
          <relation><property>7</property><value>Crossover Closed</value></relation>
          <relation><property>8</property><value>Frog 1 Connected to J</value></relation>
          <relation><property>9</property><value>Frog 1 Connected to K</value></relation>
          <relation><property>10</property><value>Frog 2 Connected to J</value></relation>
          <relation><property>11</property><value>Frog 2 Connected to K</value></relation>
          <relation><property>12</property><value>TOTI 1 status</value></relation>
          <relation><property>13</property><value>TOTI 2 status</value></relation>
        </map>
      </int>
    </group>

    <group>
      <hints><visibility hideable='yes' hidden='yes'></visibility></hints>
      <name>Servo Speed</name>
      <int size='2'>
        <name>Servo Delay</name>
        <description>This is the number of milli seconds between each degree of movement</description>
        <min>0</min><max>250</max>
        <hints><slider tickSpacing='50' immediate='yes' showValue='true'></slider></hints>
      </int>
    </group>

    <group replication=')" N(NUM_SERVO) R"('>
        <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
        <name>Servos</name>
        <repname>1</repname>
        <string size='16'><name>Description</name></string>
        <int size='1'>
          <name>Initial position</name>
          <default>1</default>
          <map>
            <relation><property>0</property><value>Thrown</value></relation>
            <relation><property>1</property><value>Mid</value></relation>
            <relation><property>2</property><value>Closed</value></relation>
          </map>
          <hints><radiobutton/></hints>
        </int>
        <group replication=')" N(NUM_POS) R"('>
            <name>Positions</name>
            <repname>Thrown</repname><repname>Mid</repname><repname>Closed</repname>
            <!-- <string size=')" N(DESCRIPTION_LENGTH) R"('><name>Description</name></string> -->
            <int size='1'>
              <name>Servo Position in Degrees</name>
              <min>0</min><max>180</max>
              <hints><slider tickSpacing='45' immediate='yes' showValue='true'></slider></hints>
            </int>
            <eventid>
              <name>Unconditional move event</name>
              <description>Receiving this event starts this servo moving to this position regardless of its locked state</description>
            </eventid>
            <eventid>
              <name>Conditional move event</name>
              <description>Receiving this event moves this servo to this position if it is not locked</description>
            </eventid>
            <eventid>
              <name>Reached event</name>
              <description>This event will be sent when this servo reaches this position</description>
            </eventid>
            <eventid>
              <name>Leaving event</name>
              <description>This event will be sent when this servo leaves this position</description>
            </eventid>
        </group>
        <eventid>
          <name>Toggle event</name>
          <description>Receiving this event starts this servo moving to the other end</description>
        </eventid>
        <eventid>
          <name>Unlock event</name>
          <description>Receiving this event resets the locked flag for this servo</description>
        </eventid>
        <eventid>
          <name>Locked event</name>
          <description>This event will be sent when this servo is locked and it receives a conditional move event</description>
        </eventid>
    </group>

    <group>
        <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
        <name>Crossover</name>
        <string size='16'><name>Description</name></string>
        <group replication=')" N(NUM_CR_POS) R"('>
            <name>Positions</name>
            <repname>Thrown</repname><repname>Closed</repname>
            <!-- <string size=')" N(DESCRIPTION_LENGTH) R"('><name>Description</name></string> -->
            <eventid>
              <name>Move event</name>
              <description>Receiving this event starts the crossover moving to this position</description>
            </eventid>
            <eventid>
              <name>Reached event</name>
              <description>This event will be sent when the crossover reaches this position</description>
            </eventid>
            <eventid>
              <name>Leaving event</name>
              <description>This event will be sent when the crossover leaves this position</description>
            </eventid>
        </group>
        <eventid>
          <name>Toggle event</name>
          <description>Receiving this event starts the crossover moving to the other end</description>
        </eventid>
    </group>

    <group replication=')" N(NUM_FROG) R"('>
        <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
        <name>Frogs</name>
        <repname>1</repname>
        <string size=')" N(DESCRIPTION_LENGTH) R"('><name>Description</name></string>
        <eventid>
          <name>Connect Frog to J</name>
          <description>Receiving this event causes this frog to be connected to the J wire.</description>
        </eventid>
        <eventid>
          <name>Disconnect Frog from J</name>
          <description>Receiving this event causes this frog to be disconnected from the J wire.</description>
        </eventid>
        <eventid>
          <name>Connect Frog to K</name>
          <description>Receiving this event causes this frog to be connected to the K wire.</description>
        </eventid>
        <eventid>
          <name>Disconnect Frog from K</name>
          <description>Receiving this event causes this frog to be disconnected from the K wire.</description>
        </eventid>
    </group>

    <group replication=')" N(NUM_TOTI) R"('>
        <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
        <name>TOTIs</name>
        <repname>1</repname>
        <string size=')" N(DESCRIPTION_LENGTH) R"('><name>Description</name></string>
        <int size='2'>
          <name>Occupied Delay</name>
          <description>The number of mS that this TOTI should permanently indicate occupied before the occupied event is sent.</description>
          <min>0</min><max>5000</max>
          <hints><slider tickSpacing='1000' immediate='yes' showValue='true'></slider></hints>
        </int>
        <int size='2'>
          <name>Not Occupied Delay</name>
          <description>The number of mS that this TOTI should permanently indicate not occupied before the not occupied event is sent.</description>
          <min>0</min><max>5000</max>
          <hints><slider tickSpacing='1000' immediate='yes' showValue='true'></slider></hints>
        </int>
        <eventid>
          <name>Occupied</name>
          <description>This event will be sent when the occupied delay expires.</description>
        </eventid>
        <eventid>
          <name>Not Occupied</name>
          <description>This event will be sent when the not occupied delay expires.</description>
        </eventid>
    </group>

    <group>
      <hints><visibility hideable='yes' hidden='yes' ></visibility></hints>
      <name>Testing</name>
      <eventid>
        <name>Start testing Servo 1</name>
        <description>Receiving this event starts the Servo 1 testing cycle.</description>
      </eventid>
      <eventid>
        <name>Start testing Servo 2</name>
        <description>Receiving this event starts the Servo 2 testing cycle.</description>
      </eventid>
      <eventid>
        <name>Start testing Crossover</name>
        <description>Receiving this event starts the Crossover testing cycle.</description>
      </eventid>
      <eventid>
        <name>Start testing Frog 1</name>
        <description>Receiving this event starts the Frog 1 testing cycle.</description>
      </eventid>
      <eventid>
        <name>Start testing Frog 2</name>
        <description>Receiving this event starts the Frog 2 testing cycle.</description>
      </eventid>
      <eventid>
        <name>Start testing TOTI 1</name>
        <description>Receiving this event starts the TOTI 1 testing cycle.</description>
      </eventid>
      <eventid>
        <name>Start testing TOTI 2</name>
        <description>Receiving this event starts the TOTI 2 testing cycle.</description>
      </eventid>
      <eventid>
        <name>Stop all testing</name>
        <description>Receiving this event stops all of the board testing cycles.</description>
      </eventid>
    </group>

  )" CDIfooter;
// ===== Enter User definitions above =====
} // end extern

// ===== MemStruct =====
//   Memory structure of EEPROM, must match CDI above
typedef struct { 
  EVENT_SPACE_HEADER eventSpaceHeader; // MUST BE AT THE TOP OF STRUCT - DO NOT REMOVE!!!
  
  char nodeName[20];  // optional node-name, used by ACDI
  char nodeDesc[24];  // optional node-description, used by ACDI
  // ===== Enter User definitions below =====

  struct {
    uint16_t ledConfiguration; // Changed from 8 bit to 16 bit so that all variables align to a 16 bit boundary.
  } RGB_LEDs[NUM_RGB_LED];

  uint16_t delaymS;  // Changed from 8 bit to 16 bit so that all variables align to a 16 bit boundary.

  struct {
    char servodesc[DESCRIPTION_LENGTH];        // description of this Servo Turnout Driver
    uint8_t initialPosition;
    struct {
      //char positiondesc[DESCRIPTION_LENGTH];      // description of this Servo Position
      uint8_t pos;      // angle for this position
      EventID eid;      // eventID consumed to move to this position unconditionally
      EventID cid;      // eventID consumed to move to this position conditionally (Check, Lock and Move)
      EventID rid;      // eventID produced when position reached
      EventID lid;      // eventID produced when position left
    } pos[NUM_POS];
    EventID tid;        // eventID consumed to toggle position
    EventID uid;        // eventID consumed to unlock
    EventID kid;        // eventID produced when a locked servo receives a conditional move event
  } servos[NUM_SERVO];

  struct {
    char crossoverdesc[DESCRIPTION_LENGTH];        // description of this Crossover
    struct {
      // char positiondesc[DESCRIPTION_LENGTH];      // description of this Crossover Position
      EventID eid;      // eventID consumed to move to this position
      // EventID cid;      // eventID consumed to move to this position conditionally (Check, Lock and Move)
      EventID rid;      // eventID produced when position reached
      EventID lid;      // eventID produced when position left
    } pos[NUM_CR_POS];
    EventID tid;        // eventID consumed to toggle position
  } crossover;

  struct {
    char frogdesc[DESCRIPTION_LENGTH];        // description of this Frog
    EventID eidConnectJ;
    EventID eidDisconnectJ;
    EventID eidConnectK;
    EventID eidDisconnectK;
  } Frogs[NUM_FROG];

  struct {
    char totidesc[DESCRIPTION_LENGTH];        // description of this TOTI
    uint16_t occupiedDelay;
    uint16_t notOccupiedDelay;
    EventID eidON;
    EventID eidOFF;
  } TOTIs[NUM_TOTI];

  EventID testStartServo1;
  EventID testStartServo2;
  EventID testStartCrossover;
  EventID testStartFrog1;
  EventID testStartFrog2;
  EventID testStartTOTI1;
  EventID testStartTOTI2;
  EventID testAllStop;

  // ===== Enter User definitions above =====
} MemStruct;       // type definition

extern "C" {
    // ===== eventid Table =====
    // useful macro to help fill the table

    // each servo position has 4 eventIDs
    #define REG_POS(s,p) CEID(servos[s].pos[p].eid), CEID(servos[s].pos[p].cid), PEID(servos[s].pos[p].rid), PEID(servos[s].pos[p].lid)  

    // each servo has three positions + toggle eventID + unlock eventID + locked eventID
    #define REG_SERVO(s) REG_POS(s,0), REG_POS(s,1), REG_POS(s,2), CEID(servos[s].tid), CEID(servos[s].uid), PEID(servos[s].kid)

    // each crossover position has 3 event IDs
    #define REG_POS_CR(p) CEID(crossover.pos[p].eid), PEID(crossover.pos[p].rid), PEID(crossover.pos[p].lid)  

    // the crossover has two positions and a toggle eventID
    #define REG_CROSSOVER REG_POS_CR(0), REG_POS_CR(1), CEID(crossover.tid)

    #define REG_FROG(s) CEID(Frogs[s].eidConnectJ), CEID(Frogs[s].eidDisconnectJ), CEID(Frogs[s].eidConnectK), CEID(Frogs[s].eidDisconnectK)

    #define REG_TOTI(s) PEID(TOTIs[s].eidON), PEID(TOTIs[s].eidOFF)
    
    //  Array of the offsets to every eventID in MemStruct/EEPROM/mem, and P/C flags
    const EIDTab eidtab[NUM_EVENT] PROGMEM = {
      REG_SERVO(0), REG_SERVO(1),
      REG_CROSSOVER,
      REG_FROG(0), REG_FROG(1),
      REG_TOTI(0), REG_TOTI(1),
      CEID(testStartServo1), CEID(testStartServo2),
      CEID(testStartCrossover),
      CEID(testStartFrog1), CEID(testStartFrog2),
      CEID(testStartTOTI1), CEID(testStartTOTI2),
      CEID(testAllStop)
    };
    
    // SNIP Short node description for use by the Simple Node Information Protocol
    // See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.3-SimpleNodeInformation-2016-02-06.pdf
    extern const char SNII_const_data[] PROGMEM = "\001" MANU "\000" MODEL "\000" HWVERSION "\000" OlcbCommonVersion ; // last zero in double-quote
} // end extern "C"

// PIP Protocol Identification Protocol uses a bit-field to indicate which protocols this node supports
// See 3.3.6 and 3.3.7 in http://openlcb.com/wp-content/uploads/2016/02/S-9.7.3-MessageNetwork-2016-02-06.pdf
uint8_t protocolIdentValue[6] = {   //0xD7,0x58,0x00,0,0,0};
        pSimple | pDatagram | pMemConfig | pPCEvents | !pIdent    | pTeach     | !pStream   | !pReservation, // 1st byte
        pACDI   | pSNIP     | pCDI       | !pRemote  | !pDisplay  | !pTraction | !pFunction | !pDCC        , // 2nd byte
        0, 0, 0, 0                                                                                           // remaining 4 bytes
    };

#define OLCB_NO_BLUE_GOLD

// This is called to initialize the EEPROM during Factory Reset.
void userInitAll()
{
  NODECONFIG.put(EEADDR(nodeName), ESTRING("ESP32"));
  NODECONFIG.put(EEADDR(nodeDesc), ESTRING("2Servo_2Frog_2TOTI_WiFi"));

  NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration), LED_CONFIG_NOT_CONFIGURED); // The red LED default is not configured.
  NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration), LED_CONFIG_NOT_CONFIGURED); // The green LED default is not configured.
  NODECONFIG.update16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration), LED_CONFIG_HUB_STATE); // The blue LED default is to show the hub connection status.

  // NODECONFIG.put(EEADDR(delaymS), (uint8_t) 25); // Default to 25 mS.
  // NODECONFIG.update(EEADDR(delaymS), 25); // Default to 25 mS.
  NODECONFIG.update16(EEADDR(delaymS), 25); // Default to 25 mS.

  for(uint8_t i = 0; i < NUM_SERVO; i++) {
    NODECONFIG.put(EEADDR(servos[i].servodesc), ESTRING(""));

    // Set all servos' initial position to Mid.
    // NODECONFIG.put(EEADDR(servos[i].initialPosition), (uint8_t) 1);
    NODECONFIG.update(EEADDR(servos[i].initialPosition), 1);

    // Factory reset positions.
    // NODECONFIG.put(EEADDR(servos[i].pos[0].pos), (uint8_t) 80); // Thrown
    // NODECONFIG.put(EEADDR(servos[i].pos[1].pos), (uint8_t) 90); // Mid point
    // NODECONFIG.put(EEADDR(servos[i].pos[2].pos), (uint8_t) 100); // Closed
    NODECONFIG.update(EEADDR(servos[i].pos[0].pos), 80); // Thrown
    NODECONFIG.update(EEADDR(servos[i].pos[1].pos), 90); // Mid point
    NODECONFIG.update(EEADDR(servos[i].pos[2].pos), 100); // Closed
  }

  NODECONFIG.put(EEADDR(crossover.crossoverdesc), ESTRING(""));

  // Set the default debounce delay for all TOTIs to 100 mS.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    NODECONFIG.update16(EEADDR(TOTIs[i].occupiedDelay), 100);
    NODECONFIG.update16(EEADDR(TOTIs[i].notOccupiedDelay), 100);
  }
}

/**
 * userState() is called when JMRI queries the state of an event index.
 */
enum evStates { VALID=4, INVALID=5, UNKNOWN=7 };
uint8_t userState(uint16_t index) {
  Serial.printf("\n%6ld In userState() for event index = 0x%02X", millis(), index);

  // Determine if a Servo object has this event index.
  for (uint8_t i=0; i<NUM_SERVO; i++) {
    if (servo[i]->eventIndexMatches(index)) {
      // This servo has this event index.
      return servo[i]->eventIndexMatchesCurrentState(index) ? VALID : INVALID;
    }
  }

  // Determine if the Crossover object has this event index.
  if (crossover.eventIndexMatches(index)) {
    // The crossover object has this event index.
    return crossover.eventIndexMatchesCurrentState(index) ? VALID : INVALID;
  }

  // Determine if a Frog object has this event index.
  for (uint8_t i=0; i<NUM_FROG; i++) {
    if (frog[i]->eventIndexMatches(index)) {
      // This Frog object has this event index.
      return frog[i]->eventIndexMatchesCurrentState(index) ? VALID : INVALID;
    }
  }

  // Determine if a TOTI object has this event index.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    if (toti[i]->eventIndexMatches(index)) {
      // This TOTI object has this event index.
      return toti[i]->eventIndexMatchesCurrentState(index) ? VALID : INVALID;
    }
  }

  return UNKNOWN; // In case index is not recognised.
}

// ===== Process Consumer-eventIDs =====
void pceCallback(uint16_t index) {
  // Invoked when an event is consumed; drive pins as needed
  // from index of all events.
  Serial.printf("\npceCallback() called with index=0x%02X", index);

  // Determine if this event index is for one of the Servo objects.
  for (uint8_t i=0; i<NUM_SERVO; i++) {
    if (servo[i]->eventIndexMatches(index)) {
      // This Servo object has this event index.
      Serial.printf("\nindex 0x%02X belongs to servo %d", index, i);

      // Drive the servo as required.
      Serial.printf("\ncalling eventReceived()");
      servo[i]->eventReceived(index);
    }
  }

  // Determine if this event index is for the Crossover object.
  if (crossover.eventIndexMatches(index)) {
    // The Crossover object has this event index.
    Serial.printf("\nindex 0x%02X belongs to the crossover", index);

    // Drive the crossover as required.
    crossover.eventReceived(index);
  }

  // Determine if this event index is for one of the Frog objects.
  for (uint8_t i=0; i<NUM_FROG; i++) {
    if (frog[i]->eventIndexMatches(index)) {
      // This Frog object has this event index.
      Serial.printf("\nindex 0x%02X belongs to frog %d", index, i);

      // Set the frog's pins as required.
      frog[i]->eventReceived(index);
    }
  }

  // Determine if this event index is for one of the TOTI objects.
  // Only used for the test start and test stop events.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    if (toti[i]->eventIndexMatches(index)) {
      // This TOTI object has this event index.
      Serial.printf("\nindex 0x%02X belongs to TOTI %d", index, i);

      // Only used for the test start and test stop events.
      toti[i]->eventReceived(index);
    }
  }
}

void userSoftReset() {}
void userHardReset() {}


// Callback from a Configuration write
// Use this to detect changes in the node's configuration
// This may be useful to take immediate action on a change.
void userConfigWritten(uint32_t address, uint16_t length, uint16_t func) {
  dPS("\nuserConfigWritten: Addr: ", (uint32_t)address); 
  dPS("  Len: ", (uint16_t)length); 
  dPS("  Func: ", (uint8_t)func);

  // Need to do an EEPROM commit as this doesn't always appear to happen!!
  EEPROM.commit();

  // Update the LED's configuration as it may have been changed.
  ledRedConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration));
  ledGreenConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration));
  ledBlueConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration));

  // Update the speed for all servos.
  for (int s=0; s<NUM_SERVO; s++) {
    // servo[s]->updateDelaymS((long) NODECONFIG.read(EEADDR(delaymS)));
    servo[s]->updateDelaymS((long) NODECONFIG.read16(EEADDR(delaymS)));
  }

  // Find the servo and position which has been changed.
  for(int s=0; s<NUM_SERVO; s++) {
    for(int p=0; p<NUM_POS; p++) {
      if(address == EEADDR(servos[s].pos[p].pos)) {
        // Find the new angle for this servo's position.
        uint8_t servoPosDegrees = NODECONFIG.read(EEADDR(servos[s].pos[p].pos)); 

        Serial.printf("\nsetting servo %d, position %d to angle %d", s, p, servoPosDegrees);

        updateServos();

        // Move the servo to this angle.
        servo[s]->servoEasing.moveTo(servoPosDegrees); // Uses no easing.

        break; // === Does this break out of both for loops ??? ===
      }
    }
  }

  // Update the TOTI's delay as they may have changed.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    toti[i]->setOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].occupiedDelay)));
    toti[i]->setNotOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].notOccupiedDelay)));
  }
}

void sendInitialEvents() {
  for (uint8_t i=0; i<NUM_SERVO; i++) {
    servo[i]->sendEventsForCurrentState();
  }

  crossover.sendEventsForCurrentState();

  for (uint8_t i=0; i<NUM_TOTI; i++) {
    toti[i]->sendEventsForCurrentState();
  }
}

void sendEventCallbackFunction(uint16_t eventIndexToSend) {
  if (hubConnected) {
    Serial.printf("\n%6ld sendEventCallbackFunction() called. event index=0x%02X", millis(), eventIndexToSend);
    OpenLcb.produce(eventIndexToSend);
  }
}

void reachedAngleCallbackFunction(uint8_t servoNumber, uint8_t currentAngle, ServoEasing::AngleDirection direction) {
  Serial.printf("\n%6ld reachedAngleCallbackFunction() called. servoNumber=%d, currentAngle=%d", millis(), servoNumber, currentAngle);

  // Pass this data back to the Servo_LCC object so that it can send the appropriate event.
  servo[servoNumber]->handleReachedAngle(currentAngle, direction);
}

/**
 * Called when a configuration change is made to the servo angles.
 * TO DO: only need to update the one position which has been changed.
 */
void updateServos() {
  for (uint8_t i=0; i<NUM_SERVO; i++) {
    for (uint8_t j=0; j<NUM_POS; j++) {
      servo[i]->updatePosition(j, NODECONFIG.read(EEADDR(servos[i].pos[j].pos)));
    }
  }
}

void initialiseRGBLEDs() {
  // Configure the built in RGB LEDs and turn them off.
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);

  // Determine how the RGB LEDs are configured.
  ledRedConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_RED].ledConfiguration));
  ledGreenConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_GREEN].ledConfiguration));
  ledBlueConfiguration = NODECONFIG.read16(EEADDR(RGB_LEDs[RGB_LED_BLUE].ledConfiguration));
}

void initialiseServos() {
  // Store pointers to the Servo objects in the servo array.
  servo[0] = &servo0;
  servo[1] = &servo1;

  // Initialise all Servo_LCC objects.
  for (uint8_t i=0; i<NUM_SERVO; i++) {
    // servo[i]->updateDelaymS((long) NODECONFIG.read(EEADDR(delaymS)));
    servo[i]->updateDelaymS((long) NODECONFIG.read16(EEADDR(delaymS)));

    // for (uint8_t j=0; j<NUM_POS; j++) {
    //   servo[i]->addPosition(
    //         j,
    //         "", // to be changed.
    //         NODECONFIG.read(EEADDR(servos[i].pos[j].pos)),
    //         SERVO_EVENT_BASE + (i*10) + (j*3) + 0,
    //         0,
    //         SERVO_EVENT_BASE + (i*10) + (j*3) + 2,
    //         SERVO_EVENT_BASE + (i*10) + (j*3) + 1);
    //   servo[i]->setEventToggle(SERVO_EVENT_BASE + (i*10) + 9);
    // }

    // for (uint8_t j=0; j<NUM_POS; j++) {
    //   servo[i]->addPosition(
    //         j,
    //         "", // to be changed.
    //         NODECONFIG.read(EEADDR(servos[i].pos[j].pos)),
    //         // 4 events per position
    //         // 3 positions = 12 events
    //         // Add one for the servo toggle
    //         // Total of 13 events per servo
    //         SERVO_EVENT_BASE + (i*13) + (j*4) + 0,
    //         SERVO_EVENT_BASE + (i*13) + (j*4) + 1,
    //         // SERVO_EVENT_BASE + (i*13) + (j*4) + 3,
    //         // SERVO_EVENT_BASE + (i*13) + (j*4) + 2);
    //         SERVO_EVENT_BASE + (i*13) + (j*4) + 2,
    //         SERVO_EVENT_BASE + (i*13) + (j*4) + 3);
    //   servo[i]->setEventToggle(SERVO_EVENT_BASE + (i*13) + 12);
    // }

    for (uint8_t j=0; j<NUM_POS; j++) {
      servo[i]->addPosition(
            j,
            "", // to be changed.
            NODECONFIG.read(EEADDR(servos[i].pos[j].pos)),
            // 4 events per position
            // 3 positions = 12 events
            // Add one for the servo toggle, one for unlock, one for locked.
            // Total of 15 events per servo
            SERVO_EVENT_BASE + (i*15) + (j*4) + 0,
            SERVO_EVENT_BASE + (i*15) + (j*4) + 1,
            SERVO_EVENT_BASE + (i*15) + (j*4) + 2,
            SERVO_EVENT_BASE + (i*15) + (j*4) + 3);
      servo[i]->setEventToggle(SERVO_EVENT_BASE + (i*15) + 12);
      servo[i]->setEventUnLock(SERVO_EVENT_BASE + (i*15) + 13);
      servo[i]->setEventLocked(SERVO_EVENT_BASE + (i*15) + 14);
    }

    // Set the current and mid angles to position 1 (Mid) for this servo.
    // Causes the servo to snap to this angle on startup.
    servo[i]->setInitialAngles(NODECONFIG.read(EEADDR(servos[i].pos[1].pos)));

    // Set the target angle to the initial position.
    // Causes the servo to ease to this position on startup.
    servo[i]->setInitialPosition(NODECONFIG.read(EEADDR(servos[i].initialPosition)));

    // Set the same call back function for all leaving event sending.
    servo[i]->setSendEventCallbackFunction(sendEventCallbackFunction);

    // Set the call back functions for all reached event sending.
    servo[i]->servoEasing.setReachedAngleCallbackFunction(reachedAngleCallbackFunction);

    servo[i]->setTestStopEventIndex(TEST_EVENT_STOP);

    // servo[i]->print();
  }

  // Initialise the servo testing events.
  servo[0]->setTestStartEventIndex(TEST_EVENT_SERVO1);
  servo[1]->setTestStartEventIndex(TEST_EVENT_SERVO2);
}

void initialiseCrossover() {
  // Add position 0 (Thrown).
  crossover.addPosition(
        POS_CR_THROWN, "Thrown",
        CROSSOVER_EVENT_BASE + 0,
        // CROSSOVER_EVENT_BASE + 1,
        // CROSSOVER_EVENT_BASE + 2,
        // CROSSOVER_EVENT_BASE + 3);
        CROSSOVER_EVENT_BASE + 1,
        CROSSOVER_EVENT_BASE + 2);

  // Add position 1 (Closed).
  crossover.addPosition(
        POS_CR_CLOSED, "Closed", // TO DO: Changed from POS_CLOSED. To be tested.
        // POS_CLOSED, "Closed", // TO DO: Changed from POS_CLOSED. To be tested.
        // CROSSOVER_EVENT_BASE + 4,
        // CROSSOVER_EVENT_BASE + 5,
        // CROSSOVER_EVENT_BASE + 6,
        // CROSSOVER_EVENT_BASE + 7);
        CROSSOVER_EVENT_BASE + 3,
        // CROSSOVER_EVENT_BASE + 4,
        CROSSOVER_EVENT_BASE + 4,
        CROSSOVER_EVENT_BASE + 5);
  
  // Add position 2 (Unknown).
  crossover.addPosition(
        POS_CR_UNKNOWN, "Unknown",
        0, 0, 0); // would be better if these were 0xFFFF
  
  // Set the initial position to Unknown.
  crossover.setInitialPosition(POS_CR_UNKNOWN);

  crossover.setEventToggle(CROSSOVER_EVENT_BASE + 6);

  crossover.setSendEventCallbackFunction(sendEventCallbackFunction);

  crossover.setTestStartEventIndex(TEST_EVENT_CROSSOVER);
  crossover.setTestStopEventIndex(TEST_EVENT_STOP);
}

void initialiseFrogs() {
  // Store pointers to the Frog objects in the frog array.
  frog[0] = &frog0;
  frog[1] = &frog1;

  // Initialise all Frog objects.
  for (uint8_t i=0; i<NUM_FROG; i++) {
    frog[i]->setDelaymS(100); // The time that both J and K are disconnected when changing from one to the other.
    frog[i]->setEvents(FROG_EVENT_BASE + (i*4) + 0,
                      FROG_EVENT_BASE + (i*4) + 1,
                      FROG_EVENT_BASE + (i*4) + 2,
                      FROG_EVENT_BASE + (i*4) + 3);
    frog[i]->setTestStopEventIndex(TEST_EVENT_STOP);

    // frog[i]->print();
  }

  // Initialise the frog testing events.
  frog[0]->setTestStartEventIndex(TEST_EVENT_FROG1);
  frog[1]->setTestStartEventIndex(TEST_EVENT_FROG2);
}

void initialiseTOTIs() {
  // Store pointers to the TOTI objects in the toti array.
  toti[0] = &toti0;
  toti[1] = &toti1;

  // Initialise all TOTI objects.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    toti[i]->setEvents(TOTI_EVENT_BASE + (i*2) + 0, TOTI_EVENT_BASE + (i*2) + 1);
    toti[i]->setSendEventCallbackFunction(sendEventCallbackFunction);

    toti[i]->setOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].occupiedDelay)));
    toti[i]->setNotOccupiedDebounceDelay(NODECONFIG.read16(EEADDR(TOTIs[i].notOccupiedDelay)));

    toti[i]->setTestStopEventIndex(TEST_EVENT_STOP);

    // toti[i]->print();
  }

  toti[0]->setTestStartEventIndex(TEST_EVENT_TOTI1);
  toti[1]->setTestStartEventIndex(TEST_EVENT_TOTI2);
}

void initialiseI2C() {
  bool retval;
  uint8_t address = 0x70;

  //call setPins() first, so that begin() can be called without arguments from libraries
  retval = Wire.setPins(I2C_SDA, I2C_SCL);
  if (!retval) Serial.printf("\n%6ld error from setPins()", millis());

  retval = Wire.begin();
  if (!retval) Serial.printf("\n%6ld error from begin()", millis());

  if (retval) Serial.printf("\n%6ld I2C configured successfully", millis());

  // Look for I2C device.
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();

  if (!error) {
    Serial.printf("\n%6ld I2C device found at address %02x", millis(), address);
  } else {
    Serial.printf("\n%6ld No I2C device found at address %02x", millis(), address);
  }
}

// Was "NodeID nodeid(NODE_ADDRESS);" which was moved here for OpenLCB_Single_Thread version 0.1.19.
// The actual value for Node ID is now set in setup() using data from Preferences or
// uses NODE_ADDRESS if not available in Preferences.
NodeID nodeid;

// The following #include needs nodeid to be already declared.
#include "OpenLCBMid.h"   // Essential - do not move or delete

// ==== Setup does initial configuration ======================
void setup() {
  Serial.begin(115200);

  // Delay to allow Serial port to be established.
  delay(1000);

  // temp for testing -- allows CoolTerm to be connected.
  delay(4000);

  Serial.printf("\n%6ld starting program", millis());
  Serial.printf("\n%6ld            Model: ", millis()); Serial.print(MODEL);
  Serial.printf("\n%6ld Software version: ", millis()); Serial.print(SWVERSION);
  Serial.printf("\n%6ld Compilation date: ", millis()); Serial.print(__DATE__);
  Serial.printf("\n%6ld Compilation time: ", millis()); Serial.print(__TIME__);

  // Create a ConfigurationOTA object and pass in the required parameters.
  ConfigurationOTA configurationOTA;
  configurationOTA.setCredentials(credentials); // A pointer to the credentials data in credentials.h
  configurationOTA.setTimeout(1000); // The 1000 mS timeout is used when connecting to one of potentially many WiFi hubs as not every WiFi hub may be available
  configurationOTA.setCurrentVersion(SWVERSION); // The currently running version of firmware
  configurationOTA.setDefaultNodeID(NodeID(NODE_ADDRESS)); // Used if a Node ID cannot be obtained   ==== required ???? ====

  // Connect to a WiFi network, download the json configuration file and perform all configuration.
  configurationOTA.doConfiguration();

  // Update nodeid according to the Node ID stored in Preferences.
  // If there is no Node ID stored, then use the default of NODE_ADDRESS.
  nodeid = ConfigurationPreferences::getNodeID(NodeID(NODE_ADDRESS));

  // Initialise Olcb with the node id from Preferences.
  Olcb_init(nodeid, RESET_TO_FACTORY_DEFAULTS);

  initialiseRGBLEDs();
  initialiseServos();
  initialiseCrossover();
  initialiseFrogs();
  initialiseTOTIs();
  initialiseI2C();

  Serial.printf("\n%6ld Initialisation finished", millis());
}

// ==== Loop ==========================
void loop() {
  // Do OpenLCB/LCC processing.
  Olcb_process();

  // Process any servo actions.
  for (uint8_t i=0; i<NUM_SERVO; i++) {
    servo[i]->loop();
  }

  // process any crossover actions.
  crossover.loop();

  // Process any frog timeouts.
  for (uint8_t i=0; i<NUM_FROG; i++) {
    frog[i]->loop();
  }

  // process any changes to TOTI state and send the appropriate event if required.
  for (uint8_t i=0; i<NUM_TOTI; i++) {
    toti[i]->loop();
  }

  /**
   * Connect to the OpenLCB/LCC hub and reconnect if contact has been lost.
   */
  if (hubConnectionMade(ConfigurationPreferences::getWiFiSSID(), ConfigurationPreferences::getWiFiPassword())) {
    ledConfigHubConnected = true; // Turn the blue LED on if configured.

    // This is required so that JMRI is initialised if JMRI starts after the node has started.
    sendInitialEvents();
  }

  if (hubConnectionLost(ConfigurationPreferences::getWiFiSSID(), ConfigurationPreferences::getWiFiPassword())) {
    ledConfigHubConnected = false; // Turn the blue LED off if configured.
  }

  /**
   * Control the LEDs.
   */
  digitalWrite(LED_RED, getLEDState(ledRedConfiguration));
  digitalWrite(LED_GREEN, getLEDState(ledGreenConfiguration));
  digitalWrite(LED_BLUE, getLEDState(ledBlueConfiguration));
}

/**
 * Returns LOW or HIGH to control one of the RGB LEDs.
 */
uint8_t getLEDState(int switchInput) {
  uint8_t ledState;

  switch (switchInput) {
    case LED_CONFIG_NOT_CONFIGURED:
      ledState = HIGH;
      break;
    case LED_CONFIG_HUB_STATE:
      ledState = ledConfigHubConnected ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_1_STATE:
      ledState = toti[0]->isOccupied() ? LOW : HIGH;
      break;
    case LED_CONFIG_TOTI_2_STATE:
      ledState = toti[1]->isOccupied() ? LOW : HIGH;
      break;
    case LED_CONFIG_FROG_1_J_CONNECTED:
      ledState = frog[0]->isConnectedJ() ? LOW : HIGH;
      break;
    case LED_CONFIG_FROG_1_K_CONNECTED:
      ledState = frog[0]->isConnectedK() ? LOW : HIGH;
      break;
    case LED_CONFIG_FROG_2_J_CONNECTED:
      ledState = frog[1]->isConnectedJ() ? LOW : HIGH;
      break;
    case LED_CONFIG_FROG_2_K_CONNECTED:
      ledState = frog[1]->isConnectedK() ? LOW : HIGH;
      break;
    case LED_CONFIG_SERVO_1_THROWN:
      ledState = servo[0]->isThrown() ? LOW : HIGH;
      break;
    case LED_CONFIG_SERVO_1_CLOSED:
      ledState = servo[0]->isClosed() ? LOW : HIGH;
      break;
    case LED_CONFIG_SERVO_2_THROWN:
      ledState = servo[1]->isThrown() ? LOW : HIGH;
      break;
    case LED_CONFIG_SERVO_2_CLOSED:
      ledState = servo[1]->isClosed() ? LOW : HIGH;
      break;
    case LED_CONFIG_CROSSOVER_THROWN:
      ledState = crossover.isThrown() ? LOW : HIGH;
      break;
    case LED_CONFIG_CROSSOVER_CLOSED:
      ledState = crossover.isClosed() ? LOW : HIGH;
      break;
  }
  
  return ledState;
}

// void specialDatagramReceived() {
//   Serial.printf("\nSpecial datagram received");
// }

char displayMemory[100];

void writeDisplayMemory(uint32_t address, uint8_t val) {
  Serial.printf("\nwriteDisplayMemory(). address=%d, val=%d", address, val);
  // Need to check that we don't write off the end!!
  displayMemory[address] = val;
}
