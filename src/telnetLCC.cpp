#include "telnetLCC.h"

namespace TelnetLCC {
  ESPTelnet telnet; // Create an instance of the ESPTelnet class
  NodeID nodeid;
  String model = "";
  String swVersion = "";

  void initialiseTelnet() {
    telnet.begin();
    telnet.onConnect(onTelnetConnect);
    telnet.onDisconnect(onTelnetDisconnect);
  }

  void setNodeID(NodeID id) {
    nodeid = id;
  }

  void setModel(String m) {
    model = m;
  }

  void setSWVersion(String v) {
    swVersion = v;
  }

  void onTelnetConnect(String ip) {

    Serial.printf("\n%6ld [onTelnetConnect] Telnet connection from %s", millis(), ip.c_str());
    
    telnet.println("\nWelcome " + telnet.getIP());

    telnet.println("\nThis is an LCC node with the following configuration;-");

    telnet.println("\n            Model: " + model);
    char charNodeID[30] = "";
    sprintf(charNodeID, "%02X.%02X.%02X.%02X.%02X.%02X", nodeid.val[0], nodeid.val[1], nodeid.val[2], nodeid.val[3], nodeid.val[4], nodeid.val[5]);

    telnet.println("          Node ID: " + String(charNodeID));
    telnet.println(" Software version: " + swVersion);
    telnet.println(" Compilation date: " + String(__DATE__));
    telnet.println(" Compilation time: " + String(__TIME__));

    telnet.println("\n(Use CTRL+] + q  to disconnect.)");

  }

  void onTelnetDisconnect(String ip) {
    Serial.printf("\n%6ld [onTelnetDisconnect] Telnet connection from %s closed", millis(), ip.c_str());
  }

  void loop() {
    telnet.loop();
  }

  void logMessageCallbackFunction(const char* format, ...) {
    char logMessageBuffer[200];

    // Format the log message using the provided format and arguments.
    va_list argptr;
    va_start(argptr, format);
    vsnprintf(logMessageBuffer, sizeof(logMessageBuffer), format, argptr);
    va_end(argptr);

    // Telnet needs a carriage return before the line feed to display correctly.
    telnet.print("\r");
    telnet.print(logMessageBuffer);

    // Serial monitor doesn't need the carriage return.
    Serial.print(logMessageBuffer);
  }

}
