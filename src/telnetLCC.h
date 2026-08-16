#ifndef TELNET_LCC_H
#define TELNET_LCC_H

#include <ESPTelnet.h>
#include "NodeID.h"

namespace TelnetLCC {
  void initialiseTelnet();
  void setNodeID(NodeID id);
  void setModel(String m);
  void setSWVersion(String v);
  void onTelnetConnect(String ip);
  void onTelnetDisconnect(String ip);
  void loop();
  void logMessageCallbackFunction(const char* format, ...);
}

#endif
