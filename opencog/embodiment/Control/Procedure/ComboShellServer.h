#ifndef COMBO_SHELL_SERVER_H
#define COMBO_SHELL_SERVER_H

#include <string>
#include <SystemParameters.h>
#include "Message.h"
#include "NetworkElement.h"

namespace MessagingSystem {
  struct ComboShellServer : public NetworkElement {
    ComboShellServer(const Control::SystemParameters &params);
    
    bool processNextMessage(Message *message);
    void idleTime();
  private:
    bool _waiting;
  }; // class
} // namespace

#endif
