#ifndef COMBO_SHELL_SERVER_H
#define COMBO_SHELL_SERVER_H

#include <string>
#include <SystemParameters.h>
#include "Message.h"
#include "EmbodimentCogServer.h"

namespace MessagingSystem {

class ComboShellServer : public EmbodimentCogServer {

    public:
        static BaseServer* createInstance();
        ComboShellServer();
        void init(const Control::SystemParameters &params);
    
        // overrides
        bool customLoopRun();
        bool processNextMessage(Message *message);
    private:
        bool _waiting;
}; // class
} // namespace

#endif
