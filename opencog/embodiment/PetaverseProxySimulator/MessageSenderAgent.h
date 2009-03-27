/**
 * MessageSenderAgent.h
 *
 * Author: Welter Luigi
 */

#ifndef MESSAGE_SENDERAGENT_H
#define MESSAGE_SENDERAGENT_H

#include <opencog/server/Agent.h>
#include <EmbodimentCogServer.h>

namespace PetaverseProxySimulator {

using namespace opencog;

class MessageSenderAgent : public Agent {

    public:

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("PetaverseProxySimulator::MessageSenderAgent");
            return _ci;
        }

        // ***********************************************/
        // Constructors/destructors

        ~MessageSenderAgent();
        MessageSenderAgent();

        void run(CogServer *ne);

}; // class
}  // namespace

#endif
