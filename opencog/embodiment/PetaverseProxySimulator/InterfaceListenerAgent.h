/**
 * InterfaceListenerAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Mon Oct  8 22:44:49 BRT 2007
 */

#ifndef INTERFACELISTENERAGENT_H
#define INTERFACELISTENERAGENT_H

#include <opencog/server/Agent.h>

namespace PetaverseProxySimulator {

using namespace opencog;

class InterfaceListenerAgent : public Agent {

    private:

    public:

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("PetaverseProxySimulator::MessageSenderAgent");
            return _ci;
        }

        // ***********************************************/
        // Constructors/destructors

        ~InterfaceListenerAgent();
        InterfaceListenerAgent();

        void run(CogServer *ne);

}; // class
}  // namespace

#endif
