/**
 * InterfaceListenerTask.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Mon Oct  8 22:44:49 BRT 2007
 */

#ifndef INTERFACELISTENERTASK_H
#define INTERFACELISTENERTASK_H

#include <IdleTask.h>

namespace PetaverseProxySimulator {

class InterfaceListenerTask : public MessagingSystem::IdleTask {

    private:

    public:

        // ***********************************************/
        // Constructors/destructors

        ~InterfaceListenerTask();
        InterfaceListenerTask();

        void run(MessagingSystem::NetworkElement *ne);

}; // class
}  // namespace

#endif
