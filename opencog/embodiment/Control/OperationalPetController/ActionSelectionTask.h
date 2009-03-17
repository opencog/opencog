/**
 * ActionSelectionTask.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:46:02 BRT 2007
 */

#ifndef ACTIONSELECTIONTASK_H
#define ACTIONSELECTIONTASK_H

#include <NetworkElement.h>
#include <time.h>

namespace OperationalPetController {

class ActionSelectionTask : public MessagingSystem::IdleTask {

    private:

        time_t lastTickTime;

    public:

        ~ActionSelectionTask();
        ActionSelectionTask();

        void run(MessagingSystem::NetworkElement *opc);

}; // class
}  // namespace

#endif
