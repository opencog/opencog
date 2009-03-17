/**
 * ImportanceDecayTask.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:30:53 BRT 2007
 */

#ifndef IMPORTANCEDECAYTASK_H
#define IMPORTANCEDECAYTASK_H

#include <NetworkElement.h>
#include <time.h>

namespace OperationalPetController {

class ImportanceDecayTask : public MessagingSystem::IdleTask {

    private:

        time_t lastTickTime;

    public:

        ImportanceDecayTask();
        virtual ~ImportanceDecayTask();

        void run(MessagingSystem::NetworkElement *opc);

}; // class
}  // namespace

#endif
