/**
 * SleepTask.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 12:36:43 BRT 2007
 */

#ifndef SLEEPTASKLS_H
#define SLEEPTASKLS_H

#include <NetworkElement.h>

namespace LearningServer {

class SleepTask : public MessagingSystem::IdleTask {

    private:

    public:

        ~SleepTask();
        SleepTask();

        void run(MessagingSystem::NetworkElement *opc);

}; // class
}  // namespace

#endif
