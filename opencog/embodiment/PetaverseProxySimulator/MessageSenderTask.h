/**
 * MessageSenderTask.h
 *
 * Author: Welter Luigi
 */

#ifndef MESSAGE_SENDERTASK_H
#define MESSAGE_SENDERTASK_H

#include <IdleTask.h>
#include <NetworkElement.h>

namespace PetaverseProxySimulator {

class MessageSenderTask : public MessagingSystem::IdleTask {

    public:

        // ***********************************************/
        // Constructors/destructors

        ~MessageSenderTask();
        MessageSenderTask();

        void run(MessagingSystem::NetworkElement *ne);

}; // class
}  // namespace

#endif
