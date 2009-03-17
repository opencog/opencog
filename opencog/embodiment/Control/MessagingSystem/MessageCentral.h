/**
 * MessageCentral.h
 *
 * $Header$
 *
 * Author: Elvys Borges
 * Creation: 
 */

#ifndef MESSAGECENTRAL_H
#define MESSAGECENTRAL_H

#include <pthread.h>
#include <string>
#include "Message.h"

namespace MessagingSystem {

/**
 * Interface implemented by classes which deal with Message Queue
 */
class MessageCentral {

    protected:

        void lockQueue();
        void unlockQueue();
    

    private:

        pthread_mutex_t messageQueueLock; // lock used coordinate manipulation of the messageQueue (main thread and listener thread)
//        const Control::SystemParameters& parameters;

    public:

        virtual ~MessageCentral();
//        MessageCentral(const Control::SystemParameters &params);
        MessageCentral();

		/**
		 * Create a internal queue. if reset == true and the queue already exists,
		 * it will be cleaned.
		 * @param id representation of queue name
		 * @param reset flag to clear queue if it alreaady exists
		 */         
		virtual void createQueue(const std::string id, const bool reset = false) = 0;

        /**
         * Remove all messages from internal queue.
         * @param id representing the queue name
         */
        virtual void clearQueue(const std::string id) = 0;

		/**
		 * Remove a internal queue. 
		 * @param id representation of queue name
		 */         
		virtual void removeQueue(const std::string id) = 0;


		/**
		 * Check if the queue is empty
		 * @param id representation of queue name
		 */		
		virtual const bool isQueueEmpty(const std::string id) = 0;		

		/**
		 * Gets the size of a given queue
		 * @param id representation of queue name
		 */		
		virtual const unsigned int queueSize(const std::string id) = 0;		

		/**
		 * Check if the queue exists
		 * @param id representation of queue name
		 */		
		virtual const bool existsQueue(const std::string id) = 0;		

		/**
		 * Put a message in a queue
		 * @param id is the queue name
		 * @param message is the message
		 */
		virtual void push(const std::string id, Message *message) = 0;
		
		/**
		 * Get the oldest messagem in queue
		 * @param id is the queue name
		 * @return a message
		 */
		virtual Message* pop(const std::string id) = 0;


}; // class
}  // namespace

#endif
