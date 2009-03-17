/** 
 * MemoryMessageCentral.h
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#ifndef MEMORYMESSAGECENTRAL_H_
#define MEMORYMESSAGECENTRAL_H_

#include "MessageCentral.h"

#include <map>
#include <queue>

namespace MessagingSystem{

/**
 * Implements MessageCentral using a map of queue, in memory.
 *
 */
class MemoryMessageCentral : public MessageCentral {
    
    private:
    
        std::map<std::string, std::queue<Message *> *> messageQueue;
        
    public:
    
        /**
         * Constructors and destructor
         */
        ~MemoryMessageCentral();
        MemoryMessageCentral();
        
        void createQueue(const std::string id, const bool reset = false);
    
        void clearQueue(const std::string id);
        
        void removeQueue(const std::string id);

        const unsigned int queueSize(const std::string id);        

        const bool isQueueEmpty(const std::string id);        

        const bool existsQueue(const std::string id);        

        void push(const std::string id, Message *message);
        
        Message* pop(const std::string id);
        
}; // class
}  // namespace

#endif 
