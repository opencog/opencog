/** 
 * MemoryMessageCentral.cc
 * 
 * Author: Elvys Borges
 * Copyright(c), 2007   
 */
#include "MemoryMessageCentral.h"

using namespace MessagingSystem;


MemoryMessageCentral::~MemoryMessageCentral(){
    //TODO: need I clear the queue ??
    std::map<std::string, std::queue<Message *> *>::iterator map_itr = messageQueue.begin();
    while (map_itr != messageQueue.end()) {
         std::string str_id = map_itr->first;
         while (!this->isQueueEmpty(str_id)) {
             Message *msg = this->pop(str_id);
             delete msg;    
         }
         delete map_itr->second;
         map_itr++;
    }

}

MemoryMessageCentral::MemoryMessageCentral() : MessageCentral() {

       
}

void MemoryMessageCentral::createQueue(const std::string id, const bool reset){

    if (reset == false) {
        //test if id already exists in map
        if (!this->existsQueue(id)) {
               std::queue<Message *> *newQueue = new std::queue<Message *>;
            this->lockQueue();
            messageQueue[id] = newQueue;
            this->unlockQueue();
           }
    } else {
        //I need delete all objects in the queue that already exists
           std::queue<Message *> *newQueue = new std::queue<Message *>;
        this->lockQueue();
        messageQueue[id] = newQueue;
        this->unlockQueue();
    }
    
}

void MemoryMessageCentral::clearQueue(const std::string id){
    
    if (this->existsQueue(id)) {
        
        // remove all messages
        while(!this->isQueueEmpty(id)){
            Message * message = this->pop(id);
            delete message;
        }
    }
}


void MemoryMessageCentral::removeQueue(const std::string id){
    
    if (this->existsQueue(id)) {
       
        clearQueue(id);
        
        // remove the queue itself
        this->lockQueue();
        std::queue<Message *> *mQueue = messageQueue[id];
        messageQueue.erase(id);
        delete mQueue;
        this->unlockQueue();
    }
}

const unsigned int MemoryMessageCentral::queueSize(const std::string id){

    unsigned int value = 0;

    if (this->existsQueue(id)) {
        this->lockQueue();
           std::queue<Message *> *queue = messageQueue[id];
           value = queue->size();
        this->unlockQueue();
    }
    
    return value;
}

const bool MemoryMessageCentral::isQueueEmpty(const std::string id){

    bool value = true;

    if (this->existsQueue(id)) {
           this->lockQueue();
           std::queue<Message *> *queue = messageQueue[id];
           value = queue->empty();
        this->unlockQueue();
    }
    
    return value;
}

const bool MemoryMessageCentral::existsQueue(const std::string id){

    bool value = true;
    
    this->lockQueue();
    std::map<std::string, std::queue<Message *> *>::iterator itr = messageQueue.find(id);
    
    if (itr == messageQueue.end()) {
        value = false;
    }
    this->unlockQueue();
    
    return value;
}

void MemoryMessageCentral::push(const std::string id, Message *message){
    
    if (!this->existsQueue(id)) {
         return;
    }
    
    this->lockQueue();
       std::queue<Message *> *queue = messageQueue[id];
       queue->push(message);
    this->unlockQueue();
        
}
  
Message* MemoryMessageCentral::pop(const std::string id){

    Message *value = NULL;

    if ((!this->existsQueue(id))) {
         return value;
    }
    
    this->lockQueue();
       std::queue<Message *> *queue = messageQueue[id];
       if (!queue->empty()) {
           value = queue->front();
           queue->pop();
       }
    this->unlockQueue();
    
    return value;

}


