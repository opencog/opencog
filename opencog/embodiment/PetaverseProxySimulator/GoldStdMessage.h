#ifndef _GOLD_STD_MESSAGE_H_
#define _GOLD_STD_MESSAGE_H_

#include "Message.h"

using namespace MessagingSystem;

namespace PetaverseProxySimulator {

class GoldStdMessage {

    unsigned long timestamp;
    Message* message;

    public: 
        GoldStdMessage(unsigned long _timestamp, Message* _message);
        ~GoldStdMessage();
        unsigned long getTimestamp();
        Message* getMessage();
};

}
    

#endif //_GOLD_STD_MESSAGE_H_
