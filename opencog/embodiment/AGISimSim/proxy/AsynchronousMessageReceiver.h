#ifndef _ASYNCHRONOUS_MESSAGE_RECEIVER_H
#define _ASYNCHRONOUS_MESSAGE_RECEIVER_H

#include <string>

class AsynchronousMessageReceiver {
    public:
        virtual void receiveAsynchronousMessage(const std::string&) = 0;
        virtual ~AsynchronousMessageReceiver() {}
};

#endif // _ASYNCHRONOUS_MESSAGE_RECEIVER_H
