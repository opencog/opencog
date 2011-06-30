/*
 * opencog/embodiment/Control/MessagingSystem/MessageTesterExecutable.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <stdio.h>
#include <stdlib.h>
#ifndef WIN32
#include <unistd.h>
#endif

#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include "NetworkElement.h"
#include "StringMessage.h"

#ifdef WIN32
unsigned sleep(unsigned seconds)
{
    Sleep(seconds * 1000);
    return 0;
}
#endif

using namespace opencog::messaging;

class MyElement : public NetworkElement
{

private:

    std::string myID;
    std::string peerID;
    int sleepBeforeSending;

public:

    MyElement(std::string myID, std::string peerID, int port, int sleep) : NetworkElement(myID, std::string("127.0.0.1"), port) {
        this->myID = myID;
        this->peerID = peerID;
        this->sleepBeforeSending = sleep;
    }

    void run() {
        StringMessage message(myID, peerID, "Message 1 sent by " + myID);
        printf("Sleeping for %d seconds...\n", sleepBeforeSending);
        sleep(sleepBeforeSending);
        printf("Sending message 1 to %s...\n", peerID.c_str());
        sendMessage(message);
        printf("Sent\n");
        printf("Waiting for message...\n");
        while (!haveUnreadMessage()) {
        }
        printf("Router notified message arrival. Requesting it...\n");
        retrieveMessages(-1);
        /*printf("Messages retrieved:\n");
        for (int i = 0; i < messages->size(); i++) {
            printf("%s\n", messages->at(i)->getPlainTextRepresentation());
        }*/
    }
};

int main(int argc, char *argv[])
{

    config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);

    printf("Handshaking with router...\n");
    MyElement *myElement = new MyElement(argv[1], argv[2], atoi(argv[3]), atoi(argv[4]));
    printf("OK. Sleeping for 10 seconds...\n");
    sleep(10);
    printf("Starting test\n");
    myElement->run();

    return 0;
}
