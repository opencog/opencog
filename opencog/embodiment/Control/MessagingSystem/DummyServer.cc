/**
 * DummyServer.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Sun Jun 24 16:34:54 BRT 2007
 */
#include <stdio.h>
#include "DummyServer.h"
#include "StringMessage.h"
unsigned sleep(unsigned useconds);
//#ifdef WIN32
//unsigned sleep(unsigned useconds)
//	{
//		Sleep(useconds * 1000);
//		return 0;
//	}
//#endif

using namespace MessagingSystem;

DummyServer::~DummyServer() {
}

DummyServer::DummyServer(const Control::SystemParameters &params, const std::string &id, const std::string &ip, int port) : NetworkElement(params, id, ip, port) {
    parameters = params;
    cycleCount = 0;
    petCount = 0;
}

bool DummyServer::processNextMessage(Message *message) {

    std::string cmdLine = message->getPlainTextRepresentation();
    std::string command;
    std::queue<std::string> args;

    parseCommandLine(cmdLine, command, args);

    if (command == "SHUTDOWN") {
        printf("Received command <%s>. Exiting...\n", command.c_str());
        exit(0);
    } else if (command == "SLEEP") {
        printf("Received command <%s>.\n", command.c_str());
        int n = atoi(args.front().c_str());
        args.pop();
        printf("Sleeping for %d seconds\n", n);
        sleep(n);
    } else {
        printf("Received command <%s>. Discarding it.\n", command.c_str());
    }
    return false;
}

void DummyServer::idleTime() {

    if (haveUnreadMessage()) {
        // Retrieve at most 1 new Message per cycle
        retrieveMessages(1);
    } else {
        if ((cycleCount % 10) == 0) {
            /*
            printf("Spawning %d\n", petCount);
            char buf[256];
            sprintf(buf, "LOAD_PET %d", petCount);
            StringMessage message(getID(), "SPAWNER", buf);
            sendMessage(message);
            petCount++;
            */
        } else {
            sleep(1);
        }
    }

    cycleCount++;
}
