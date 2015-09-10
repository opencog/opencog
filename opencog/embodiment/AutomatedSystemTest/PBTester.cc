/*
 * opencog/embodiment/AutomatedSystemTest/PBTester.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


/**
 * PBTester.cc
 * This program performs automated system tests on PB-side of Petaverse code.
 * It simulates the PVP Proxy by reading from a file a sequence of messages to be sent and
 * received to/from Petaverse ROUTER.
 *
 * Author: Welter Luigi
 */

#include "PBTester.h"
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

using namespace AutomatedSystemTest;
using namespace opencog::messaging;
using namespace opencog;

static const bool EXIT_ON_FAILURE = false;

opencog::BaseServer* PBTester::createInstance()
{
    return new PBTester;
}

PBTester::PBTester()
{
}

PBTester::~PBTester()
{
    if (config().get_bool("SAVE_MESSAGES_TO_FILE")) {
        delete goldStdGen;
    }
}

void PBTester::init()
{
    initialize();
}

void PBTester::init(const std::string &myId, const std::string &ip, int portNumber)
{
    setNetworkElement(new NetworkElement(myId, ip, portNumber));
    initialize();
}

void PBTester::initialize()
{
    if (config().get_bool("SAVE_MESSAGES_TO_FILE")) {
        goldStdGen = new GoldStdGen(config().get("MESSAGES_FILENAME").c_str());
    }
    failed = false;
    numberOfReceivedMessages = 0;
}

bool PBTester::processNextMessage(opencog::messaging::Message *message)
{
    //logger().info("RECEIVED MESSAGE:\n%s", message->getPlainTextRepresentation());

    if (config().get_bool("SAVE_MESSAGES_TO_FILE")) {
        goldStdGen->writeMessage(*message, false);
    }
    if (expectedMessages.empty()) {
        // TODO: implement a timeout to wait for the message be read
        logger().error("Received message when there is no expected message! Unexpected message: \n%s\n", message->getPlainTextRepresentation());
        failed = true;
        if (EXIT_ON_FAILURE) exit(-1);
    } else {
        Message* expectedMessage = expectedMessages[0];
        if (strcmp(expectedMessage->getPlainTextRepresentation(),
                   message->getPlainTextRepresentation())) {
            logger().error("Received message does not match the expected one!\nReceived:\n'%s'\nExpected:\n'%s'\n", message->getPlainTextRepresentation(), expectedMessage->getPlainTextRepresentation());
            failed = true;
            if (EXIT_ON_FAILURE) exit(-1);
        }
        expectedMessages.erase(expectedMessages.begin());
        receivedTimeMessages.erase(receivedTimeMessages.begin());

        delete expectedMessage;
    }
    logger().info("Number of expected messages: %d", expectedMessages.size());
    numberOfReceivedMessages++;
    return false;
}

void PBTester::addExpectedMessage(Message* message, unsigned long time)
{
    expectedMessages.push_back(message);
    receivedTimeMessages.push_back(time);

    logger().info("Number of expected messages: %d", expectedMessages.size());
}

bool PBTester::hasExpectedMessages()
{
    return !expectedMessages.empty();
}

void PBTester::notifyEndOfGoldStdFile()
{
    if (!expectedMessages.empty()) {
        logger().error("End of gold std file reached without receiving some expected messages.");
        failed = true;
    }
    if (!numberOfReceivedMessages) {
        logger().error("Did not received any message.");
        failed = true;
    }
#if 0
    std::string msgCmd = "UNLOAD_PET Fido";
    MessagingSystem::StringMessage msg(config().get("PROXY_ID"),
                                       config().get("SPAWNER_ID"), msgCmd);
    sendMessage(msg);
#endif
    if (failed) {
        logger().error("Automated system test failed!");
        exit(-1);
    }
    logger().info("Automated system test passed!");
    exit(0);
}

GoldStdGen* PBTester::getGoldStdGen()
{
    return goldStdGen;
}
