/*
 * opencog/embodiment/AutomatedSystemTest/GoldStdReaderAgent.cc
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


#include "GoldStdReaderAgent.h"
#include "PBTester.h"
#include <unistd.h>

using namespace AutomatedSystemTest;
using namespace opencog::messaging;
using namespace opencog;

GoldStdReaderAgent::~GoldStdReaderAgent()
{
    if (goldStdFile) fclose(goldStdFile);
}

GoldStdReaderAgent::GoldStdReaderAgent(CogServer& cs) : Agent(cs)
{
}

void GoldStdReaderAgent::init(const char* goldStdFilename)
{
    goldStdFile = fopen(goldStdFilename, "r");
    if (!goldStdFile) {
        logger().error("Could not open gold standard file: %s", goldStdFilename);
        exit(-1);
    }
    messageToSend = NULL;
    initialTime = GoldStdGen::getCurrentTimestamp();
    endOfFile = false;
}

void GoldStdReaderAgent::run()
{

    PBTester* pbTester = dynamic_cast<PBTester*>(&_cogserver);
    //if (!pbTester->hasExpectedMessages()) { // THIS WAY, IT WAITS NEXT CYCLE TO RUN THE NEXT CONSECUTIVE MESSAGE
    if (!pbTester->hasExpectedMessages() && endOfFile) {
        pbTester->notifyEndOfGoldStdFile();
        return;
    }

    if (pbTester->hasExpectedMessages()) {
        unsigned long elapsedTime = GoldStdGen::getCurrentTimestamp() - pbTester->getReceivedTimeOfCurrentExpectedMessage();
        if (elapsedTime > timeout) {
            logger().error("Timeout. elapsed time: %lu.", elapsedTime);
            exit(-1);
        }
    }

    while (!pbTester->hasExpectedMessages() && !endOfFile) { // THIS WAY, IT SENDS ALL CONSECUTIVE MESSAGES AT ONCE
        logger().info("No expected messages");
        if (messageToSend) {
            unsigned long elapsedTime = GoldStdGen::getCurrentTimestamp() - initialTime;
            if (elapsedTime >= messageToSend->getTimestamp()) {
                logger().info("Send last read message...");
                if (!pbTester->sendMessage(*(messageToSend->getMessage()))) {
                    logger().error("Error sending message to router! Aborting test...");
                    exit(-1);
                }
                if (opencog::config().get_bool("SAVE_MESSAGES_TO_FILE")) {
                    pbTester->getGoldStdGen()->writeMessage(*(messageToSend->getMessage()), true);
                }
                delete messageToSend;
                messageToSend = NULL;
            } else {
#ifdef DATETIME_DECIMAL_RESOLUTION
                unsigned long usec = (messageToSend->getTimestamp() - elapsedTime) * 100000;
#else
                unsigned long usec = (messageToSend->getTimestamp() - elapsedTime) * 10000;
#endif
                logger().info("Sleeping for %lu micro seconds before sending next message...", usec);
                usleep(usec);
            }
        }
        while (!messageToSend && !endOfFile) {
            logger().info("Resume reading of gold std file...");
            if (fgets(line_buf, LINE_BUF_SIZE, goldStdFile) == NULL) {
                endOfFile = true;
                break;
                //pbTester->notifyEndOfGoldStdFile();
                //return;
            }
            if (!strcmp(RECEIVED_MESSAGE_FLAG, line_buf)) {
                logger().info("Reading expected message...");
                GoldStdMessage* msg = GoldStdGen::readMessage(line_buf, LINE_BUF_SIZE, goldStdFile);
                if (!msg) exit(-1);

                pbTester->addExpectedMessage(msg->getMessage(), GoldStdGen::getCurrentTimestamp());

            } else if (!strcmp(SENT_MESSAGE_FLAG, line_buf)) {
                logger().info("Reading message to send...");
                messageToSend = GoldStdGen::readMessage(line_buf, LINE_BUF_SIZE, goldStdFile);
                logger().fine("Message to send: %s", messageToSend->getMessage()->getPlainTextRepresentation());
                if (!messageToSend) exit(-1);
            } else {
                logger().error("Unexpected line: '%s' (expecting '%s' or '%s')", line_buf, RECEIVED_MESSAGE_FLAG, SENT_MESSAGE_FLAG);
                exit(-1);
            }
        }
    }
}

