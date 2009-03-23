/**
 * GoldStdReaderTask.cc
 *
 * Author: Welter Luigi
 */

#include "GoldStdReaderTask.h"
#include "PBTester.h"
#include <unistd.h>

using namespace AutomatedSystemTest;
using namespace MessagingSystem;
using namespace PetaverseProxySimulator;
using namespace opencog;

GoldStdReaderTask::~GoldStdReaderTask() {
    if (goldStdFile) fclose(goldStdFile);
}

GoldStdReaderTask::GoldStdReaderTask(TestParameters& _testParameters, const char* goldStdFilename) : testParameters(_testParameters) {
    goldStdFile = fopen(goldStdFilename, "r");
    if (!goldStdFile) {
        logger().log(opencog::Logger::ERROR, "Could not open gold standard file: %s", goldStdFilename);
	exit(-1);
    }
    messageToSend = NULL;
    initialTime = GoldStdGen::getCurrentTimestamp();
    endOfFile = false;
}

void GoldStdReaderTask::run(MessagingSystem::NetworkElement *ne) {

    PBTester* pbTester = (PBTester*) ne;
    //if (!pbTester->hasExpectedMessages()) { // THIS WAY, IT WAITS NEXT CYCLE TO RUN THE NEXT CONSECUTIVE MESSAGE
    if (!pbTester->hasExpectedMessages() && endOfFile){
        pbTester->notifyEndOfGoldStdFile();
        return;
    }

    if (pbTester->hasExpectedMessages())
    {
        unsigned long elapsedTime = GoldStdGen::getCurrentTimestamp() - pbTester->getReceivedTimeOfCurrentExpectedMessage(); 
        if (elapsedTime > timeout) {
            logger().log(opencog::Logger::ERROR, "Timeout. elapsed time: %lu.", elapsedTime);
            exit(-1);
        }
    } 

    while (!pbTester->hasExpectedMessages() && !endOfFile) { // THIS WAY, IT SENDS ALL CONSECUTIVE MESSAGES AT ONCE
        logger().log(opencog::Logger::INFO, "No expected messages");
        if (messageToSend) {
            unsigned long elapsedTime = GoldStdGen::getCurrentTimestamp() - initialTime;
            if (elapsedTime >= messageToSend->getTimestamp()) {
                logger().log(opencog::Logger::INFO, "Send last read message...");
                pbTester->sendMessage(*(messageToSend->getMessage()));
                if (testParameters.get("SAVE_MESSAGES_TO_FILE") == "1") {
                    pbTester->getGoldStdGen()->writeMessage(*(messageToSend->getMessage()), true);
                }
                delete messageToSend;
                messageToSend = NULL;
            } else {
#ifdef DATETIME_DECIMAL_RESOLUTION
                unsigned long usec = (messageToSend->getTimestamp() - elapsedTime)*100000; 
#else
                unsigned long usec = (messageToSend->getTimestamp() - elapsedTime)*10000; 
#endif
                logger().log(opencog::Logger::INFO, "Sleeping for %lu micro seconds before sending next message...", usec);
                usleep(usec);
            }
        }
        while (!messageToSend && !endOfFile) {
            logger().log(opencog::Logger::INFO, "Resume reading of gold std file...");
            if (fgets(line_buf, LINE_BUF_SIZE, goldStdFile) == NULL) {
                endOfFile = true;
                break;
                //pbTester->notifyEndOfGoldStdFile();
                //return;
            }
            if (!strcmp(RECEIVED_MESSAGE_FLAG, line_buf)) {
                logger().log(opencog::Logger::INFO, "Reading expected message...");
                GoldStdMessage* msg = GoldStdGen::readMessage(line_buf, LINE_BUF_SIZE, goldStdFile);
                if (!msg) exit(-1);
                
                pbTester->addExpectedMessage(msg->getMessage(), GoldStdGen::getCurrentTimestamp());
  
            } else if (!strcmp(SENT_MESSAGE_FLAG, line_buf)) {
                logger().log(opencog::Logger::INFO, "Reading message to send...");
                messageToSend = GoldStdGen::readMessage(line_buf, LINE_BUF_SIZE, goldStdFile);
                logger().log(opencog::Logger::FINE, "Message to send: %s", messageToSend->getMessage()->getPlainTextRepresentation());
                if (!messageToSend) exit(-1);
            } else {
                logger().log(opencog::Logger::ERROR, "Unexpected line: '%s' (expecting '%s' or '%s')", line_buf, RECEIVED_MESSAGE_FLAG, SENT_MESSAGE_FLAG);
                exit(-1);
            }
        }
    }
}

