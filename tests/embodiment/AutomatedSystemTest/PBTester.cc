/**
 * PBTester.cc
 * This program performs automated system tests on PB-side of Petaverse code.
 * It simulates the PVP Proxy by reading from a file a sequence of messages to be sent and 
 * received to/from Petaverse ROUTER. 
 *
 * Author: Welter Luigi
 */

#include "PBTester.h"

using namespace AutomatedSystemTest;
using namespace MessagingSystem;
using namespace PetaverseProxySimulator;

PBTester::~PBTester() {
    if (testParams.get("SAVE_MESSAGES_TO_FILE") == "1") {
        delete goldStdGen;
    }
}

PBTester::PBTester(TestParameters& _testParams) : testParams(_testParams) {
    initialize();
}

PBTester::PBTester(const Control::SystemParameters &params, TestParameters& _testParams, const std::string &myId, const std::string &ip, int portNumber) : NetworkElement(params, myId, ip, portNumber), testParams(_testParams) {
    initialize();
}

void PBTester::initialize() {
    if (testParams.get("SAVE_MESSAGES_TO_FILE") == "1") {
        goldStdGen = new GoldStdGen(testParams.get("MESSAGES_FILENAME").c_str());
    }
    failed = false;
    numberOfReceivedMessages = 0;
}

bool PBTester::processNextMessage(MessagingSystem::Message *message) {
    //MAIN_LOGGER.log(opencog::Logger::INFO, "RECEIVED MESSAGE:\n%s", message->getPlainTextRepresentation());

    if (testParams.get("SAVE_MESSAGES_TO_FILE") == "1") {
        goldStdGen->writeMessage(*message, false);
    }
    if (expectedMessages.empty()) {
        // TODO: implement a timeout to wait for the message be read
        MAIN_LOGGER.log(opencog::Logger::ERROR, "Received message when there is no expected message! Unexpected message: \n%s\n",message->getPlainTextRepresentation());
	exit(-1);
    } else {
        Message* expectedMessage = expectedMessages[0];
        if (strcmp(expectedMessage->getPlainTextRepresentation(),
                   message->getPlainTextRepresentation())) {
            MAIN_LOGGER.log(opencog::Logger::ERROR, "Received message does not match the expected one!\nReceived:\n'%s'\nExpected:\n'%s'\n", message->getPlainTextRepresentation(), expectedMessage->getPlainTextRepresentation());
         //   failed = true;	
            expectedMessages.erase(expectedMessages.begin());
            receivedTimeMessages.erase(receivedTimeMessages.begin());

            delete expectedMessage;
            
            exit(-1);
        }
	expectedMessages.erase(expectedMessages.begin());
	receivedTimeMessages.erase(receivedTimeMessages.begin());

	delete expectedMessage;
    }
    MAIN_LOGGER.log(opencog::Logger::INFO, "Number of expected messages: %d", expectedMessages.size()); 
    numberOfReceivedMessages++;
    return false;
}

void PBTester::addExpectedMessage(Message* message, unsigned long time) {
    expectedMessages.push_back(message);
    receivedTimeMessages.push_back(time);

    MAIN_LOGGER.log(opencog::Logger::INFO, "Number of expected messages: %d", expectedMessages.size()); 
}

bool PBTester::hasExpectedMessages() {
    return !expectedMessages.empty();
}

void PBTester::notifyEndOfGoldStdFile() {
    if (!expectedMessages.empty()) {
        MAIN_LOGGER.log(opencog::Logger::ERROR, "End of gold std file reached without receiving some expected messages.");
        failed = true;	
    }
    if (!numberOfReceivedMessages) {
        MAIN_LOGGER.log(opencog::Logger::ERROR, "Did not received any message.");
        failed = true;	
    }
    std::string msgCmd = "UNLOAD_PET Fido";
#if 0
    MessagingSystem::StringMessage msg(NetworkElement::parameters.get("PROXY_ID"), NetworkElement::parameters.get("SPAWNER_ID"), msgCmd);
    sendMessage(msg);
#endif
    if (failed) {
        MAIN_LOGGER.log(opencog::Logger::ERROR, "Automated system test failed!");
	exit(-1);
    }
    MAIN_LOGGER.log(opencog::Logger::INFO, "Automated system test passed!");
    exit(0);
}

GoldStdGen* PBTester::getGoldStdGen() {
    return goldStdGen;
}
