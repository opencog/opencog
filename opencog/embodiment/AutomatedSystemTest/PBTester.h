/**
 * PBTester.h
 *
 * Author: Welter Luigi 
 */

#ifndef _PB_TESTER_H
#define _PB_TESTER_H

#include <string>
#include <map>
#include <set>

#include <EmbodimentCogServer.h>
#include <Message.h>
#include "TestParameters.h"
#include "GoldStdGen.h"

namespace AutomatedSystemTest {

    class PBTester : public MessagingSystem::EmbodimentCogServer {

        private:

            TestParameters* testParams;
            bool failed;
            unsigned long numberOfReceivedMessages;

            void initialize();

            std::vector<MessagingSystem::Message*> expectedMessages;
            std::vector<unsigned long> receivedTimeMessages;

            PetaverseProxySimulator::GoldStdGen* goldStdGen;

        public:

            static opencog::BaseServer* createInstance();

            PBTester();
            void init(TestParameters&);
            void init(const Control::SystemParameters &params, TestParameters& testParams, const std::string &myId, const std::string &ip, int portNumber);
            ~PBTester();

            bool processNextMessage(MessagingSystem::Message *message);
            void addExpectedMessage(MessagingSystem::Message* message, unsigned long time);
            bool hasExpectedMessages();
            void notifyEndOfGoldStdFile();
            PetaverseProxySimulator::GoldStdGen* getGoldStdGen();
            unsigned long getReceivedTimeOfCurrentExpectedMessage(){ return receivedTimeMessages.back();}

    }; // class
}  // namespace

#endif
