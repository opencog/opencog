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

#include <NetworkElement.h>
#include <Message.h>
#include "TestParameters.h"
#include "GoldStdGen.h"

namespace AutomatedSystemTest {

    class PBTester : public MessagingSystem::NetworkElement {

        private:

            TestParameters& testParams;
            bool failed;
            unsigned long numberOfReceivedMessages;

            void initialize();

	    std::vector<MessagingSystem::Message*> expectedMessages;
	    std::vector<unsigned long> receivedTimeMessages;

	    PetaverseProxySimulator::GoldStdGen* goldStdGen;

        public:


            PBTester(TestParameters&);
            PBTester(const Control::SystemParameters &params, TestParameters& testParams, const std::string &myId, const std::string &ip, int portNumber);
            ~PBTester();

            /**
             * Called when NE retrieve a Message from router to perform some useful proceesing on it. Subclasses
             * are supposed to override this to perform something useful (default implementation just outputs
             * Message contents to stdout).
             */
            virtual bool processNextMessage(MessagingSystem::Message *message);

	    void addExpectedMessage(MessagingSystem::Message* message, unsigned long time);
	    bool hasExpectedMessages();
            void notifyEndOfGoldStdFile();
	    PetaverseProxySimulator::GoldStdGen* getGoldStdGen();
        unsigned long getReceivedTimeOfCurrentExpectedMessage(){ return receivedTimeMessages.back();}

    }; // class
}  // namespace

#endif
