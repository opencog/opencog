/**
 * GoldStdReaderAgent.h
 *
 * Author: Welter Luigi
 */

#ifndef _GOLD_STD_READER_AGENT_H_
#define _GOLD_STD_READER_AGENT_H_

#include <opencog/server/Factory.h>
#include <opencog/server/Agent.h>
#include "TestParameters.h"
#include "GoldStdMessage.h"

namespace AutomatedSystemTest {

using namespace opencog;

const unsigned long timeout = 18000; //three minutes of timeout
 
#define LINE_BUF_SIZE 1<<16
class GoldStdReaderAgent : public Agent {

    private:

        TestParameters* testParameters;
        FILE* goldStdFile;
        char line_buf[LINE_BUF_SIZE];
        PetaverseProxySimulator::GoldStdMessage* messageToSend;
        unsigned long initialTime;
        bool endOfFile;

    public:

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("AutomatedSystemTest::GoldStdReaderAgent");
            return _ci;
        }

        ~GoldStdReaderAgent();
        GoldStdReaderAgent();
        void init(TestParameters&, const char* goldStdFilename);

        void run(CogServer *server);

}; // class
}  // namespace

#endif
