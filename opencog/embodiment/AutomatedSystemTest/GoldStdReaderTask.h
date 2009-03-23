/**
 * GoldStdReaderTask.h
 *
 * Author: Welter Luigi
 */

#ifndef _GOLD_STD_READER_TASK_H_
#define _GOLD_STD_READER_TASK_H_

#include <IdleTask.h>
#include <NetworkElement.h>
#include "TestParameters.h"
#include "GoldStdMessage.h"

namespace AutomatedSystemTest {

const unsigned long timeout = 18000; //three minutes of timeout
 
#define LINE_BUF_SIZE 1<<16
class GoldStdReaderTask : public MessagingSystem::IdleTask {

    private:

        TestParameters& testParameters;
        FILE* goldStdFile;
        char line_buf[LINE_BUF_SIZE];
	PetaverseProxySimulator::GoldStdMessage* messageToSend;
	unsigned long initialTime;
    bool endOfFile;

    public:

        ~GoldStdReaderTask();
        GoldStdReaderTask(TestParameters&, const char* goldStdFilename);

        void run(MessagingSystem::NetworkElement *ne);

}; // class
}  // namespace

#endif
