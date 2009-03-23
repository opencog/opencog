/**
 * GoldStdGen.h
 *
 * Author: Welter Luigi
 */

#ifndef _GOLD_STD_GEN_H_
#define _GOLD_STD_GEN_H_

#include <stdio.h>
#include "GoldStdMessage.h"
#include "PAI.h" // This should be here (not in cc file) since it defines DATETIME_DECIMAL_RESOLUTION

#define SENT_MESSAGE_FLAG "SENT MESSAGE:\n"
#define RECEIVED_MESSAGE_FLAG "RECEIVED MESSAGE:\n"
#define MESSAGE_END_FLAG "MESSAGE END.\n"

namespace PetaverseProxySimulator {

    class GoldStdGen {

        private:

            FILE* file;
            unsigned long initial_time;

        public:

            GoldStdGen(const char* goldStdFilename);
            ~GoldStdGen();
            
            void writeMessage(MessagingSystem::Message& message, bool sending);
            static GoldStdMessage* readMessage(char* line_buf, size_t lineBufSize, FILE* file);
            static unsigned long getCurrentTimestamp();

    }; // class
}  // namespace

#endif
