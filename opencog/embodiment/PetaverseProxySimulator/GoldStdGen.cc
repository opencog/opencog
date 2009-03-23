/**
 * GoldStdGen.cc
 *
 * Author: Welter Luigi
 */

#include "GoldStdGen.h"
#include "util/Logger.h"
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace PetaverseProxySimulator;
using namespace MessagingSystem;

GoldStdGen::GoldStdGen(const char* goldStdFilename) {
    file = fopen(goldStdFilename, "w");
    if (file) {
        logger().log(opencog::Logger::INFO, "Generating gold standard in file: %s", goldStdFilename);
    } else {
        logger().log(opencog::Logger::ERROR, "Could not open gold standard file: %s", goldStdFilename);
    }
    initial_time = getCurrentTimestamp();
}

GoldStdGen::~GoldStdGen() {
    if (file) {
        fclose(file);
        fflush(file);
    }
}

            
void GoldStdGen::writeMessage(MessagingSystem::Message& message, bool sending) {
    if (file) {
        fprintf(file, "%s%lu\n%s %s %d\n%s\n%s", 
                      sending?SENT_MESSAGE_FLAG:RECEIVED_MESSAGE_FLAG,
                      getCurrentTimestamp()-initial_time,
                      message.getFrom().c_str(), 
                      message.getTo().c_str(),
                      message.getType(),
                      message.getPlainTextRepresentation(),
                      MESSAGE_END_FLAG); 
        fflush(file);
    }
}

GoldStdMessage* GoldStdGen::readMessage(char* line_buf, size_t lineBufSize, FILE* file) {
    // read timestamp
    if (fgets(line_buf, lineBufSize, file) == NULL) {
        logger().log(opencog::Logger::ERROR, "Unexpected end of file while reading message's header");
        return NULL;
    }
    unsigned long timestamp;
    sscanf(line_buf, "%lu", &timestamp);

    // read message header
    if (fgets(line_buf, lineBufSize, file) == NULL) {
        logger().log(opencog::Logger::ERROR, "Unexpected end of file while reading message's header");
        return NULL;
    }
    char from[64];
    char to[64];
    int type;
    sscanf(line_buf, "%s %s %d", from, to, &type);

    // read the message body
    std::string message; 
    while (true) {
        if (fgets(line_buf, lineBufSize, file) == NULL) {
            logger().log(opencog::Logger::ERROR, "Unexpected end of file while reading message: %s", message.c_str());
            return NULL;
        }
        if (!strcmp(MESSAGE_END_FLAG, line_buf)) {
            break;
        }
        message += line_buf;
    }
    // check and remove the '\n' added by writeMessage() method at the end of the message
    if (message.at(message.length()-1) != '\n') {
        logger().log(opencog::Logger::ERROR, "Failed reading message, which should terminate with <new line> character: %s", message.c_str());
        exit(-1);
    }
    message.erase(message.length()-1); 
    
    return new GoldStdMessage(timestamp, Message::factory(from, to, type, message));
}

unsigned long GoldStdGen::getCurrentTimestamp() {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    string timeStr = to_iso_extended_string(now);
    //cout << "Current date/time = " << timeStr << endl;
    return PerceptionActionInterface::PAI::getTimestampFromXsdDateTimeStr(timeStr.c_str());
}

