#ifndef LSMESSAGESENDERMOCK_H_
#define LSMESSAGESENDERMOCK_H_

#include "MessageSender.h"
using namespace OperationalPetController;

#include <stdio.h>

class LSMessageSenderMock : public MessageSender {
    
    public:
        LSMessageSenderMock() {}
        ~LSMessageSenderMock() {}    
         
        bool sendReward(const std::string &schema, const std::vector<std::string> & schemaArgs, const std::string &triedSchema, const double reward){
            fprintf(stdout, "Sending reward message. schema: '%s', triedSchema: '%s', reward: '%f'\n", schema.c_str(), triedSchema.c_str(), reward);
            return true;
        }
        
        bool sendExemplar(const std::string &schema, const std::vector<std::string> &schemaArgs, const std::string &ownerId, const std::string &avatarId, SpaceServer &spaceServer){
            fprintf(stdout, "Sending exemplar. schema: '%s', owner: '%s', avatar: '%s'\n", schema.c_str(), ownerId.c_str(), avatarId.c_str());            
            return true;
        }
    
        bool sendCommand(const std::string &command, const std::string &schema){
            fprintf(stdout, "Sending command. command: '%s', schema: '%s'\n", command.c_str(), schema.c_str());            
            return true;
        }

        bool sendFeedback(const std::string &ownerId, const std::string &feedback){
            fprintf(stdout, "Sending feedback. ownerId: '%s', feedback: '%s'\n", ownerId.c_str(), feedback.c_str());            
            return true;
        }
        
        bool sendTrySchema(const std::string &schemaName, const std::vector<std::string> & schemaArgs) {
            fprintf(stdout, "Sending Try command. schema name: '%s'.\n", schemaName.c_str());            

            return true;
        }
	
        bool sendStopLearning(const std::string &schemaName, const std::vector<std::string> & schemaArgs) {
            fprintf(stdout, "Sending stop learning command. schema name: '%s'.\n", schemaName.c_str());            

            return true;
        }
};

#endif /*LSMESSAGESENDERMOCK_H_*/
