/** 
 * PVPActionPlanSender.cc
 * 
 * Concrete subclass of ActionPlanSender that sends action plans to the PVP 
 * 
 * Author: Welter Luigi 
 * Copyright(c), 2007
 */
 
#include "PVPActionPlanSender.h"
#include "StringMessage.h"

using namespace OperationalPetController;
using namespace std;
using namespace opencog;

PVPActionPlanSender::PVPActionPlanSender(const std::string& petId, NetworkElement * ne) {
    this->petId = petId;
    this->ne = ne;
    this->logPVPMessage = (atoi(ne->parameters.get(std::string("DISABLE_LOG_OF_PVP_MESSAGES")).c_str()) == 0);
}

PVPActionPlanSender::~PVPActionPlanSender() {
}

bool PVPActionPlanSender::sendActionPlan(const PerceptionActionInterface::ActionPlan& actionPlan) {
    MessagingSystem::StringMessage msg(ne->getID(), ne->parameters.get("PROXY_ID"), actionPlan.getPVPmessage(petId));
    if (logPVPMessage) {
        logger().log(opencog::Logger::INFO, "PVPActionPlanSender::sendActionPlan():\n%s\n", msg.getPlainTextRepresentation());
    }
    return ne->sendMessage(msg);
}

bool PVPActionPlanSender::sendEmotionalFeelings(const std::string& feelings){
    MessagingSystem::StringMessage msg(ne->getID(), ne->parameters.get("PROXY_ID"), feelings);
    if (logPVPMessage) {
        logger().log(opencog::Logger::INFO, "PVPActionPlanSender::sendEmotionalFeelings():\n%s\n", msg.getPlainTextRepresentation());
    }
    return ne->sendMessage(msg);
}
