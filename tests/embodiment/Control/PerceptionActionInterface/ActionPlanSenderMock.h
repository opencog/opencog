#include "ActionPlanSender.h"
#include "PAI.h"

// Mock subclasses of ActionPlanSender
class ResponsiveActionPlanSender : public PerceptionActionInterface::ActionPlanSender
{
private:
    PerceptionActionInterface::PAI* pai;
    string pvpMsg;

public:

    ResponsiveActionPlanSender() {}
    virtual ~ResponsiveActionPlanSender() {}

    void setPai(PerceptionActionInterface::PAI* _pai) {
        pai = _pai;
    }
    bool sendActionPlan(const PerceptionActionInterface::ActionPlan& actionPlan) {
        printf("Action plan sent/executed successfully: %s\n", actionPlan.getID().c_str());
        printf("message:\n%s\n", actionPlan.getPVPmessage(pai->getPetInterface().getPetId()).c_str());
        // Create a XML PVP message with the action plan status
        char msg[1024];
        sprintf(msg, "<pet:petaverse-msg\n"
                "xmlns:pet=\"http://proxy.esheepco.com/brain\"\n"
                "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
                "xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\">\n"
                "<pet-signal pet-id=\"%s\" name=\"whatever\" status=\"done\" action-plan-id=\"%s\" timestamp=\"2007-06-18T20:15:00.000-07:00\" />\n"
                "</pet:petaverse-msg>",
                pai->getPetInterface().getPetId().c_str(), actionPlan.getID().c_str());
        pvpMsg.assign(msg);
        return true;
    }

    void proccessSentMessage() {

        if (pvpMsg.length() > 0) {
            HandleSeq handles;
            pai->processPVPMessage(pvpMsg, handles);
            pvpMsg = "";
        }

    }

    bool sendEmotionalFeelings(const std::string& feelings) {
        printf("Emotional Feelings sent successfully ");
        printf("message:\n%s\n", feelings.c_str());
        return true;
    }
};

class OKActionPlanSender : public PerceptionActionInterface::ActionPlanSender
{
private:
    std::list<PerceptionActionInterface::ActionPlan>& plans;

public:

    OKActionPlanSender(std::list<PerceptionActionInterface::ActionPlan>& _plans) : plans(_plans) {}
    virtual ~OKActionPlanSender() {}

    bool sendActionPlan(const PerceptionActionInterface::ActionPlan& actionPlan) {
        //printf("Action plan sent successfully\n");
        plans.push_back(actionPlan);
        return true;
    }

    bool sendEmotionalFeelings(const std::string& feelings) {
        return true;
    }

};

class FailureActionPlanSender : public PerceptionActionInterface::ActionPlanSender
{
public:
    virtual ~FailureActionPlanSender() {}

    bool sendActionPlan(const PerceptionActionInterface::ActionPlan& actionPlan) {
        //printf("Could not sent Action plan\n");
        return false;
    }

    bool sendEmotionalFeelings(const std::string& feelings) {
        return false;
    }

};

