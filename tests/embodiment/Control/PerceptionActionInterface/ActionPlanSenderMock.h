/*
 * tests/embodiment/Control/PerceptionActionInterface/ActionPlanSenderMock.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionPlanSender.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

using std::string;

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
        printf("message:\n%s\n", actionPlan.getPVPmessage(pai->getAvatarInterface().getPetId()).c_str());
        // Create a XML PVP message with the action plan status
        char msg[1024];
        sprintf(msg, "<pet:petaverse-msg\n"
                "xmlns:pet=\"http://proxy.esheepco.com/brain\"\n"
                "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
                "xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\">\n"
                "<pet-signal pet-id=\"%s\" name=\"whatever\" status=\"done\" action-plan-id=\"%s\" timestamp=\"2007-06-18T20:15:00.000-07:00\" />\n"
                "</pet:petaverse-msg>",
                pai->getAvatarInterface().getPetId().c_str(), actionPlan.getID().c_str());
        pvpMsg.assign(msg);
        return true;
    }

    bool sendSpecificActionFromPlan(const PerceptionActionInterface::ActionPlan& actionPlan, unsigned int actionSequenceNum) {
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

    bool sendSpecificActionFromPlan(const PerceptionActionInterface::ActionPlan& actionPlan, unsigned int actionSequenceNum) {
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

    bool sendSpecificActionFromPlan(const PerceptionActionInterface::ActionPlan& actionPlan, unsigned int actionSequenceNum) {
        return false;
    }

    bool sendEmotionalFeelings(const std::string& feelings) {
        return false;
    }

};

