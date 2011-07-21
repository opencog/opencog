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
using namespace opencog::pai;

// Mock subclasses of ActionPlanSender
class ResponsiveActionPlanSender : public ActionPlanSender
{
private:
    PAI* pai;
    string pvpMsg;

public:

    ResponsiveActionPlanSender() {}
    virtual ~ResponsiveActionPlanSender() {}

    void setPai(PAI* _pai) {
        pai = _pai;
    }
    bool sendActionPlan(const ActionPlan& actionPlan) {
        printf("Action plan sent/executed successfully: %s\n", actionPlan.getID().c_str());
        printf("message:\n%s\n", actionPlan.getPVPmessage(pai->getAvatarInterface().getPetId()).c_str());
        // Create a XML PVP message with the action plan status
        char msg[1024];
        sprintf(msg, "<pet:embodiment-msg\n"
                "xmlns:pet=\"http://www.opencog.org/brain\"\n"
                "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
                "xsi:schemaLocation=\"http://www.opencog.org/brain BrainProxyAxon.xsd\">\n"
                "<agent-signal id=\"%s\" timestamp=\"2007-06-18T20:15:00.000-07:00\">\n"
                "<action name=\"whatever\" status=\"done\" plan-id=\"%s\"/></agent-signal>\n"
                "</pet:embodiment-msg>",
                pai->getAvatarInterface().getPetId().c_str(), actionPlan.getID().c_str());
        pvpMsg.assign(msg);
        return true;
    }

    bool sendSpecificActionFromPlan(const ActionPlan& actionPlan, unsigned int actionSequenceNum) {
        return true;
    }

    void processSentMessage() {

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

class OKActionPlanSender : public ActionPlanSender
{
private:
    std::list<ActionPlan>& plans;

public:

    OKActionPlanSender(std::list<ActionPlan>& _plans) : plans(_plans) {}
    virtual ~OKActionPlanSender() {}

    bool sendActionPlan(const ActionPlan& actionPlan) {
        //printf("Action plan sent successfully\n");
        plans.push_back(actionPlan);
        return true;
    }

    bool sendSpecificActionFromPlan(const ActionPlan& actionPlan, unsigned int actionSequenceNum) {
        return true;
    }

    bool sendEmotionalFeelings(const std::string& feelings) {
        return true;
    }

};

class FailureActionPlanSender : public ActionPlanSender
{
public:
    virtual ~FailureActionPlanSender() {}

    bool sendActionPlan(const ActionPlan& actionPlan) {
        //printf("Could not sent Action plan\n");
        return false;
    }

    bool sendSpecificActionFromPlan(const ActionPlan& actionPlan, unsigned int actionSequenceNum) {
        return false;
    }

    bool sendEmotionalFeelings(const std::string& feelings) {
        return false;
    }

};

