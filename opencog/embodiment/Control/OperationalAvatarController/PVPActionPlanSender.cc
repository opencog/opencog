/*
 * opencog/embodiment/Control/OperationalAvatarController/PVPActionPlanSender.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


/**
 * Concrete subclass of ActionPlanSender that sends action plans to the PVP
 */

#include "PVPActionPlanSender.h"
#include <opencog/embodiment/Control/MessagingSystem/StringMessage.h>
#include <opencog/util/Config.h>

namespace opencog { namespace oac {

using namespace std;
using namespace pai;
using namespace oac;
using messaging::StringMessage;

PVPActionPlanSender::PVPActionPlanSender(const std::string& petId,
                                         NetworkElement * ne)
{
    this->petId = petId;
    this->ne = ne;
    this->logPVPMessage = !(config().get_bool("DISABLE_LOG_OF_PVP_MESSAGES"));
}

PVPActionPlanSender::~PVPActionPlanSender()
{
}

bool PVPActionPlanSender::sendActionPlan(const ActionPlan& actionPlan)
{
    StringMessage msg(ne->getID(), config().get("PROXY_ID"),
                      actionPlan.getPVPmessage(petId));
    if (logPVPMessage) {
        logger().info("PVPActionPlanSender::sendActionPlan():\n%s\n",
                      msg.getPlainTextRepresentation());
    }
    return ne->sendMessage(msg);
}

bool PVPActionPlanSender::sendSpecificActionFromPlan(const ActionPlan& actionPlan,
                                                     unsigned int actionSequenceNum)
{
	StringMessage msg(ne->getID(), config().get("PROXY_ID"),
                      actionPlan.getPVPmessage(petId, actionSequenceNum));
    if (logPVPMessage) {
        logger().info("PVPActionPlanSender::sendActionPlan():\n%s\n",
                      msg.getPlainTextRepresentation());
    }
    return ne->sendMessage(msg);

}

bool PVPActionPlanSender::sendEmotionalFeelings(const std::string& feelings)
{
    StringMessage msg(ne->getID(), config().get("PROXY_ID"), feelings);
    if (logPVPMessage) {
        logger().info("PVPActionPlanSender::sendEmotionalFeelings():\n%s\n", msg.getPlainTextRepresentation());
    }
    return ne->sendMessage(msg);
}

bool PVPActionPlanSender::sendSingleActionCommand(const std::string& action)
{
    StringMessage msg(ne->getID(), config().get("PROXY_ID"), action);
    if (logPVPMessage) {
        logger().info("PVPActionPlanSender::sendSingleActionCommand():\n%s\n", msg.getPlainTextRepresentation());
    }
    return ne->sendMessage(msg);
}

} // ~namespace oac
} // ~namespace opencog
