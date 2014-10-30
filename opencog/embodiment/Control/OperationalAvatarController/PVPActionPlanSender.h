/*
 * opencog/embodiment/Control/OperationalAvatarController/PVPActionPlanSender.h
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

#ifndef PVPACTIONPLANSENDER_H_
#define PVPACTIONPLANSENDER_H_

/**
 * Concrete subclass of ActionPlanSender that sends action plans to the PVP
 */

#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionPlanSender.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionPlan.h>

namespace opencog { namespace oac {

using namespace messaging;

class PVPActionPlanSender: public pai::ActionPlanSender
{

private:

    /**
     * The id of the pet which the action plans will be sent for
     */
    std::string petId;

    /**
     * A network element object used to send the action plan in a message to PVP
     */
    NetworkElement* ne;

    /**
     * Indicates if the pvp messages should be logged or not.
     */
    bool logPVPMessage;

public:

    /**
     * Constructor
     */
    PVPActionPlanSender(const std::string& petId, NetworkElement *);
    ~PVPActionPlanSender();

    /**
     * Sends the action plan to the target Virtual World.
     *
     * @param actionPlan a reference to the object that contains the
     *        action plan to be sent. Note that this reference may not
     *        be valid after the call of this method because caller
     *        may release the object.
     *
     * @return a boolean value that indicates the success (true) or
     *         failure (false) of this sending operation.
     */
    bool sendActionPlan(const pai::ActionPlan& actionPlan);

    /**
     * Sends a specific(atomic) action from plan to the target virtual world.
     *
     * @param actionPlan a reference to the object that contains the action plan to be sent. 
     *
     * @param actionSequenceNum a specific sequence number of an atomic action in a 
     *        given action plan
     *
     * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
     */
    bool sendSpecificActionFromPlan(const pai::ActionPlan& actionPlan,
            unsigned int actionSequenceNum);

    /**
     * Sends the emotional feelings meessage to the target Virtual World.
     *
     * @param feelings The XML message already converted int a string representation
     *
     * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
     */
    bool sendEmotionalFeelings(const std::string& emotionalFeelings);

    bool sendSingleActionCommand(const std::string& action);
};

} } // namespace opencog::oac

#endif // PVPACTIONPLANSENDER_H_
