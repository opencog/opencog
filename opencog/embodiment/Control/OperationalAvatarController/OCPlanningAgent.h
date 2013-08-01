/*
 * @file opencog/embodiment/Control/OperationalAvatarController/OCPlanningAgent.h
 *
 * @author Shujing KE <rainkekekeke@gmail.com>
 * @date 2013-07-27
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

#ifndef OCPLANNING_AGENT_H
#define OCPLANNING_AGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>
#include "OCPlanner.h"

namespace opencog { namespace oac {

class OCPlanningAgent : public opencog::Agent
{
protected:

    unsigned long cycleCount;  // Indicate how many times this mind
                               // agent has been executed

    // Indicate whether the mind agent has been initialized,
    // then the 'run' method will not initialize it over and over,
    // unless you call 'forceInitNextCycle' method.
    bool bInitialized;

    OCPlanner* ocplanner;

    ActionPlanID currentOCPlanID; // only be used by ocplanner

    Handle hSelectedDemandGoal;

    HandleSeq current_actions;

    Handle current_action;

    int current_step; // the undergoing step number in the current plan

    bool use_ocplanner;

    // Time out for executing Action (combo script) defined by PROCEDURE_EXECUTION_TIMEOUT
    long procedureExecutionTimeout;

    time_t timeStartCurrentAction; // When the current action was executed


    bool isMoveAction(string s);

public:

    OCPlanningAgent();
    void init(opencog::CogServer * server);

    virtual ~OCPlanningAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::OCPlanningAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    void run(opencog::CogServer * server);


    // After calling this function, the Agent will invoke its "init" method firstly
    // in "run" function during its next cycle
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

} } // namespace opencog::oac

#endif
