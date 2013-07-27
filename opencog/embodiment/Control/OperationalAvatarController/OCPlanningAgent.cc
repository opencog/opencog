/*
 * @file opencog/embodiment/Control/OperationalAvatarController/OCPlanningAgent.cc
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

#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/util/platform.h>

#include "OAC.h"
#include "OCPlanningAgent.h"
#include "Strips.h"
#include "PsiRuleUtil.h"

using namespace opencog::oac;

OCPlanningAgent::OCPlanningAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

void OCPlanningAgent::init(opencog::CogServer * server)
{
    logger().debug( "OCPlanningAgent::%s - Initializing the Agent [cycle = %d]",
                    __FUNCTION__,
                    this->cycleCount
                  );

    // Get OAC
    this->oac = dynamic_cast<OAC*>(server);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace
    this->atomSpace = oac->getAtomSpace();

    // Avoid initialize during next cycle
    this->bInitialized = true;

    this->use_ocplanner = config().get_long("USE_OCPLANNDER");

    // if use ocplanner , create a ocplanner
    if (use_ocplanner)
    {
        ocplanner = new OCPlanner(atomSpace ,oac);
        logger().debug("PsiActionSelectionAgent::init - using OCPlanner! ");
    }

    this->currentOCPlanID = "";
}

void OCPlanningAgent::getCurrentDemand()
{
    // Select the most important demand for now, by scm
    SchemeEval & evaluator = SchemeEval::instance();
    std::string scheme_expression;

    scheme_expression = "( update_selected_demand_goal )";

    // Run the Procedure that do planning
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() )
    {
        logger().error( "OCPlanningAgent::%s -getCurrentDemand() failed '%s'",
                         __FUNCTION__,
                         scheme_expression.c_str()
                      );

        hSelectedDemandGoal = Handle::UNDEFINED;
    }

    hSelectedDemandGoal =  AtomSpaceUtil::getReference(atomSpace,
                                    atomSpace.getHandle(CONCEPT_NODE,
                                                        "plan_selected_demand_goal"));

    // Update the pet's previously/ currently Demand Goal
    PsiRuleUtil::setCurrentDemandGoal(atomSpace, this->hSelectedDemandGoal);

}

void OCPlanningAgent::runOCPlanner()
{
    getCurrentDemand();

    logger().debug( "OCPlanningAgent::%s - do planning for the Demand Goal: %s [cycle = %d]",
                    __FUNCTION__,
                    atomSpace.atomAsString(this->hSelectedDemandGoal).c_str(),
                    this->cycleCount
                  );

    currentOCPlanID = ocplanner->doPlanning();

    // if planning failed
    if (currentOCPlanID == "")
    {

    }
}

void OCPlanningAgent::run(opencog::CogServer * server)
{
    this->cycleCount = server->getCycleCount();

    logger().debug( "PsiActionSelectionAgent::%s - Executing run %d times",
                     __FUNCTION__,
                     this->cycleCount
                  );


    if (this->currentOCPlanID != "")
    {
        if (! oac->getPAI().isPlanFinished(currentOCPlanID))
        {
            // the plan is still being execting
            return;

        }

        if(oac->getPAI().hasPlanFailed(currentOCPlanID))
        {
            std::cout<<std::endl<<"OCPlanningAgent::Action execution failed! Plan ID = "<< this->currentOCPlanID
                     <<std::endl<<"Selected Demand Goal [cycle = "<<this->cycleCount<<"]:"
                     <<std::endl<<atomSpace.atomAsString(this->hSelectedDemandGoal)<<std::endl;

        }
        else
        {
            std::cout<<std::endl<<"OCPlanningAgent::Action execution success! Plan ID = "<< this->currentOCPlanID
                     <<std::endl<<"Selected Demand Goal [cycle = "<<this->cycleCount<<"]:"
                     <<std::endl<<atomSpace.atomAsString(this->hSelectedDemandGoal)<<std::endl;
        }

        this->currentOCPlanID = "";

    }

    runOCPlanner();

}
