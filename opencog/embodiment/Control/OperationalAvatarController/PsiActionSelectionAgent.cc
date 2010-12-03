/*
 * @file opencog/embodiment/Control/OperationalPetController/ActionSelectionAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Andre Senna, Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-11-15
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

#include "OPC.h"
#include "ActionSelectionAgent.h"

using namespace OperationalPetController;

ActionSelectionAgent::~ActionSelectionAgent()
{

}

ActionSelectionAgent::ActionSelectionAgent()
{
    lastTickTime = 0;
}

int ActionSelectionAgent::prepareForActionSelection(opencog::CogServer *server)
{
    logger().fine("ActionSelectionAgent::prepareForNextCycle - prepare for selecting an action");

    OPC * opc = (OPC *) server;

    // TODO: what's the job of ModeHandler? maybe we can remove it later, by Zhenhua Cai, on 2010-11-15
    opc->getPet().getCurrentModeHandler().update();

    // TODO: remove this function calling later 
    opc->getRuleEngine().prepareForNextCycle( );

    // TODO: remove obsolete configurations below
    //       "RE_SCHEMA_SELECTION_RANDOM_NOISE"
    //       "RE_WILD_CARD_RANDOM_SELECTION"

    // Process next action only if there is map info data available and pet spatial info is already received
    if ( opc->getAtomSpace()->getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn("ActionSelectionAgent::selectAction - There is no map info available yet!");
        return -1;
    }

    if ( !opc->getAtomSpace()->getSpaceServer().getLatestMap().containsObject(opc->getPet().getPetId()) ) {
        logger().error("ActionSelectionAgent::selectAction - Pet was not inserted in the space map yet!" );
        return -1;
    }

    /* TODO: Maybe we need another MindAgeent called FeelingUpdaterAgent 
    //       that map Modulators to a set of feelings and send them to PAI

    if (getCurrentAction().length() == 0) {
        std::map<std::string, float> feelingsUpdatedMap;
        foreach(std::string feeling, feelings) {
            float value = AtomSpaceUtil::getPredicateValue(*(opc->getAtomSpace()),
                          feeling, petHandle);
            feelingsUpdatedMap[ feeling ] = value;
        } // foreach

        opc->getPAI().sendEmotionalFeelings(this->petName,
                                            feelingsUpdatedMap);
    }
    */

    // TODO: make these function below members of ActionSelectionAgent
    
    // update novelty predicates
    opc->getRuleEngine().updateKnownEntities();

    // update last pet action done and current action repetitions predicates
    opc->getRuleEngine().updateSchemaExecutionStatus();

    opc->getRuleEngine().processRules();
}

void ActionSelectionAgent::selectAction(opencog::CogServer *server)
{
    OPC * opc = (OPC *) server;

    // TODO: remove RuleEngine::processNextAction once finished 
    //       this->opc->getRuleEngine().processNextAction();

    /* TODO: select an action using PLN
    //       for current stage, just split learning and other activities (randomly choose an action)
    //       once the whole system works, real PLN planner will be applied 
    logger().debug(
                     "RuleEngine - Selected candidate rule '%s',"
                     " schema '%s' with weigth '%.3f'.",
                     this->candidateRule.c_str(),
                     this->candidateAction.getName().c_str(),
                     maximumWeight);

    } // end block
    */



    // TODO: what's the job of the line below? Make it a member of ActionSelectionAgent if necessary.
    // update the STI values of the learned tricks
//    opc->getRuleEngine().learnedTricksHandler->update();
}

void ActionSelectionAgent::executeAction(opencog::CogServer *server)
{
    OPC * opc = (OPC *) server;

    // TODO: remove RuleEngine::runSchemaForCurrentAction once finished
    // opc->getRuleEngine().runSchemaForCurrentAction();
}

void ActionSelectionAgent::run(opencog::CogServer *server)
{
    logger().fine("ActionSelectionAgent::run - Executing run");

    if ( this->prepareForActionSelection(server) == 0 ) {
        this->selectAction(server);
        this->executeAction(server);
    }
}
