/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-01-25
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


#include "OAC.h"
#include "PsiActionSelectionAgent.h"


#include<boost/tokenizer.hpp>

using namespace OperationalAvatarController;

PsiActionSelectionAgent::~PsiActionSelectionAgent()
{

}

PsiActionSelectionAgent::PsiActionSelectionAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

void PsiActionSelectionAgent::init(opencog::CogServer * server) 
{
    logger().debug( "PsiActionSelectionAgent::%s - Initialize the Agent [ cycle = %d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    this->atomSpace = oac->getAtomSpace();

    // Get Procedure repository
//    const Procedure::ProcedureRepository & procedureRepository = 
//                                               oac->getProcedureRepository();

    // Get petId
//    const std::string & petId = oac->getPet().getPetId();

    // Initialize the list of Demand Goals
    this->initDemandGoalList(server);

    // Reset the seed for pseudo-random numbers
    srand(time(0));

    // Initialize ASW
//    AtomSpaceWrapper* asw = ASW(server().getAtomSpace());
//#if LOCAL_ATW
//    ((LocalATW*)asw)->SetCapacity(10000);
//#endif  
//    asw->archiveTheorems = false;
//    asw->allowFWVarsInAtomSpace = 
//        config().get_bool("PLN_FW_VARS_IN_ATOMSPACE");

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiActionSelectionAgent::initDemandGoalList(opencog::CogServer * server)
{
    logger().debug(
            "PsiActionSelectionAgent::%s - Initialize the list of Demand Goals (Final Goals) [ cycle =%d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    const AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Clear old demandGoalList;
    this->demandGoalList.clear();

    // Get demand names from the configuration file
    std::string demandNames = config()["PSI_DEMANDS"];

    // Process Demands one by one
    boost::tokenizer<> demandNamesTok (demandNames);
    std::string demand;
    Handle simultaneousEquivalenceLink, evaluationLinkDemandGoal;

    for ( boost::tokenizer<>::iterator iDemandName = demandNamesTok.begin();
          iDemandName != demandNamesTok.end();
          iDemandName ++ ) {

        demand = (*iDemandName);

        // Search the corresponding SimultaneousEquivalenceLink
        simultaneousEquivalenceLink =  AtomSpaceUtil::getDemandSimultaneousEquivalenceLink
                                           ( atomSpace, 
                                             demand,
                                             petId
                                           );

        if ( simultaneousEquivalenceLink == Handle::UNDEFINED )
        {
            logger().warn( "PsiActionSelectionAgent::%s - Failed to get the SimultaneousEquivalenceLink for demand '%s'",
                           __FUNCTION__, 
                           demand.c_str()
                         );

            continue;
        }

        // Get the Handle to EvaluationLinkDemandGoal
        //
        // Since SimultaneousEquivalenceLink inherits from UnorderedLink,
        // we should make a choice

        Handle firstEvaluationLink = atomSpace.getOutgoing(simultaneousEquivalenceLink, 0);

        if ( atomSpace.getType( 
                                  atomSpace.getOutgoing(firstEvaluationLink, 0) 
                              ) ==  PREDICATE_NODE ) {
            evaluationLinkDemandGoal = atomSpace.getOutgoing(simultaneousEquivalenceLink, 0);
        }
        else {
            evaluationLinkDemandGoal = atomSpace.getOutgoing(simultaneousEquivalenceLink, 1);
        }// if

        // Append the Demand Goal to demandGoalList
        this->demandGoalList.push_back(evaluationLinkDemandGoal);

        logger().debug(
                        "PsiActionSelectionAgent::%s - Add demand '%s' to demandGoalList successfully.", 
                        __FUNCTION__, 
                        demand.c_str() 
                      );
    }// for
}

Handle PsiActionSelectionAgent::chooseRandomDemandGoal() 
{
    std::vector<Handle>::iterator iDemandGoalList = this->demandGoalList.begin();
    iDemandGoalList += rand()%this->demandGoalList.size();
    return (*iDemandGoalList);
}
 
Handle PsiActionSelectionAgent::chooseMostCriticalDemandGoal(const AtomSpace & atomSpace)
{
    std::vector<Handle>::iterator iDemandGoalList, iCritialDemandGoal;
   
    // Pick up the Demand Goal with minimum truth value
    for( iDemandGoalList = this->demandGoalList.begin(), iCritialDemandGoal = this->demandGoalList.begin(); 
         iDemandGoalList != this->demandGoalList.end();
         iDemandGoalList ++ ) {

        if ( atomSpace.getTV(*iDemandGoalList).getMean() < atomSpace.getTV(*iCritialDemandGoal).getMean() ) {
            iCritialDemandGoal = iDemandGoalList;
        }
    }// for

    return (*iCritialDemandGoal);
}

const std::set<VtreeProvider*> & PsiActionSelectionAgent::searchBackward(Handle goalHandle, int & steps) 
{
    AtomSpace & atomSpace = * ( server().getAtomSpace() ); 
   
    // Create BITNodeRoot for the Goal (Target)
    pHandleSeq fakeHandles = ASW(&atomSpace)->realToFakeHandle(goalHandle);
    pHandle fakeHandle = fakeHandles[0];
   
std::cout<<"Initialize ASW OK"<<std::endl;

    Btr<vtree> target_(new vtree(fakeHandle));
    
    // The BIT uses real Links as a cue that it has already found the link,
    // so it is necessary to make them virtual
    Btr<vtree> target = ForceAllLinksVirtual(target_);

    // TODO: Do we have to use PLN_RECORD_TRAILS?
    bool recordingTrails = config().get_bool("PLN_RECORD_TRAILS");
    
    Bstate.reset(new BITNodeRoot(target, 
                                 new DefaultVariableRuleProvider,
                                 recordingTrails, 
                                 getFitnessEvaluator(PLN_FITNESS_BEST)
                                )
                );

    logger().debug("PsiActionSelectionAgent::%s - BITNodeRoot init ok.",  __FUNCTION__);

    // TODO: What if something bad happen while initialize BITNodeRoot?

    // Do inference backward
    BITNodeRoot * state = Bstate.get();
    state->setLoosePoolPolicy(true);

    const std::set<VtreeProvider *> & result = state->infer(steps,     // proof resources
                                                            0.000001f, // minimum confidence for storage
                                                            0.01f      // minimum confidence for abort
                                                           ); 

    logger().debug(
        "PsiActionSelectionAgent::%s - Got %d results (trees) after doing backward inference starting from demand %s", 
                    __FUNCTION__, 
                    result.size(), 
                    atomSpace.getName( atomSpace.getOutgoing(goalHandle, 0)
                                     ).c_str() 
                  );

    return result;
}

bool PsiActionSelectionAgent::extractPsiRules(const std::set<VtreeProvider *> & inferResult, 
                                              std::vector< std::vector<Handle> > & psiRulesList)
{
    // Make sure the result is not empty
    if ( inferResult.empty() ) {
        logger().warn("PsiActionSelectionAgent::%s - The result of backward searching is empty.",
                       __FUNCTION__
                     );

        return false; 
    }

    // Clear old Psi rules
    psiRulesList.clear();
    psiRulesList.resize(inferResult.size());

    // Process the tree one by one
    std::vector< std::vector<Handle> >::iterator iPsiRules = psiRulesList.begin();

    foreach(VtreeProvider * vtp, inferResult) {

        // Get the pHandle to Goal (i.e. the root node of the tree)
        pHandle goalpHandle = _v2h( *(vtp->getVtree().begin()) );

        this->extractPsiRules(goalpHandle, *iPsiRules, 0);        
       
        iPsiRules++;
    }// foreach

    return true;
}

void PsiActionSelectionAgent::extractPsiRules(pHandle ph, std::vector<Handle> & psiRules, unsigned int level)
{
    AtomSpaceWrapper *asw = GET_ASW;

    if ( ph == PHANDLE_UNDEFINED || asw->isType(ph) ) {
        logger().warn("PsiActionSelectionAgent::%s - Trying to extract Psi rules from NULL/ Virtual atom.",
                       __FUNCTION__
                     );
        return;
    }

    if (level > 20) {
        logger().warn("PsiActionSelectionAgent::%s - Maximum recursion depth exceeded.",
                       __FUNCTION__
                     );
    	return; 
    }

    // TODO: add h psiRulesList if h is a Psi Rule
    vhpair vhp = asw->fakeToRealHandle(ph);
    Handle realHandle = vhp.first; 

    if ( this->isHandleToPsiRule(realHandle) ) 
        psiRules.push_back(realHandle);

    // Extract Psi rules from its children nodes
    std::map<pHandle,vector<pHandle> >::const_iterator ph_it = haxx::get_inferred_from().find(ph);

    if( ph_it == haxx::get_inferred_from().end() ) 
        return; 

    foreach(pHandle arg_ph, ph_it->second)
        this->extractPsiRules(arg_ph, psiRules, level+1);
}

bool PsiActionSelectionAgent::isHandleToPsiRule(Handle h)
{
// Format of Psi rule    
//
// PredictiveImplicationLink
//     AndLink
//         AndLink
//             EvaluationLink
//                 GroundedPredicateNode "precondition_1_name"
//                 ListLink
//                     Node:arguments
//                     ...
//             EvaluationLink
//                 PredicateNode         "precondition_2_name"
//                 ListLink 
//                     Node:arguments
//                     ...
//             ...
//                        
//         ExecutionLink
//             GroundedSchemaNode "schema_name"
//             ListLink
//                 Node:arguments
//                 ...
//
//     EvaluationLink
//         (SimpleTruthValue indicates how well the demand is satisfied)
//         (ShortTermInportance indicates the urgency of the demand)
//         PredicateNode: "goal_name" 
//         ListLink
//             Node:arguments
//             ...
//

    const AtomSpace & atomSpace = this->getAtomSpace();

    // Check h
    if ( atomSpace.getType(h) != PREDICTIVE_IMPLICATION_LINK || atomSpace.getArity(h) != 2 )
        return false; 

    // Get handles to premise and goal
    Handle andLinkPreconditionAction = atomSpace.getOutgoing(h, 0);
    Handle evaluationLinkGoal = atomSpace.getOutgoing(h, 1);

    // Check premise
    if ( atomSpace.getType(andLinkPreconditionAction) != AND_LINK || atomSpace.getArity(h) != 2 )
        return false;

    // Get handle to action
    Handle executionLinkAction = atomSpace.getType( 
                                                      atomSpace.getOutgoing(andLinkPreconditionAction, 0) 
                                                  ) == EXECUTION_LINK ? 
                                 atomSpace.getOutgoing(andLinkPreconditionAction, 0) :
                                 atomSpace.getOutgoing(andLinkPreconditionAction, 1);

    // Check action
    if ( atomSpace.getType(executionLinkAction) != EXECUTION_LINK ||
         atomSpace.getArity(executionLinkAction) != 2 ||
         atomSpace.getType( atomSpace.getOutgoing(executionLinkAction, 0) ) != GROUNDED_PREDICATE_NODE
       )
        return false; 

    // Check goal
    if ( atomSpace.getType(evaluationLinkGoal) != EVALUATION_LINK ||
         atomSpace.getArity(evaluationLinkGoal) != 2 ||
         atomSpace.getType( atomSpace.getOutgoing(evaluationLinkGoal, 0) ) != PREDICATE_NODE 
       )
        return false; 

    // Now we are pretty sure it(h) is a Psi rule
    return true;
}

void PsiActionSelectionAgent::run(opencog::CogServer * server)
{
    this->cycleCount ++;

    logger().debug( "PsiActionSelectionAgent::%s - Executing run %d times",
                     __FUNCTION__, 
                     this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Check if map info data is available
    if ( atomSpace.getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn( 
      "PsiActionSelectionAgent::%s - There is no map info available yet [ cycle = %d ]", 
                        __FUNCTION__, 
                        this->cycleCount
                     );
        return;
    }

    // Check if the pet spatial info is already received
    if ( !atomSpace.getSpaceServer().getLatestMap().containsObject(petId) ) {
        logger().warn(
 "PsiActionSelectionAgent::%s - Pet was not inserted in the space map yet [ cycle = %d ]", 
                     __FUNCTION__, 
                     this->cycleCount
                     );
        return;
    }

    // Initialize the Agent (demandGoalList etc)
    if ( !this->bInitialized )
        this->init(server);

    // Choose a Demand Goal 
    Handle currentDemandGoal;

    // Set current Demand Goal to TestEnergyDemand for debugging
    std::vector<Handle>::iterator iDemandGoal; 
    Handle goalPredicateNode; 
    for (iDemandGoal=this->demandGoalList.begin(); iDemandGoal!=this->demandGoalList.end(); iDemandGoal++) {
        goalPredicateNode = atomSpace.getOutgoing(*iDemandGoal, 0);

        logger().debug(
                "PsiActionSelectionAgent::%s - Name of current searching PredicateNode for Demand is '%s'", 
                __FUNCTION__, 
                atomSpace.getName(goalPredicateNode).c_str()
                      );        

        if ( atomSpace.getName(goalPredicateNode) == "TestEnergyDemandGoal" )
            break;
    }

    if ( iDemandGoal == this->demandGoalList.end() ) {
        logger().error(
                "PsiActionSelectionAgent::%s - Can not find TestEnergyDemandGoal for debugging [ cycle = %d ]", 
                __FUNCTION__, 
                this->cycleCount
                      );
        return; 
    }
    else {
        currentDemandGoal = *iDemandGoal; 
    }

    // Do backward inference starting from the selected Demand Goal using PLN
    int steps = 2000;
    const std::set<VtreeProvider *> & inferResult = this->searchBackward(currentDemandGoal, steps);


}

