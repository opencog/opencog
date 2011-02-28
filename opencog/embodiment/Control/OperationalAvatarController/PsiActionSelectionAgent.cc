/**
 * TODO: 1. Deal with the truth value of Action, i.e. ExecutionLink 
 */
/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-02-04
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

extern int currentDebugLevel;

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
    logger().debug( "PsiActionSelectionAgent::%s - Initializing the Agent [ cycle = %d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
//    OAC * oac = (OAC *) server;

    // Get Procedure repository
//    const Procedure::ProcedureRepository & procedureRepository = 
//                                               oac->getProcedureRepository();

    // Get petId
//    const std::string & petId = oac->getPet().getPetId();

    // Initialize the list of Demand Goals
    this->initDemandGoalList(server);

    // Reset the seed for pseudo-random numbers
    srand(time(0));

    // Initialize ASW etc.
    // TODO: Shall we have to to do so? 
    AtomSpaceWrapper* asw = ASW(opencog::server().getAtomSpace());
#if LOCAL_ATW
    ((LocalATW*)asw)->SetCapacity(10000);
#endif  
    asw->archiveTheorems = false;
    asw->allowFWVarsInAtomSpace = 
    config().get_bool("PLN_FW_VARS_IN_ATOMSPACE");

    currentDebugLevel = config().get_int("PLN_LOG_LEVEL");

    // Initialize other members
    this->currentPsiRule = opencog::Handle::UNDEFINED; 
    this->previousPsiRule = opencog::Handle::UNDEFINED;
    this->currentSchemaId = 0;
    this->procedureExecutionTimeout = config().get_long("PROCEDURE_EXECUTION_TIMEOUT");

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiActionSelectionAgent::initDemandGoalList(opencog::CogServer * server)
{
    logger().debug(
            "PsiActionSelectionAgent::%s - Initializing the list of Demand Goals (Final Goals) [ cycle =%d ]",
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
    if ( this->demandGoalList.empty() ) {
        logger().warn( "PsiActionSelectionAgent::%s - Failed to choose a Demand Goal randomly. Because demandGoalList is empty [ cycle = %d ]. ", 
                        __FUNCTION__, 
                        this->cycleCount
                      );

        return opencog::Handle::UNDEFINED;
    }

    std::vector<Handle>::iterator iDemandGoalList = this->demandGoalList.begin();
    iDemandGoalList += rand()%this->demandGoalList.size();
    return (*iDemandGoalList);
}
 
Handle PsiActionSelectionAgent::chooseMostCriticalDemandGoal(opencog::CogServer * server)
{
    if ( this->demandGoalList.empty() ) {
        logger().warn( "PsiActionSelectionAgent::%s - Failed to choose the most critical Demand Goal randomly. Because demandGoalList is empty [ cycle = %d ]. ", 
                        __FUNCTION__, 
                        this->cycleCount
                      );

        return opencog::Handle::UNDEFINED;
    }

    const AtomSpace & atomSpace = * ( server->getAtomSpace() );

    std::vector<Handle>::iterator iDemandGoalList, iCritialDemandGoal;
   
    // Pick up the Demand Goal with minimum truth value
    for( iDemandGoalList = this->demandGoalList.begin(), iCritialDemandGoal = this->demandGoalList.begin(); 
         iDemandGoalList != this->demandGoalList.end();
         iDemandGoalList ++ ) {

        if ( atomSpace.getTV(*iDemandGoalList)->getMean() < atomSpace.getTV(*iCritialDemandGoal)->getMean() ) {
            iCritialDemandGoal = iDemandGoalList;
        }
    }// for

    return (*iCritialDemandGoal);
}

const std::set<VtreeProvider*> & PsiActionSelectionAgent::searchBackward(Handle goalHandle, int & steps) 
{
    // The implementation below borrowed many source code from the function opencog::pln::infer ("PLNModule.cc")

    AtomSpace & atomSpace = * ( server().getAtomSpace() ); 
   
    // Create BITNodeRoot for the Goal (Target)
    pHandleSeq fakeHandles = ASW()->realToFakeHandle(goalHandle);

    pHandle fakeHandle = fakeHandles[0];

    logger().debug("PsiActionSelectionAgent::%s - Initialize ASW OK [ cycle = %d ]", 
                   __FUNCTION__, 
                   this->cycleCount
                  );

    Btr<vtree> target_(new vtree(fakeHandle));
    
    // The BIT uses real Links as a cue that it has already found the link,
    // so it is necessary to make them virtual
    Btr<vtree> target = ForceAllLinksVirtual(target_);

    // TODO: Do we have to use PLN_RECORD_TRAILS?
    //       Actually, we should always set it true currently,
    //       because 'extractPsiRules' relies on the trails information. 
    bool recordingTrails = config().get_bool("PLN_RECORD_TRAILS");
    
    Bstate.reset(new BITNodeRoot(target, 
                                 &referenceRuleProvider(),
                                 recordingTrails, 
                                 getFitnessEvaluator(PLN_FITNESS_BEST)
                                )
                );

    logger().debug("PsiActionSelectionAgent::%s - BITNodeRoot init ok [ cycle = %d ].", 
                   __FUNCTION__,           
                   this->cycleCount
                  );

    // TODO: What if something bad happen while initialize BITNodeRoot?

    // Do inference backward
    BITNodeRoot * state = Bstate.get();
    state->setLoosePoolPolicy(true);

    // TODO: How to forbid the output to the screen during inference?
    //       We don't want to mess up the screen with too much details of PLN inference. 
    const std::set<VtreeProvider *> & result = state->infer(steps,     // proof resources
                                                            0.000001f, // minimum confidence for storage
                                                            0.01f      // minimum confidence for abort
                                                           ); 

    // Print the results to the screen for debugging. 
    state->printResults();

    logger().debug(
        "PsiActionSelectionAgent::%s - Got %d results (trees) after doing backward inference starting from demand %s, left %d steps [ cycle = %d ].", 
                    __FUNCTION__, 
                    result.size(), 
                    atomSpace.getName( atomSpace.getOutgoing(goalHandle, 0)
                                     ).c_str() ,
                    steps, 
                    this->cycleCount
                  );

    return result;
}

bool PsiActionSelectionAgent::extractPsiRules(opencog::CogServer * server, 
                                              const std::set<VtreeProvider *> & inferResult, 
                                              std::vector< std::vector<Handle> > & psiRulesList)
{
    // Make sure the result is not empty
    if ( inferResult.empty() ) {
        logger().warn("PsiActionSelectionAgent::%s - The result of backward searching is empty [ cycle = %d ] .",
                       __FUNCTION__, 
                       this->cycleCount
                     );

        return false; 
    }

    // Clear old Psi rules
    psiRulesList.clear();

    psiRulesList.resize(inferResult.size());
    int planNo = 1; 

    // Process the tree one by one
    std::vector< std::vector<Handle> >::iterator iPsiRules = psiRulesList.begin();

    foreach(VtreeProvider * vtp, inferResult) {

        logger().debug("PsiActionSelectionAgent::%s - Going to extract Psi Rules for plan No.%d [ cycle = %d ].", 
                       __FUNCTION__, 
                       planNo, 
                       this->cycleCount
                      );

        // Get the pHandle to Goal (i.e. the root node of the tree)
        pHandle goalpHandle = _v2h( *(vtp->getVtree().begin()) );

        // Extract Psi Rules
        this->extractPsiRules(server, goalpHandle, *iPsiRules, 0);        

        logger().debug(
            "PsiActionSelectionAgent::%s - Extract Psi Rules for plan No.%d done. Found %d Psi Rules [ cycle = %d ] .",
                       __FUNCTION__, 
                       planNo, 
                       iPsiRules->size(), 
                       this->cycleCount
                      );

        iPsiRules ++;
        planNo ++; 

    }// foreach

    return true;
}

bool PsiActionSelectionAgent::planByPLN( opencog::CogServer * server,
                                         Handle goalHandle,
                                         std::vector< std::vector<Handle> > & psiPlanList,
                                         int & steps )
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get the name of the goal
    std::string goalName = atomSpace.getName( atomSpace.getOutgoing(goalHandle, 0) );

    logger().debug(
            "PsiActionSelectionAgent::%s - Do action planning starting from the Demand Goal '%s' [ cycle = %d ].", 
                    __FUNCTION__, 
                    goalName.c_str(), 
                    this->cycleCount
                  );

    // Do backward inference starting from the selected Demand Goal using PLN
    const std::set<VtreeProvider *> & inferResult = this->searchBackward(goalHandle, steps);

    if ( inferResult.empty() ) {
        logger().warn( 
                "PsiActionSelectionAgent::%s - Got empty result while searching backward via PLN [ cycle = %d ]", 
                     __FUNCTION__, 
                     this->cycleCount
                    );
        return false; 
    }

    // Extract Psi Rules from trees returned by PLN
    if ( !this->extractPsiRules(server, inferResult, psiPlanList) ) {
        logger().warn(
              "PsiActionSelectionAgent::%s - Failed to extract Psi rules from the tree returned by PLN [ cycle = %d ]", 
                    __FUNCTION__, 
                    this->cycleCount
                   );
        return false; 
    }

    // Reset all the truth value of subgoals to false
    this->resetPlans(server, goalHandle, psiPlanList);

    // TODO: only for debugging, comment it later 
    // Print plans
    this->printPlans(server, goalHandle, psiPlanList);

    logger().debug( "PsiActionSelectionAgent::%s - Figure out %d plans for the goal '%s' successfully [ cycle = %d ]", 
                    __FUNCTION__, 
                    psiPlanList.size(), 
                    goalName.c_str(), 
                    this->cycleCount
                  );

    return true; 
}

bool PsiActionSelectionAgent::planByNaiveBreadthFirst( opencog::CogServer * server, 
                                                       Handle goalHandle, 
                                                       std::vector< std::vector<Handle> > & psiPlanList,
                                                       int & steps )
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get rand generator
    RandGen & randGen = oac->getRandGen();

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get the name of the goal
    std::string goalName = atomSpace.getName( atomSpace.getOutgoing(goalHandle, 0) );

    logger().debug(
            "PsiActionSelectionAgent::%s - Do action planning starting from the Demand Goal '%s' [ cycle = %d ].", 
                    __FUNCTION__, 
                    goalName.c_str(), 
                    this->cycleCount
                  );

    // Clear old Plans
    psiPlanList.clear();
    psiPlanList.resize(1);

    // Push the ultimate Goal to the Open List
    std::list<Handle> openList; 
    std::vector<Handle> closeList; 

    openList.push_back(goalHandle);

    while ( !openList.empty() && steps>0 ) {

	steps --;

        // Pop up a Goal from Open List, push it to Close List, and set it as the current goal
        Handle hCurrentGoal = openList.front();
        openList.pop_front();    
        closeList.push_back(hCurrentGoal);

        Handle hCurrentGoalNode = atomSpace.getOutgoing(hCurrentGoal, 0);
        std::string currentGoalName = atomSpace.getName(hCurrentGoalNode);

        logger().debug( "PsiActionSelectionAgent::%s - Going to figure out how to reach the goal '%s'. [ cycle = %d ]", 
                        __FUNCTION__, 
                        currentGoalName.c_str(), 
                        this->cycleCount
                      );

        // Get all the Psi Rules that would lead to the current goal 
        std::vector<Handle> implicationLinkSet;
        std::vector<Handle> psiRuleCandidates;
        atomSpace.getHandleSet( back_inserter(implicationLinkSet), hCurrentGoal, IMPLICATION_LINK, false );

        foreach(Handle hImplicationLink, implicationLinkSet) {
            if ( AtomSpaceUtil::isHandleToPsiRule(atomSpace, hImplicationLink) )
                psiRuleCandidates.push_back(hImplicationLink);
        }

        if ( psiRuleCandidates.empty() ) {
            logger().warn( "PsiActionSelectionAgent::%s - There's no Psi Rule that would lead to the goal '%s' [ cycle = %d]", 
                           __FUNCTION__, 
                           currentGoalName.c_str(), 
                           this->cycleCount
                         );
            continue; 
        }

        // Randomly select one Psi Rule for current goal
        int selectedIndex = randGen.randint( psiRuleCandidates.size() );
        Handle hSelectedPsiRule = psiRuleCandidates[selectedIndex];

        logger().debug( "PsiActionSelectionAgent::%s - Randomly select a Psi Rule: %s [ cycle = %d ]", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(hSelectedPsiRule).c_str()
                      );

        // Split the Psi Rule into Goal, Action and Preconditions
        Handle hGoalEvaluationLink, hActionExecutionLink, hPreconditionAndLink; 
        AtomSpaceUtil::splitPsiRule( atomSpace,
                                     hSelectedPsiRule, 
                                     hGoalEvaluationLink, 
                                     hActionExecutionLink,
                                     hPreconditionAndLink
                                   );

        // Store those Preconditions, i.e. subgoals, to Open List if they are not GroundedPredicateNode and 
        // do no exist in both Open and Close List
        foreach( Handle hPrecondition, atomSpace.getOutgoing(hPreconditionAndLink) ) {

           logger().debug("PsiActionSelectionAgent::%s - Going to check the precondition: %s [ cycle = %d ]", 
			   __FUNCTION__, 
			   atomSpace.atomAsString(hPrecondition).c_str(), 
			   this->cycleCount
			 );
         		
            Handle hPreconditionNode = atomSpace.getOutgoing(hPrecondition, 0);

 	    logger().debug("PsiActionSelectionAgent::%s - Get hPreconditionNode: %s [ cycle = %d ]", 
			    __FUNCTION__, 
			    atomSpace.atomAsString(hPreconditionNode).c_str(), 
			    this->cycleCount
                          );

            Type preconditionType = atomSpace.getType(hPreconditionNode);

	    logger().debug("PsiActionSelectionAgent::%s - Get hPreconditionNode type : %s [ cycle = %d ]", 
			    __FUNCTION__, 
			    classserver().getTypeName(preconditionType).c_str(), 
			    this->cycleCount
			  );

            if ( preconditionType != GROUNDED_PREDICATE_NODE &&
                 std::find(openList.begin(), openList.end(), hPrecondition) == openList.end() &&
                 std::find(closeList.begin(), closeList.end(), hPrecondition) == closeList.end()
               ) {
                openList.push_back(hPrecondition); 
		logger().debug("PsiActionSelectionAgent::%s - Append %s to Open List [ cycle = %d ]", 
				__FUNCTION__, 
				atomSpace.atomAsString(hPrecondition).c_str(), 
				this->cycleCount
			      );
            }// if

	    logger().debug("PsiActionSelectionAgent::%s - It has not been added to Open List [ cycle = %d ]", 
			    __FUNCTION__, 
			    this->cycleCount
			  );

        }// foreach

        // Store the Psi Rule
 	logger().debug("PsiActionSelectionAgent::%s - Stored an Psi Rule: %s [ cycle = %d ]", 
			__FUNCTION__, 
			atomSpace.atomAsString(hSelectedPsiRule).c_str(), 
			this->cycleCount
		      );
        psiPlanList[0].push_back(hSelectedPsiRule);
    }// while

    logger().debug( "PsiActionSelectionAgent::%s - Figure out %d plans for the goal '%s' successfully [ cycle = %d ]", 
                    __FUNCTION__, 
                    psiPlanList.size(), 
                    goalName.c_str(), 
                    this->cycleCount
                  );

    // Reset all the truth value of subgoals to false
    this->resetPlans(server, goalHandle, psiPlanList);

    logger().debug( "PsiActionSelectionAgent::%s - Reset plans successfully [ cycle = %d]", 
		    __FUNCTION__, 
		    this->cycleCount
		  );

    // TODO: only for debugging, comment it later 
    // Print plans
    this->printPlans(server, goalHandle, psiPlanList);

    return true; 
}

void PsiActionSelectionAgent::extractPsiRules(opencog::CogServer * server, pHandle ph,
                                              std::vector<Handle> & psiRules, unsigned int level)
{
    AtomSpace & atomSpace = * ( server->getAtomSpace() ); 
   
    AtomSpaceWrapper *asw = GET_ASW;

    if ( ph == PHANDLE_UNDEFINED || asw->isType(ph) ) {
        logger().warn( "PsiActionSelectionAgent::%s - Trying to extract Psi rules from NULL/ Virtual atom [ level = %d, cycle = %d].",
                       __FUNCTION__, 
                       level, 
                       this->cycleCount
                     );
        return;
    }

    if (level > 20) {
        logger().warn("PsiActionSelectionAgent::%s - Maximum recursion depth exceeded [ level = %d, cycle = %d ].",
                       __FUNCTION__, 
                       level, 
                       this->cycleCount
                     );
    	return; 
    }

    // Append the given handle (its corresponding real handle) to psiRulesList if it is a Psi Rule
    vhpair vhp = asw->fakeToRealHandle(ph);
    Handle realHandle = vhp.first; 

    if ( AtomSpaceUtil::isHandleToPsiRule(atomSpace, realHandle) ) { 

        logger().debug("PsiActionSelectionAgent::%s - Found a Psi Rule: '%s' [ level = %d, cycle = %d ].",
                       __FUNCTION__, 
                       atomSpace.atomAsString(realHandle).c_str(),
                       level, 
                       this->cycleCount
                      );
       
        psiRules.push_back(realHandle);
    }        

    // Extract Psi rules from its children nodes
    std::map<pHandle, RulePtr>::const_iterator pln_rule_it = haxx::get_inferred_with().find(ph); 
    std::map<pHandle, vector<pHandle> >::const_iterator ph_it = haxx::get_inferred_from().find(ph);

    logger().debug("PsiActionSelectionAgent::%s - Size of trail for all the trees is %d [ level = %d, cycle = %d ].",
                   __FUNCTION__, 
                   haxx::get_inferred_from().size(), 
                   level, 
                   this->cycleCount
                  );

    if( pln_rule_it == haxx::get_inferred_with().end() || 
        ph_it == haxx::get_inferred_from().end() ) 
        return; 

    logger().debug("PsiActionSelectionAgent::%s - Size of trail for this Atom is %d [ level = %d, cycle = %d ].",
                   __FUNCTION__, 
                   ph_it->second.size(), 
                   level, 
                   this->cycleCount
                  );
   

    foreach(pHandle arg_ph, ph_it->second) {

        logger().debug(
                "PsiActionSelectionAgent::%s - Go one depth further to check Atom: '%s' [ level = %d, cycle = %d ].",
                       __FUNCTION__, 
                       atomSpace.atomAsString(asw->fakeToRealHandle(arg_ph).first
                                             ).c_str(),
                       level, 
                       this->cycleCount
                      );

        this->extractPsiRules(server, arg_ph, psiRules, level+1);

    }// foreach 
}

void PsiActionSelectionAgent::printPlans(opencog::CogServer * server, Handle hDemandGoal, 
                                         const std::vector< std::vector<Handle> > & psiRulesLists)
{
    const AtomSpace & atomSpace = * ( server->getAtomSpace() ); 

    std::cout<<"Found "<<psiRulesLists.size()<<" plans for Demand Goal: "
             <<atomSpace.getName( atomSpace.getOutgoing(hDemandGoal, 0)
                                ).c_str()
             <<" [ cycle = "<<this->cycleCount<< " ]"
             <<std::endl;

    int planNo=1;

    foreach(std::vector<Handle> psiRules, psiRulesLists) {

        std::cout<<std::endl<<"Plan No."<<planNo<<" contains "<<psiRules.size()<<" Actions:"
                <<std::endl
                <<"DemandGoal ";

        foreach(Handle h, psiRules) {
            // Get handles to premise and goal
            Handle andLinkPreconditionAction = atomSpace.getOutgoing(h, 0);

            // Get handle to execution link
            Handle executionLinkAction = atomSpace.getType( 
                                                              atomSpace.getOutgoing(andLinkPreconditionAction, 0) 
                                                          ) == EXECUTION_LINK ? 
                                         atomSpace.getOutgoing(andLinkPreconditionAction, 0) :
                                         atomSpace.getOutgoing(andLinkPreconditionAction, 1);

            // Get handle to action 
            Handle action = atomSpace.getOutgoing(executionLinkAction, 0); 

            // Print the action
            std::cout<<" <== "<<atomSpace.getName(action);

        }// foreach

        std::cout<<std::endl; 

        planNo++;   

    }// foreach
}

void PsiActionSelectionAgent::resetPlans( opencog::CogServer * server,
                                          Handle hDemandGoal, 
                                          std::vector< std::vector<Handle> > & psiPlanList)
{
    // Get AtomSpace
    AtomSpace & atomSpace = * ( server->getAtomSpace() ); 

    Handle hGoalEvaluationLink, hActionExecutionLink, hPreconditionAndLink; 

    // Process each Plan
    foreach(std::vector<Handle> psiPlan, psiPlanList) {

        // Process each Psi Rule within a specific Plan
        foreach(Handle hPsiRule, psiPlan) {

            // Split the Psi Rule into Goal, Action and Preconditions
            if ( !AtomSpaceUtil::splitPsiRule(atomSpace, 
                                              hPsiRule, 
                                              hGoalEvaluationLink, 
                                              hActionExecutionLink,
                                              hPreconditionAndLink
                                             ) ) {
                logger().warn( "PsiActionSelectionAgent::%s - Failed to split the Psi Rule, %s [ cycle = %d ]", 
                               __FUNCTION__, 
                               atomSpace.atomAsString(hPsiRule).c_str(), 
                               this->cycleCount
                              );

                continue; 
            }// if 

            // Set the truth value of the subgoal to false 
            //
            // Note: we'll skip GroundedPredicateNode and Demald Goal.
            //       Because the TruthValue of a subgoal with GroundedPredicateNode 
            //       is achieved by running combo script, and 
            //       the TruthValue of a DemandGoal is handled by 'PsiDemandUpdaterAgent'
            //
            Type goalType = atomSpace.getType( atomSpace.getOutgoing(hGoalEvaluationLink, 0) );

            SimpleTruthValue stvFalse(0.000001, 1.0);

            if ( goalType != GROUNDED_PREDICATE_NODE &&
                 hGoalEvaluationLink != hDemandGoal ) {
                atomSpace.setTV(hGoalEvaluationLink, stvFalse);
            }

        }// foreach

    }// foreach
}

bool PsiActionSelectionAgent::getSchemaArguments(opencog::CogServer * server,
                                                 Handle hListLink, 
                                                 const std::vector<std::string> & varBindCandidates, 
                                                 std::vector <combo::vertex> & schemaArguments) 
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get rand generator
    RandGen & randGen = oac->getRandGen();

    // Get AtomSpace
    AtomSpace & atomSpace = * ( server->getAtomSpace() );

    // Check hListLink is of type ListLink
    if ( atomSpace.getType(hListLink) != LIST_LINK ) {
        logger().error(
                "PsiActionSelectionAgent::%s - Link containing combo arguments should be of type ListLink. Got '%s'", 
                         __FUNCTION__,
                         classserver().getTypeName( atomSpace.getType(hListLink)
                                                  ).c_str() 
                      );
        return 0;
    }

    // Randomly choose a variable binding from varBindCandidates for all the VariableNodes within hListLink
    //
    // TODO: Strictly speaking, the implementation below is not so correct, 
    //       because it will replace all the VarialbeNode within hListLink with the same variable binding 
    //       [By Zhennua Cai, on 2011-02-14]
    //
    bool bChooseVariableBind = false; 
    std::string variableBind;
    int indexVariableBind; 

    if ( !varBindCandidates.empty() ) {
        indexVariableBind = randGen.randint( varBindCandidates.size() ); 
        variableBind = varBindCandidates[indexVariableBind];
        bChooseVariableBind = true; 
    }

    // Clear old schema arguments
    schemaArguments.clear(); 

    // Process the arguments according to its type
    foreach( Handle  hArgument, atomSpace.getOutgoing(hListLink) ) {

        Type argumentType = atomSpace.getType(hArgument);

        if (argumentType == NUMBER_NODE) {
            schemaArguments.push_back(combo::contin_t(
                                          boost::lexical_cast<combo::contin_t>(atomSpace.getName(hArgument)
                                                                              )
                                                     ) 
                                     );
        }
        else if (argumentType == VARIABLE_NODE) {
            schemaArguments.push_back(combo::contin_t(
                                          boost::lexical_cast<combo::contin_t>(atomSpace.getName(hArgument)
                                                                              )
                                                     ) 
                                     );

            if ( bChooseVariableBind ) {
                schemaArguments.push_back(variableBind);  
            }
            else {
                logger().error( "PsiActionSelectionAgent::%s - Failed to find any valid variable binding for VarialbeNode '%s' [ cycle = %d ]", 
                                __FUNCTION__, 
                                atomSpace.getName(hArgument).c_str(), 
                                this->cycleCount
                              );
                return false; 
            }
        }
        else {
            schemaArguments.push_back( atomSpace.getName(hArgument) );
        }
    }// foreach

    return true; 
}

void PsiActionSelectionAgent::initVarBindCandidates(opencog::CogServer * server, 
                                                    std::vector<std::string> & varBindCandidates)
{
    varBindCandidates.clear(); 

    const AtomSpace & atomSpace = * ( server->getAtomSpace() );
    const SpaceServer::SpaceMap& spaceMap = atomSpace.getSpaceServer().getLatestMap(); 

    spaceMap.findAllEntities( back_inserter(varBindCandidates) );
}

void PsiActionSelectionAgent::initUnifier(combo::variable_unifier & unifier,
                                          const std::vector<std::string> & varBindCandidates)
{
    unifier.clear(); 

    foreach(std::string varBind, varBindCandidates) {
        unifier.insert(varBind, 
                       true  // Each variable binding is considered as valid initially. 
                             // Then the combo procedure would update its state (valid/invalid)
                             // after running the procedure. 
                      );
    }
} 

void PsiActionSelectionAgent::updateVarBindCandidates(const combo::variable_unifier & unifier, 
                                                      std::vector<std::string> & varBindCandidates)
{
    varBindCandidates.clear();

    if ( !unifier.isUpdated() ) {
        return;
    }

    std::map<std::string, bool>::const_iterator iVarBind;

    for ( iVarBind = unifier.begin(); iVarBind != unifier.end(); iVarBind ++ ) {
        if ( iVarBind->second ) {
            varBindCandidates.push_back( iVarBind->first );

            logger().debug("PsiActionSelectionAgent::%s - Found a valid variable binding '%s' [ cycle = %d ].", 
                           __FUNCTION__, 
                           iVarBind->first.c_str(),
                           this->cycleCount
                          );
        }
    }
}

bool PsiActionSelectionAgent::isSatisfied(opencog::CogServer * server, 
                                          Handle hPrecondition, 
                                          combo::variable_unifier & unifier
                                         ) 
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get ProcedureInterpreter
    Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

    // Variables used by combo interpreter
    std::vector <combo::vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    combo::vertex result; // combo::vertex is actually of type boost::variant <...>

    // Check the input (hPrecondition)
    if ( atomSpace.getType(hPrecondition) != EVALUATION_LINK ||
         atomSpace.getArity(hPrecondition) != 2 ) {
        logger().warn( "PsiActionSelectionAgent::%s - Precondition should be of type EvaluationLink and with two arity. But got %s [ cycle = %d ]", 
                        __FUNCTION__,
                        atomSpace.atomAsString(hPrecondition).c_str(),
                        this->cycleCount
                     );

        return false;    
    }
       
    // Get the PredicateNode or GroundedPredicateNode inside hPrecondition
    Handle hNode = atomSpace.getOutgoing(hPrecondition, 0); 

    // Get Precondition name
    std::string preconditionName = atomSpace.getName(hNode);

    // For PredicateNode, simply check its truth value
    if ( atomSpace.getType(hNode) == PREDICATE_NODE ) {

        logger().debug("PsiActionSelectionAgent::%s - Got a PredicateNode: %s [ cycle = %d ]", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hNode).c_str(), 
                       this->cycleCount
                      );

        if ( atomSpace.getTV(hPrecondition)->getMean() >= 0.99 ) 
            return true; 
    }
    // For GroundedPredicateNode, run the corresponding combo script, and then analyze the result
    else if ( atomSpace.getType(hNode) == GROUNDED_PREDICATE_NODE ) {

        logger().debug("PsiActionSelectionAgent::%s - Got a GroundedPredicateNode: %s [ cycle = %d ]", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hNode).c_str(), 
                       this->cycleCount
                      );

        // Get schemaArguments
        Handle hListLink = atomSpace.getOutgoing(hPrecondition, 1); // Handle to ListLink containing arguments
        std::vector<std::string> emptyVarBindCandidates; 

        if ( !this->getSchemaArguments( server, hListLink, emptyVarBindCandidates, schemaArguments) )
            return false; 

        logger().debug("PsiActionSelectionAgent::%s - Got %d arguments [ cycle = %d ]", 
                       __FUNCTION__, 
                       schemaArguments.size(),
                       this->cycleCount
                      );

        // Run the Procedure of the Precondition
        // TODO: If the get method fails, what happens?
        const Procedure::GeneralProcedure & procedure = procedureRepository.get(preconditionName);

        executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments, unifier);

        // Wait until the combo script has been done
        while ( !procedureInterpreter.isFinished(executingSchemaId) )
            procedureInterpreter.run(NULL);  

        logger().debug("PsiActionSelectionAgent::%s - Finish executing combo procedure %s [ cycle = %d ]", 
                       __FUNCTION__, 
                       preconditionName.c_str(), 
                       this->cycleCount
                      );

        // Check if the the Procedure run successfully
        if ( procedureInterpreter.isFailed(executingSchemaId) ) {
            logger().warn( "PsiActionSelectionAgent::%s - Failed to execute '%s'", 
                             __FUNCTION__, 
                             preconditionName.c_str()
                          );

            return false; 
        }

        result = procedureInterpreter.getResult(executingSchemaId);

        logger().debug(
                "PsiActionSelectionAgent::%s - Got the result after executing combo procedure %s [ cycle = %d ]", 
                       __FUNCTION__, 
                       preconditionName.c_str(), 
                       this->cycleCount
                      );

        if ( result == combo::id::logical_true ) {
            logger().debug( "PsiActionSelectionAgent::%s - This Precondition '%s' is true. [ cycle = %d ]", 
			    __FUNCTION__, 
			    preconditionName.c_str(), 
			    this->cycleCount
			  );   		
            return true; 
	} 
        else {
            logger().debug( "PsiActionSelectionAgent::%s - This Precondition '%s' is false. [ cycle = %d ]", 
			    __FUNCTION__, 
			    preconditionName.c_str(), 
			    this->cycleCount
			  );   		
            return false; 
	}
    }
    // For other types of Atom, return false
    else {
        logger().warn( "PsiActionSelectionAgent::%s - First outgoing of Precondition should be of type PredicateNode or GroundedPredicateNode. But got type '%s' in '%s'. [ cycle = %d ]", 
                         __FUNCTION__,
                         classserver().getTypeName( atomSpace.getType(hNode)
                                                  ).c_str(), 
                         atomSpace.atomAsString(hPrecondition).c_str(),  
                         this->cycleCount
                     );

        return false;    
    }// if
}

Handle PsiActionSelectionAgent::pickUpPsiRule(opencog::CogServer * server,
                                              const std::vector<Handle> & psiRules,
                                              std::vector<std::string> & varBindCandidates) 
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    bool bAllPreconditionsSatisfied = true;
    Handle hSelectedPsiRule = opencog::Handle::UNDEFINED; 
    Handle hGoalEvaluationLink, hActionExecutionLink, hPreconditionAndLink;
    std::vector<Handle> hPreconditionEvalutaionLinks;  

    foreach(Handle hPsiRule, psiRules) {
        logger().debug( "PsiActionSelectionAgent::%s Going to check the Psi Rule: %s [ cycle = %d ]", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(hPsiRule).c_str(), 
                        this->cycleCount 
                      ); 

        // Get the Handles to Preconditions
        if ( !AtomSpaceUtil::splitPsiRule(atomSpace, 
                                          hPsiRule, 
                                          hGoalEvaluationLink, 
                                          hActionExecutionLink,
                                          hPreconditionAndLink
                                         ) ) 
            continue; 

        hPreconditionEvalutaionLinks = atomSpace.getOutgoing(hPreconditionAndLink);

        // Initialize the variable bindings with all the entities the pet encounters 
        this->initVarBindCandidates(server, varBindCandidates); 

        logger().debug( "PsiActionSelectionAgent::%s Initialize the variable bindings ( size = %d ) with all the entities the pet encounters [ cycle = %d ]", 
                        __FUNCTION__, 
                        varBindCandidates.size(), 
                        this->cycleCount 
                      ); 

        // Initialize the unifier used by combo interpreter
        //
        // Initially all the entities the pet encounters are considered as valid variable bindings. 
        // Then the combo interpreter would update the states (valid/ invalid) of each binding.
        combo::variable_unifier unifier; 
        this->initUnifier(unifier, varBindCandidates);

        logger().debug( "PsiActionSelectionAgent::%s Initialize the unifier ( size = %d ) [ cycle = %d ]", 
                        __FUNCTION__, 
                        unifier.size(), 
                        this->cycleCount 
                      ); 

        // Check Preconditions one by one
        foreach( Handle hPrecondition, hPreconditionEvalutaionLinks ) {
            logger().debug( "PsiActionSelectionAgent::%s - Going to check the Precondition: %s [ cycle = %d ]", 
                            __FUNCTION__, 
                            atomSpace.atomAsString(hPrecondition).c_str(), 
                            this->cycleCount
                          );

            if ( !isSatisfied(server, hPrecondition, unifier) ) {
                bAllPreconditionsSatisfied = false;
                break; 
            }
        }// foreach
       
        if (bAllPreconditionsSatisfied) {

            // Record the Psi Rule that all its Preconditions are satisfied
            hSelectedPsiRule = hPsiRule;  

            // Get all the valid variable bindings based on the unifier 
            // updated by the combo interpreter after running the combo procedure.
            this->updateVarBindCandidates(unifier, varBindCandidates);  

            logger().debug( "PsiActionSelectionAgent::%s Update the variable bindings [ cycle = %d ]", 
                            __FUNCTION__, 
                            this->cycleCount 
                          ); 

            break; 
        }

    }// foreach 

    // Print the result to log file
    if ( hSelectedPsiRule != opencog::Handle::UNDEFINED  ) {
        logger().debug( "PsiActionSelectionAgent::%s - Pick up a Psi Rule [ cycle = %d ] : \n %s", 
                         __FUNCTION__,
                         this->cycleCount, 
                         atomSpace.atomAsString(hSelectedPsiRule).c_str()
                      );
    }
    else {
        logger().debug( "PsiActionSelectionAgent::%s - Pick up none Psi Rule [ cycle = %d ].", 
                         __FUNCTION__,
                         this->cycleCount 
                      );

    }// if

    return hSelectedPsiRule; 
}

bool PsiActionSelectionAgent::applyPsiRule(opencog::CogServer * server, 
                                           Handle hPsiRule,
                                           const std::vector<std::string> & varBindCandidates)
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get Goal, Action and Preconditions from Psi Rule  
    Handle evaluationLinkGoal, executionLinkAction, andLinkPrecondition;

    if ( !AtomSpaceUtil::splitPsiRule( atomSpace, 
                                       hPsiRule, 
                                       evaluationLinkGoal, 
                                       executionLinkAction, 
                                       andLinkPrecondition
                                     ) 
       ) 
        return false; 

    // Get ProcedureInterpreter
    Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

    // Variables used by combo interpreter
    std::vector <combo::vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    combo::vertex result; // combo::vertex is actually of type boost::variant <...>

    // Get Action name
    std::string actionName = atomSpace.getName( atomSpace.getOutgoing(executionLinkAction, 0)
                                              );
    // Get combo arguments for Action
    Handle hListLink = atomSpace.getOutgoing(executionLinkAction, 1); // Handle to ListLink containing arguments

    if ( !this->getSchemaArguments(server, hListLink, varBindCandidates, schemaArguments) )
        return false; 

    // Run the Procedure of the Action
    //
    // We will not check the state of the execution of the Action here. Because it may take some time to finish it. 
    // Instead, we will check the result of the execution within 'run' method during next "cognitive cycle". 
    //
    // There are three kinds of results: success, fail and time out (defined by 'PROCEDURE_EXECUTION_TIMEOUT')
    //
    // TODO: Before running the combo procedure, check the number of arguments the procedure needed and it actually got
    //
    // Reference: "SchemaRunner.cc" line 264-286
    //
    const Procedure::GeneralProcedure & procedure = procedureRepository.get(actionName);

    executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments);

    // Update related information
    //
    // Don't set this->previousPsiRule here, which shall be updated within 'run'  method. 
    // Otherwise you will always get opencog::Handle::UNDEFINED
    //
    this->currentSchemaId = executingSchemaId; 
    this->currentPsiRule = hPsiRule; 
    this->timeStartCurrentPsiRule = time(NULL);

    logger().debug( "PsiActionSelectionAgent::%s - Applying Psi rule: %s successfully ( currentSchemaId = %d ) [ cycle = %d ]", 
		    __FUNCTION__, 
		    atomSpace.atomAsString(hPsiRule).c_str(), 
		    this->currentSchemaId, 
		    this->cycleCount
		  );

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

    // Get rand generator
    RandGen & randGen = oac->getRandGen();

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get ProcedureInterpreter
    Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Variables used by combo interpreter
    std::vector <combo::vertex> schemaArguments;
    combo::vertex result; // combo::vertex is actually of type boost::variant <...>

    // Get pet
    Pet & pet = oac->getPet();

    // Get petId
    const std::string & petId = pet.getPetId();

    // Check if map info data is available
    if ( atomSpace.getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn( "PsiActionSelectionAgent::%s - There is no map info available yet [ cycle = %d ]", 
                        __FUNCTION__, 
                        this->cycleCount
                     );
        return;
    }

    // Check if the pet spatial info is already received
    if ( !atomSpace.getSpaceServer().getLatestMap().containsObject(petId) ) {
        logger().warn( "PsiActionSelectionAgent::%s - Pet was not inserted in the space map yet [ cycle = %d ]", 
                       __FUNCTION__, 
                       this->cycleCount
                     );
        return;
    }

    // Initialize the Mind Agent (demandGoalList etc)
    if ( !this->bInitialized )
        this->init(server);

    // Check the state of current running Action: 
    //
    // If it success, fails, or is time out, update corresponding information respectively, and continue processing.
    // Otherwise, say the current Action is still running, do nothing and simply returns. 
    //
    if (this->currentPsiRule != opencog::Handle::UNDEFINED) {

        logger().debug( "PsiActionSelectionAgent::%s currentSchemaId = %d [ cycle = %d] ", 
			__FUNCTION__, 
			this->currentSchemaId,
                        this->cycleCount	
		      );

        // If the Action has been done, check the result
        if ( procedureInterpreter.isFinished(this->currentSchemaId) ) {

            logger().debug( "PsiActionSelectionAgent::%s - The Action [ id = %d ] is finished for the Psi Rule: %s [ cycle = %d ].", 
                            __FUNCTION__,
                            this->currentSchemaId,
                            atomSpace.atomAsString(this->currentPsiRule).c_str(), 
                            this->cycleCount
                          );

            combo::vertex result = procedureInterpreter.getResult(this->currentSchemaId);

            // If check result: success
            if ( ( is_action_result(result) && get_action(result) == combo::id::action_success ) ||
                 ( is_builtin(result) && get_builtin(result) == combo::id::logical_true )
               ) {   

                // Update the truth value of the Goal related to the Action
                Handle evaluationLinkGoal = atomSpace.getOutgoing(this->currentPsiRule, 1);
                SimpleTruthValue stvTrue(1.0, 1.0); 
                atomSpace.setTV(evaluationLinkGoal, stvTrue);

                // Remove the corresponding Psi Rule from the rule list
                //
                // Note: std::remove itself actually removes NOTHING! 
                //       It only move all the elements to be removed to the end of the vector, 
                //       and then returns the iterator pointing to the first element to be removed.
                //       So you should call 'erase' method to really REMOVE. 
                //
                //       An exception is std::list, its remove method really remove element. 
                //       The behavior of 'remove_if' and 'unique' is similar to 'remove'
                this->psiPlanList[0].erase( std::remove( this->psiPlanList[0].begin(), 
                                                         this->psiPlanList[0].end(),
                                                         this->currentPsiRule
                                                       ), 
                                            this->psiPlanList[0].end()
                                          );      

               // Update current/ previous Psi Rule 
                this->previousPsiRule = this->currentPsiRule; 
                this->currentPsiRule = opencog::Handle::UNDEFINED; 
                this->currentSchemaId = 0;
            }
            // If check result: fail
            else if ( is_action_result(result) || is_builtin(result) ) {

                this->currentPsiRule = opencog::Handle::UNDEFINED;
                this->currentSchemaId = 0;

                logger().warn( "PsiActionSelectionAgent::%s - Failed to execute the Action, while applying the Psi Rule: %s [ cycle = %d ].", 
                               __FUNCTION__, 
                               atomSpace.atomAsString(this->currentPsiRule).c_str(), 
                               this->cycleCount
                             );
            }
            // If check result: unexpected result
            else {

                this->currentPsiRule = opencog::Handle::UNDEFINED; 
                this->currentSchemaId = 0; 

                stringstream unexpected_result;
                unexpected_result << result;
                logger().warn( "PsiActionSelectionAgent::%s - Action procedure result should be 'built-in' or 'action result'. Got '%s' [ cycle = %d ].",
                               __FUNCTION__, 
                               unexpected_result.str().c_str(), 
                               this->cycleCount
                             );
            }
        } 
        // If the Action fails
        else if ( procedureInterpreter.isFailed(this->currentSchemaId) ) {

            // TODO: How to judge whether an Action has failed?
            //
            //       Approach 1: Check if the Action has been finished, get the result, and then analyze the result
            //       Approach 2: Check if the Action has failed directly via ProcedureInterpreter.isFailed method 
            //
            //       We have implemented both approaches currently. However it seems one of them is surplus. 
            //       We should erase one of them, when we really understand the difference between both. 
            //       
            //       [By Zhennua Cai, on 2011-02-03]

            this->currentPsiRule = opencog::Handle::UNDEFINED;
            this->currentSchemaId = 0;

            logger().warn( "PsiActionSelectionAgent::%s - Failed to execute the Action, while applying the Psi Rule: %s [ cycle = %d ].", 
                           __FUNCTION__, 
                           atomSpace.atomAsString(this->currentPsiRule).c_str(), 
                           this->cycleCount
                         );
        }
        // If the Action is time out
        else if ( time(NULL) - this->timeStartCurrentPsiRule >  this->procedureExecutionTimeout ) { 

            // Stop the time out Action
            procedureInterpreter.stopProcedure(this->currentSchemaId);

            logger().warn( "PsiActionSelectionAgent::%s - Execution of the Action is time out, while applying the Psi Rule: %s [ cycle = %d ].", 
                           __FUNCTION__, 
                           atomSpace.atomAsString(this->currentPsiRule).c_str(), 
                           this->cycleCount
                         );

            this->currentPsiRule = opencog::Handle::UNDEFINED;
            this->currentSchemaId = 0;
        }
        // If the Action is still running, simply returns
        else {  
            logger().debug( "PsiActionSelectionAgent::%s - Current Action is still running. [ cycle = %d ].", 
                            __FUNCTION__, 
                            this->cycleCount
                          );

            return; 
        }

    }// if (this->currentPsiRule != opencog::Handle::UNDEFINED)

    // Select a Demand Goal
    Handle selectedDemandGoal;

    if ( this->psiPlanList.empty() || this->psiPlanList[0].empty() ) {

        // Select the Demand Goal with lowest truth value
        selectedDemandGoal = this->chooseMostCriticalDemandGoal(server);

        // Update the pet's previously/ currently Demand Goal
        pet.setCurrentDemandGoal( selectedDemandGoal );

        logger().debug( "PsiActionSelectionAgent::%s - Select the Demand Goal: %s [ cycle = %d ].", 
                        __FUNCTION__, 
                        atomSpace.getName( atomSpace.getOutgoing(selectedDemandGoal, 0)
                                         ).c_str(), 
                        this->cycleCount
                      );

        // Figure out a plan for the selected Demand Goal
        int steps = 5000;   // TODO: Emotional states shall have impact on steps, i.e., resource of cognitive process

//        this->planByPLN(server, selectedDemandGoal, this->psiPlanList, steps);
        this->planByNaiveBreadthFirst(server, selectedDemandGoal, this->psiPlanList, steps);
    }// if

    // Change the current Demand Goal randomly (controlled by the modulator 'SelectionThreshold')
    float selectionThreshold = AtomSpaceUtil::getCurrentModulatorLevel(randGen,
                                                                       atomSpace,
                                                                       SELECTION_THRESHOLD_MODULATOR_NAME,
                                                                       petId
                                                                      );
// TODO: uncomment the line below once finish testing
    if ( randGen.randfloat() > selectionThreshold )
//    if ( false )  // skip this for debugging
    {

        selectedDemandGoal = this->chooseRandomDemandGoal(); 

        if ( selectedDemandGoal != pet.getCurrentDemandGoal() ) {

            // Update the pet's previously/ currently Demand Goal
            pet.setCurrentDemandGoal( selectedDemandGoal );
           
            logger().debug( "PsiActionSelectionAgent::%s - Switch the Demand Goal to: %s [ cycle = %d ].", 
                            __FUNCTION__, 
                            atomSpace.getName( atomSpace.getOutgoing(selectedDemandGoal, 0)
                                             ).c_str(), 
                            this->cycleCount
                          );

            // Figure out a plan for the selected Demand Goal
            int steps = 5000; // TODO: Emotional states shall have impact on steps, i.e. resource of cognitive process

//            this->planByPLN(server, selectedDemandGoal, this->psiPlanList, steps);
            this->planByNaiveBreadthFirst(server, selectedDemandGoal, this->psiPlanList, steps);
        }// if

    }// if

    // Choose a Psi Rule to be applied
    Handle hSelectedPsiRule = opencog::Handle::UNDEFINED;

    // This vector has all possible objects/avatars ids to replace wildcard in Psi Rules
    //
    // TODO: We would not use wildcard later, 
    //       and the vector here should be used to replace VariableNode in Psi Rules with ForAllLink
    std::vector<std::string> varBindCandidates;

    if ( !this->psiPlanList.empty() ) {
        hSelectedPsiRule = this->pickUpPsiRule(server, this->psiPlanList[0], varBindCandidates);
    }
    else {
        logger().warn( "PsiActionSelectionAgent::%s - Failed to select a Psi Rule to apply because Psi Rule Lists is empty [ cycle = %d].", 
                       __FUNCTION__, 
                       this->cycleCount
                     );
        return; 
    }

    // Apply the selected Psi Rule
    if ( hSelectedPsiRule != opencog::Handle::UNDEFINED ) {

        logger().debug( "PsiActionSelectionAgent::%s - Applying the selected Psi Rule: %s [ cycle = %d ].", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(hSelectedPsiRule).c_str(),  
                        this->cycleCount
                      );
       
        this->applyPsiRule(server, hSelectedPsiRule, varBindCandidates);
    }
    else {
        logger().warn( "PsiActionSelectionAgent::%s - Failed to select a Psi Rule to apply because none of them meets their Precondition [ cycle = %d ] .", 
                       __FUNCTION__, 
                       this->cycleCount
                     );

        return; 
    }



//    // Set current Demand Goal to TestEnergyDemand for debugging
//    std::vector<Handle>::iterator iDemandGoal; 
//    Handle goalPredicateNode; 
//    for (iDemandGoal=this->demandGoalList.begin(); iDemandGoal!=this->demandGoalList.end(); iDemandGoal++) {
//        goalPredicateNode = atomSpace.getOutgoing(*iDemandGoal, 0);
//
//        if ( atomSpace.getName(goalPredicateNode) == "TestEnergyDemandGoal" )
//            break;
//    }
//
//    if ( iDemandGoal == this->demandGoalList.end() ) {
//        logger().error(
//                "PsiActionSelectionAgent::%s - Can not find TestEnergyDemandGoal for debugging [ cycle = %d ]", 
//                __FUNCTION__, 
//                this->cycleCount
//                      );
//        return; 
//    }
//    else {
//        currentDemandGoal = *iDemandGoal; 
//    }
//
//static bool bTestPLN = false;
//
//if (!bTestPLN) {
//
//bTestPLN = true; 
//
//    // Do backward inference starting from the selected Demand Goal using PLN
//    int steps = 2000;
//    const std::set<VtreeProvider *> & inferResult = this->searchBackward(currentDemandGoal, steps);
//
//    // Extract Psi Rules from trees returned by PLN
//    this->extractPsiRules(server, inferResult, this->psiRulesLists);
//
//    // Print plans
//    this->printPlans(server, this->currentDemandGoal, this->psiRulesLists);
//}

}

