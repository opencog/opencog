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
    // TODO: Shall we have to to do so? 
    AtomSpaceWrapper* asw = ASW(opencog::server().getAtomSpace());
#if LOCAL_ATW
    ((LocalATW*)asw)->SetCapacity(10000);
#endif  
    asw->archiveTheorems = false;
    asw->allowFWVarsInAtomSpace = 
    config().get_bool("PLN_FW_VARS_IN_ATOMSPACE");

    currentDebugLevel = config().get_int("PLN_LOG_LEVEL");

    this->currentPsiRule = opencog::Handle::UNDEFINED; 
    this->previousPsiRule = opencog::Handle::UNDEFINED;

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
    // The implementation below borrowed many source code from the function opencog::pln::infer ("PLNModule.cc")
    //
    // TODO: This method would cause infinite loop

    AtomSpace & atomSpace = * ( server().getAtomSpace() ); 
   
    // Create BITNodeRoot for the Goal (Target)
//    pHandleSeq fakeHandles = ASW(&atomSpace)->realToFakeHandle(goalHandle);
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

    const std::set<VtreeProvider *> & result = state->infer(steps,     // proof resources
                                                            0.000001f, // minimum confidence for storage
                                                            0.01f      // minimum confidence for abort
                                                           ); 

//    std::set<VtreeProvider *> result = state->infer(steps,     // proof resources
//                                                            0.000001f, // minimum confidence for storage
//                                                            0.01f      // minimum confidence for abort
//                                                           ); 

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

bool PsiActionSelectionAgent::extractPsiRules(const std::set<VtreeProvider *> & inferResult, 
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

    // Process the tree one by one
    std::vector< std::vector<Handle> >::iterator iPsiRules = psiRulesList.begin();

    foreach(VtreeProvider * vtp, inferResult) {

std::cout<<"PsiActionSelectionAgent::extractPsiRules - Going to extract Rules"<<std::endl;        
        // Get the pHandle to Goal (i.e. the root node of the tree)
        pHandle goalpHandle = _v2h( *(vtp->getVtree().begin()) );

        this->extractPsiRules(goalpHandle, *iPsiRules, 0);        

        logger().debug("PsiActionSelectionAgent::%s - Extract %d OpenPsi Rules [ cycle = %d ] .",
                       __FUNCTION__, 
                       iPsiRules->size(), 
                       this->cycleCount
                      );

 std::cout<<"PsiActionSelectionAgent::extractPsiRules - Extract Rules Done. Found "
         <<iPsiRules->size()
         <<" OpenPsi Rules."
         <<std::endl;
     
        iPsiRules++;

    }// foreach

    return true;
}

void PsiActionSelectionAgent::extractPsiRules(pHandle ph, std::vector<Handle> & psiRules, unsigned int level)
{
    AtomSpace & atomSpace = * ( server().getAtomSpace() ); 
   
    AtomSpaceWrapper *asw = GET_ASW;

    if ( ph == PHANDLE_UNDEFINED || asw->isType(ph) ) {
        logger().warn(
                "PsiActionSelectionAgent::%s - Trying to extract Psi rules from NULL/ Virtual atom [ cycle = %d ] .",
                       __FUNCTION__, 
                       this->cycleCount
                     );
        return;
    }

    if (level > 20) {
        logger().warn("PsiActionSelectionAgent::%s - Maximum recursion depth exceeded [ cycle = %d ] .",
                       __FUNCTION__, 
                       this->cycleCount
                     );
    	return; 
    }

    // TODO: add h psiRulesList if h is a Psi Rule
    vhpair vhp = asw->fakeToRealHandle(ph);
    Handle realHandle = vhp.first; 

logger().debug("PsiActionSelectionAgent::%s - Going to check Atom: '%s' [ cycle = %d ].",
               __FUNCTION__, 
               atomSpace.atomAsString(realHandle).c_str(),
               this->cycleCount
              );
   

    if ( this->isHandleToPsiRule(realHandle) ) { 

logger().debug("PsiActionSelectionAgent::%s - Found a Psi Rule: '%s' [ cycle = %d ].",
               __FUNCTION__, 
               atomSpace.atomAsString(realHandle).c_str(),
               this->cycleCount
              );
       
        psiRules.push_back(realHandle);
    }        

    // Extract Psi rules from its children nodes
    std::map<pHandle, RulePtr>::const_iterator pln_rule_it = haxx::get_inferred_with().find(ph); 
    std::map<pHandle, vector<pHandle> >::const_iterator ph_it = haxx::get_inferred_from().find(ph);

logger().debug("PsiActionSelectionAgent::%s - Size of trail for the whole tree is %d [ cycle = %d ].",
               __FUNCTION__, 
               haxx::get_inferred_from().size(), 
               this->cycleCount
              );

    if( pln_rule_it == haxx::get_inferred_with().end() || 
        ph_it == haxx::get_inferred_from().end() ) 
        return; 

logger().debug("PsiActionSelectionAgent::%s - Size of trail for this Atom is %d [ cycle = %d ].",
               __FUNCTION__, 
               ph_it->second.size(), 
               this->cycleCount
              );
   

    foreach(pHandle arg_ph, ph_it->second) {

logger().debug("PsiActionSelectionAgent::%s - Will recursively check Atom: '%s' [ cycle = %d ].",
               __FUNCTION__, 
               atomSpace.atomAsString(asw->fakeToRealHandle(arg_ph).first
                                     ).c_str(),
               this->cycleCount
              );

        this->extractPsiRules(arg_ph, psiRules, level+1);

    }        
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

logger().debug("PsiActionSelectionAgent::%s - Going to check Atom: '%s' [ cycle = %d ].",
               __FUNCTION__, 
               atomSpace.atomAsString(h).c_str(),
               this->cycleCount
              );

    // Check h
    // TODO: Use PredictiveImplicationLink instead of ImplicationLink
//    if ( atomSpace.getType(h) != PREDICTIVE_IMPLICATION_LINK || atomSpace.getArity(h) != 2 )
    if ( atomSpace.getType(h) != IMPLICATION_LINK || atomSpace.getArity(h) != 2 )
        return false; 

logger().debug("PsiActionSelectionAgent::%s - Check h OK. [ cycle = %d ].",
               __FUNCTION__, 
               this->cycleCount
              );

    // Get handles to premise and goal
    Handle andLinkPreconditionAction = atomSpace.getOutgoing(h, 0);
    Handle evaluationLinkGoal = atomSpace.getOutgoing(h, 1);

    // Check premise
    if ( atomSpace.getType(andLinkPreconditionAction) != AND_LINK || atomSpace.getArity(h) != 2 )
        return false;

logger().debug("PsiActionSelectionAgent::%s - Check Premise OK. [ cycle = %d ].",
               __FUNCTION__, 
               this->cycleCount
              );

    // Get handle to action
    Handle executionLinkAction = atomSpace.getType( 
                                                      atomSpace.getOutgoing(andLinkPreconditionAction, 0) 
                                                  ) == EXECUTION_LINK ? 
                                 atomSpace.getOutgoing(andLinkPreconditionAction, 0) :
                                 atomSpace.getOutgoing(andLinkPreconditionAction, 1);

    // Check action
    if ( atomSpace.getType(executionLinkAction) != EXECUTION_LINK ||
         atomSpace.getArity(executionLinkAction) != 2 ||
         atomSpace.getType( atomSpace.getOutgoing(executionLinkAction, 0) ) != GROUNDED_SCHEMA_NODE
       )
        return false; 

logger().debug("PsiActionSelectionAgent::%s - Check Action OK. [ cycle = %d ].",
               __FUNCTION__, 
               this->cycleCount
              );

    // Check goal
    if ( atomSpace.getType(evaluationLinkGoal) != EVALUATION_LINK ||
         atomSpace.getArity(evaluationLinkGoal) != 2 ||
         atomSpace.getType( atomSpace.getOutgoing(evaluationLinkGoal, 0) ) != PREDICATE_NODE 
       )
        return false; 

logger().debug("PsiActionSelectionAgent::%s - Check Goal OK. [ cycle = %d ].",
               __FUNCTION__, 
               this->cycleCount
              );

    // Now we are pretty sure it(h) is a Psi rule
    return true;
}

void PsiActionSelectionAgent::printPlans(const std::vector< std::vector<Handle> > & psiRulesLists)
{
    std::cout<<"Found "<<psiRulesLists.size()<<" plans"<<std::endl;

    int planNo=1;
    const AtomSpace & atomSpace = * ( opencog::server().getAtomSpace() );

    foreach(std::vector<Handle> psiRules, psiRulesLists) {
        std::cout<<std::endl<<"Plan No."<<planNo<<" contains "<<psiRules.size()<<" Actions:"
                <<std::endl
                <<"TestEnergyDemandGoal ";

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

         planNo++;   

    }// foreach
}

int PsiActionSelectionAgent::getSchemaArguments(opencog::CogServer * server, Handle hListLink, 
                                                std::vector <combo::vertex> & schemaArguments) 
{
    AtomSpace & atomSpace = * ( server->getAtomSpace() );

    // Check hListLink is of type ListLink
    if ( atomSpace.getType(hListLink) != LIST_LINK ) {
        logger().error(
                "PsiActionSelectionAgent::%s - Link containing combo arguments shoud be of type ListLink. Got '%s'", 
                         __FUNCTION__,
                         classserver().getTypeName( atomSpace.getType(hListLink)
                                                  ).c_str() 
                      );
        return 0;
    }

    schemaArguments.clear(); 

    // Process the arguments according to its type
    foreach( Handle  hArgument, atomSpace.getOutgoing(hListLink) ) {
        if (atomSpace.getType(hArgument) == NUMBER_NODE) {
            schemaArguments.push_back(combo::contin_t(
                                          boost::lexical_cast<combo::contin_t>(atomSpace.getName(hArgument)
                                                                              )
                                                     ) 
                                     );
        }
        else {
            schemaArguments.push_back( atomSpace.getName(hArgument) );
        }
    }// foreach

    return schemaArguments.size();
}

bool PsiActionSelectionAgent::isSatisfied(opencog::CogServer * server, Handle hPrecondition) 
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get Precondition name
    std::string preconditionName = atomSpace.getName(hPrecondition);

    // Get ProcedureInterpreter
    Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

    // Variables used by combo interpreter
    std::vector <combo::vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    combo::vertex result; // combo::vertex is actually of type boost::variant <...>

    // Check if the Precondition is satisfied
    if ( atomSpace.getType(hPrecondition) == PREDICATE_NODE ) {
        if ( atomSpace.getTV(hPrecondition).getMean() >= 0.99 ) 
            return true; 
    }
    else if ( atomSpace.getType(hPrecondition) == GROUNDED_PREDICATE_NODE ) {
        // Get schemaArguments
        this->getSchemaArguments( server,
                                  atomSpace.getOutgoing(hPrecondition, 1), // Handle to ListLink containing arguments
                                  schemaArguments
                                );

        // Run the Procedure of the Precondition
        const Procedure::GeneralProcedure & procedure = procedureRepository.get(preconditionName);

        executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments);

        // TODO: What does this for?
        while ( !procedureInterpreter.isFinished(executingSchemaId) )
            procedureInterpreter.run(NULL);  

        // Check if the the Procedure run successfully
        if ( procedureInterpreter.isFailed(executingSchemaId) ) {
            logger().warn( "PsiActionSelectionAgent::%s - Failed to execute '%s'", 
                             __FUNCTION__, 
                             preconditionName.c_str()
                          );

            return false; 
        }

        result = procedureInterpreter.getResult(executingSchemaId);

        if ( result == combo::id::logical_true )
            return true;  
    }
    else {
        logger().warn( "PsiActionSelectionAgent::%s - Precondition should be of type PredicateNode or GroundedPredicateNode. But got type '%s' for '%s' precondition", 
                         __FUNCTION__,
                         classserver().getTypeName( atomSpace.getType(hPrecondition)
                                                  ).c_str(), 
                         atomSpace.getName(hPrecondition).c_str(),  
                         preconditionName.c_str()
                     );

        return false;    
    }// if
}

Handle PsiActionSelectionAgent::pickUpPsiRule(opencog::CogServer * server, const std::vector<Handle> & psiRules) 
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    bool bAllPreconditionsSatisfied = true;
    Handle hSelectedPsiRule = opencog::Handle::UNDEFINED; 

    foreach(Handle hPsiRule, psiRules) {
        // Get the handle to Preconditions and Action
        Handle andLinkPreconditionAction = atomSpace.getOutgoing(hPsiRule, 0);

        // Get handle to Preconditions
        Handle andLinkPreconditions = atomSpace.getType( 
                                                           atomSpace.getOutgoing(andLinkPreconditionAction, 0) 
                                                       ) == AND_LINK ? 
                                          atomSpace.getOutgoing(andLinkPreconditionAction, 0) :
                                          atomSpace.getOutgoing(andLinkPreconditionAction, 1);

        // Check Preconditions one by one
        foreach( Handle hPrecondition, atomSpace.getOutgoing(andLinkPreconditions) ) {
            if ( !isSatisfied(server, hPrecondition) ) {
                bAllPreconditionsSatisfied = false;
                break; 
            }
        }// foreach
       
        if (bAllPreconditionsSatisfied) {
            hSelectedPsiRule = hPsiRule;  // Record the Psi Rule that all its Preconditions are satisfied
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

bool PsiActionSelectionAgent::applyPsiRule(opencog::CogServer * server, Handle hPsiRule)
{
    if ( hPsiRule == opencog::Handle::UNDEFINED )
        return false;

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

    // Get handles to premise and goal
    Handle andLinkPreconditionAction = atomSpace.getOutgoing(hPsiRule, 0);
    Handle evaluationLinkGoal = atomSpace.getOutgoing(hPsiRule, 1);

    // Get handle to action
    Handle executionLinkAction = atomSpace.getType( 
                                                      atomSpace.getOutgoing(andLinkPreconditionAction, 0) 
                                                  ) == EXECUTION_LINK ? 
                                 atomSpace.getOutgoing(andLinkPreconditionAction, 0) :
                                 atomSpace.getOutgoing(andLinkPreconditionAction, 1);

    // Get Action name
    std::string actionName = atomSpace.getName( atomSpace.getOutgoing(executionLinkAction, 0)
                                              );
    // Get combo arguments for Action
    this->getSchemaArguments( server,
                              atomSpace.getOutgoing(executionLinkAction, 1), // Handle to ListLink containing arguments
                              schemaArguments
                            );

    // Run the Procedure of the Action
    //
    // TODO: Before running the combo procedure, check the number of arguments the procedure needed and it actually got
    //
    // Reference: "SchemaRunner.cc" line 264-286
    //
    const Procedure::GeneralProcedure & procedure = procedureRepository.get(actionName);

    executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments);

    // TODO: What does this for?
    while ( !procedureInterpreter.isFinished(executingSchemaId) )
        procedureInterpreter.run(NULL);  

    // Check if the the Procedure run successfully
    if ( procedureInterpreter.isFailed(executingSchemaId) ) {
        logger().warn( "PsiActionSelectionAgent::%s - Failed to execute '%s'", 
                         __FUNCTION__, 
                         actionName.c_str()
                     );

        return false; 
    }

    // TODO: Set the truth value of the corresponding Goal to 'true', because the Action is supposed to lead to the Goal
   


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

static bool bTestPLN = false;

if (!bTestPLN) {

bTestPLN = true; 

    // Do backward inference starting from the selected Demand Goal using PLN
    int steps = 2000;
    const std::set<VtreeProvider *> & inferResult = this->searchBackward(currentDemandGoal, steps);

    // Extract Psi Rules from trees returned by PLN
    this->extractPsiRules(inferResult, this->psiRulesLists);

    // Print plans
    this->printPlans(this->psiRulesLists);
}

}

