/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-05-31
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "OAC.h"
#include "PsiActionSelectionAgent.h"
#include "PsiRuleUtil.h"

#include <boost/tokenizer.hpp>

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

    // Initialize other members
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
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
//    const std::string & petId = oac->getPet().getPetId();

    // Get demand names from the configuration file
    std::string demandNames = config()["PSI_DEMANDS"];

    // Process Demands one by one
    boost::tokenizer<> demandNamesTok (demandNames);

    std::string demandPredicateName;
    std::vector<Handle> outgoings; 
    std::vector<Handle> demandGoalOutgoings; 

    outgoings.clear(); 
    Handle hListLink = atomSpace.addLink(LIST_LINK, outgoings);
    Handle hDemandGoalEvaluationLink; 

    for ( boost::tokenizer<>::iterator iDemandName = demandNamesTok.begin();
          iDemandName != demandNamesTok.end();
          iDemandName ++ ) {

        demandPredicateName = (*iDemandName) + "DemandGoal";

        // Get EvaluationLink to the demand goal
        outgoings.clear(); 
        outgoings.push_back( atomSpace.addNode(PREDICATE_NODE, demandPredicateName) ); 
        outgoings.push_back(hListLink); 

        demandGoalOutgoings.push_back( atomSpace.addLink(EVALUATION_LINK, outgoings) ); 
        
    }// for

    // Create an ReferenceLink holding all the demand goals (EvaluationLink)
    outgoings.clear(); 
    outgoings.push_back( atomSpace.addNode(CONCEPT_NODE, "plan_demand_goal_list") );
    outgoings.push_back( atomSpace.addLink(LIST_LINK, demandGoalOutgoings) );  
    Handle referenceLink = atomSpace.addLink(REFERENCE_LINK, outgoings);

    logger().debug("PsiActionSelectionAgent::%s - Add the list of demand goals to AtomSpace: %s [cycle = %d]", 
                   __FUNCTION__, 
                   atomSpace.atomAsString(referenceLink).c_str(), 
                   this->cycleCount
                  ); 
}

void PsiActionSelectionAgent::printPlan(opencog::CogServer * server)
{
    AtomSpace & atomSpace = * ( server->getAtomSpace() ); 

    Handle hSelectedDemandGoal =
        AtomSpaceUtil::getReference(atomSpace, 
                                    atomSpace.getHandle(CONCEPT_NODE,
                                                        "plan_selected_demand_goal"
                                                       )
                                   );

    Handle hRuleList =
        AtomSpaceUtil::getReference(atomSpace, 
                                    atomSpace.getHandle(CONCEPT_NODE, 
                                                        "plan_rule_list"
                                                       )
                                   );

    Handle hContextList =
        AtomSpaceUtil::getReference(atomSpace, 
                                    atomSpace.getHandle(CONCEPT_NODE, 
                                                        "plan_context_list"
                                                       )
                                   );

    Handle hActionList =
        AtomSpaceUtil::getReference(atomSpace, 
                                    atomSpace.getHandle(CONCEPT_NODE, 
                                                        "plan_action_list"
                                                       )
                                   );

    std::cout<<std::endl<<"Selected Demand Goal [cycle = "<<this->cycleCount<<"]:"
             <<std::endl<<atomSpace.atomAsString(hSelectedDemandGoal)<<std::endl; 

    int i = 1; 

    foreach( const Handle hAction, atomSpace.getOutgoing(hActionList) ) {
        std::cout<<std::endl<<"Action No."<<i
                 <<std::endl<<atomSpace.atomAsString(hAction);

        ++i; 
    }

    std::cout << std::endl;
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

    // Get Procedure repository
    const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

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

#if HAVE_GUILE    

    // Initialize scheme evaluator
    SchemeEval & evaluator = SchemeEval::instance(&atomSpace);    
    std::string scheme_expression, scheme_return_value;

    scheme_expression = "( do_planning )";

    // Run the Procedure that do planning
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() ) {
        logger().error( "PsiActionSelectionAgent::%s - Failed to execute '%s'", 
                         __FUNCTION__, 
                         scheme_expression.c_str() 
                      );

        return; 
    }

    // Print the plan to the screen
    this->printPlan(server); 

#endif // HAVE_GUILE    


/**    
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

                logger().warn( "PsiActionSelectionAgent::%s - Failed to execute the Action, while applying the Psi Rule: %s [ cycle = %d ].", 
                               __FUNCTION__, 
                               atomSpace.atomAsString(this->currentPsiRule).c_str(), 
                               this->cycleCount
                             );

                this->currentPsiRule = opencog::Handle::UNDEFINED;
                this->currentSchemaId = 0;

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
            logger().warn( "PsiActionSelectionAgent::%s - Failed to execute the Action, while applying the Psi Rule: %s [ cycle = %d ].", 
                           __FUNCTION__, 
                           atomSpace.atomAsString(this->currentPsiRule).c_str(), 
                           this->cycleCount
                         );

            this->currentPsiRule = opencog::Handle::UNDEFINED;
            this->currentSchemaId = 0;
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

        if ( selectedDemandGoal == opencog::Handle::UNDEFINED ) {
            logger().warn("PsiActionSelectionAgent::%s - Failed to select the most critical Demand Goal [cycle = %d]", 
                          __FUNCTION__, 
                          this->cycleCount
                         );
            return; 
        }

        // Update the pet's previously/ currently Demand Goal
        PsiRuleUtil::setCurrentDemandGoal(atomSpace, selectedDemandGoal);         

        logger().debug( "PsiActionSelectionAgent::%s - Select the Demand Goal: %s [ cycle = %d ].", 
                        __FUNCTION__, 
                        atomSpace.getName( atomSpace.getOutgoing(selectedDemandGoal, 0)
                                         ).c_str(), 
                        this->cycleCount
                      );

        // Figure out a plan for the selected Demand Goal
        int steps = 2000000;   // TODO: Emotional states shall have impact on steps, i.e., resource of cognitive process

        if (this->bPlanByPLN)
            this->planByPLN(server, selectedDemandGoal, this->psiPlanList, steps);
        else
            this->planByNaiveBreadthFirst(server, selectedDemandGoal, this->psiPlanList, steps);
    }// if

    // Change the current Demand Goal randomly (controlled by the modulator 'SelectionThreshold')
    float selectionThreshold = AtomSpaceUtil::getCurrentModulatorLevel(atomSpace,
                                                                       SELECTION_THRESHOLD_MODULATOR_NAME,
                                                                       randGen
                                                                      );
// TODO: uncomment the line below once finish testing
    if ( randGen.randfloat() > selectionThreshold )
//    if ( false )  // skip this for debugging
    {

        selectedDemandGoal = this->chooseRandomDemandGoal(); 

        if ( selectedDemandGoal == opencog::Handle::UNDEFINED ) {
            logger().warn("PsiActionSelectionAgent::%s - Failed to randomly select a Demand Goal [cycle = %d]", 
                          __FUNCTION__, 
                          this->cycleCount
                         );
            return; 
        }

        Handle hCurrentDemandGoal, hReferenceLink; 
        hCurrentDemandGoal = PsiRuleUtil::getCurrentDemandGoal(atomSpace, hReferenceLink); 

        if ( selectedDemandGoal != hCurrentDemandGoal ) {

            // Update the pet's previously/ currently Demand Goal
            PsiRuleUtil::setCurrentDemandGoal(atomSpace, selectedDemandGoal);         
           
            logger().debug( "PsiActionSelectionAgent::%s - Switch the Demand Goal to: %s [ cycle = %d ].", 
                            __FUNCTION__, 
                            atomSpace.getName( atomSpace.getOutgoing(selectedDemandGoal, 0)
                                             ).c_str(), 
                            this->cycleCount
                          );

            // Figure out a plan for the selected Demand Goal
            int steps = 2000000; // TODO: Emotional states shall have impact on steps, i.e. resource of cognitive process

            if (this->bPlanByPLN)
                this->planByPLN(server, selectedDemandGoal, this->psiPlanList, steps);
            else
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
      
        this->currentSchemaId = PsiRuleUtil::applyPsiRule( atomSpace,
                                                           procedureInterpreter, 
                                                           procedureRepository, 
                                                           hSelectedPsiRule, 
                                                           varBindCandidates,
                                                           randGen
                                                         );
        this->currentPsiRule = hSelectedPsiRule; 
        this->timeStartCurrentPsiRule = time(NULL);
    }
    else {
        logger().warn( "PsiActionSelectionAgent::%s - Failed to select a Psi Rule to apply because none of them meets their Precondition [ cycle = %d ] .", 
                       __FUNCTION__, 
                       this->cycleCount
                     );

        return; 
    }
*/    

}

