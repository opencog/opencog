/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRuleUtil.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-05-18
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

#include <opencog/comboreduct/combo/vertex.h>

#include <opencog/nlp/types/atom_types.h>
#include <opencog/spacetime/SpaceServer.h>

#include "PsiRuleUtil.h"

using namespace opencog;
using namespace oac;
using namespace combo;

bool PsiRuleUtil::splitPsiRule(const AtomSpace & atomSpace,
                               const Handle hPsiRule, 
                               Handle & hGoalEvaluationLink,
                               Handle & hActionExecutionLink, 
                               Handle & hPreconditionAndLink
                              )
{
    // Check hPsiRule
    // TODO: Use PredictiveImplicationLink instead of ImplicationLink
    //       if ( atomSpace.getType(hPsiRule) != PREDICTIVE_IMPLICATION_LINK ||
    //            atomSpace.getArity(hPsiRule) != 2 )
    if ( atomSpace.getType(hPsiRule) != IMPLICATION_LINK ||
         atomSpace.getArity(hPsiRule) != 2 ){

        logger().error( "PsiRuleUtil::%s - %s is not a valid Psi Rule. The correct Psi Rule should be of type ImplicationLink and with two arity.", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(hPsiRule).c_str()
                      );

        return false; 
    }
 
    // Get handles to premise and goal
    Handle andLinkPreconditionAction = atomSpace.getOutgoing(hPsiRule, 0);
    Handle evaluationLinkGoal = atomSpace.getOutgoing(hPsiRule, 1);

    // Check goal
    if ( atomSpace.getType(evaluationLinkGoal) != EVALUATION_LINK ||
         atomSpace.getArity(evaluationLinkGoal) != 2 ||
         ! classserver().isA(
         atomSpace.getType( atomSpace.getOutgoing(evaluationLinkGoal, 0) ),
                            PREDICATE_NODE) ) {

        logger().error( "PsiRuleUtil::%s - %s is not a valid Psi Goal. The correct Psi Goal should be of type EvaluationLink, with two arity and its first outgoing should be of type PredicateNode.", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(evaluationLinkGoal).c_str()
                      ); 

        return false; 
    }

    // Check premise
    if ( atomSpace.getType(andLinkPreconditionAction) != AND_LINK || 
         atomSpace.getArity(andLinkPreconditionAction) != 2 ) {

        logger().error( 
                "PsiRuleUtil::%s - Failed to find an AndLink holding the Action and all the Preconditions in %s",
                       __FUNCTION__, 
                       atomSpace.atomAsString(hPsiRule).c_str()
                      );

        return false;
    }

    // Get handle to action and preconditions
    Handle executionLinkAction;
    Handle andLinkPreconditions;

    if ( atomSpace.getType( atomSpace.getOutgoing(andLinkPreconditionAction, 0) 
                          ) == EXECUTION_LINK ) {
        executionLinkAction = atomSpace.getOutgoing(andLinkPreconditionAction, 0);
        andLinkPreconditions = atomSpace.getOutgoing(andLinkPreconditionAction, 1);
    }
    else {
        executionLinkAction = atomSpace.getOutgoing(andLinkPreconditionAction, 1);
        andLinkPreconditions = atomSpace.getOutgoing(andLinkPreconditionAction, 0);
    }
           
    // Check action
    if ( atomSpace.getType(executionLinkAction) != EXECUTION_LINK ||
         atomSpace.getArity(executionLinkAction) != 2 ||
         atomSpace.getType( atomSpace.getOutgoing(executionLinkAction, 0) ) != GROUNDED_SCHEMA_NODE ) {

        logger().error( "PsiRuleUtil::%s - %s is not a valid Psi Action. The correct Psi Action should be of type ExecutionLink, with two arity and its first outgoing should be of type GroundedSchemaNode.", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(executionLinkAction).c_str()
                      ); 

        return false; 
    }

    // Check preconditions
    if ( atomSpace.getType(andLinkPreconditions) != AND_LINK ) {

        logger().error( "PsiRuleUtil::%s - %s is not a valid AndLink containing all the Psi preconditions.", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(andLinkPreconditions).c_str()
                      );

        return false; 
    }

    // Return the Goal, Action and Preconditions 
    hGoalEvaluationLink = evaluationLinkGoal; 
    hActionExecutionLink = executionLinkAction; 
    hPreconditionAndLink = andLinkPreconditions; 

    return true; 
}

bool PsiRuleUtil::isHandleToPsiRule(const AtomSpace & atomSpace, Handle h)
{
    logger().debug("PsiRuleUtil::%s - Going to check Atom: '%s'.",
                   __FUNCTION__, 
                   atomSpace.atomAsString(h).c_str()
                  );

    // Check h
    // TODO: Use PredictiveImplicationLink instead of ImplicationLink
    //       if ( atomSpace.getType(h) != PREDICTIVE_IMPLICATION_LINK || atomSpace.getArity(h) != 2 )
    if ( atomSpace.getType(h) != IMPLICATION_LINK || atomSpace.getArity(h) != 2 )
        return false; 
 
    // Get handles to premise and goal
    Handle andLinkPreconditionAction = atomSpace.getOutgoing(h, 0);
    Handle evaluationLinkGoal = atomSpace.getOutgoing(h, 1);

    // Check premise
    if ( atomSpace.getType(andLinkPreconditionAction) != AND_LINK ||
         atomSpace.getArity(andLinkPreconditionAction) != 2 )
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
         atomSpace.getType( atomSpace.getOutgoing(executionLinkAction, 0) ) != GROUNDED_SCHEMA_NODE
       )
        return false; 

    // Check goal
    if ( atomSpace.getType(evaluationLinkGoal) != EVALUATION_LINK ||
         atomSpace.getArity(evaluationLinkGoal) != 2 ||
         ! classserver().isA(atomSpace.getType( atomSpace.getOutgoing(evaluationLinkGoal, 0)), PREDICATE_NODE) ||
         ! classserver().isA(atomSpace.getType( atomSpace.getOutgoing(evaluationLinkGoal, 1)), LIST_LINK)
       )
        return false; 

    // Now we are pretty sure it(h) is a Psi rule
    return true;
}

bool PsiRuleUtil::getSchemaArguments(const AtomSpace & atomSpace, 
                                     Handle hListLink, 
                                     const std::vector<std::string> & varBindCandidates, 
                                     std::vector <vertex> & schemaArguments)
{
    // Check hListLink is of type ListLink
    if ( atomSpace.getType(hListLink) != LIST_LINK ) {
        logger().error(
                "PsiRuleUtil::%s - Link containing combo arguments should be of type ListLink. Got '%s'", 
                         __FUNCTION__,
                         classserver().getTypeName( atomSpace.getType(hListLink)
                                                  ).c_str() 
                      );
        return false;
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
        indexVariableBind = randGen().randint( varBindCandidates.size() ); 
        variableBind = varBindCandidates[indexVariableBind];
        bChooseVariableBind = true; 
    }

    // Clear old schema arguments
    schemaArguments.clear(); 

    // Process the arguments according to its type
    foreach( Handle  hArgument, atomSpace.getOutgoing(hListLink) ) {

        Type argumentType = atomSpace.getType(hArgument);

        if (argumentType == NUMBER_NODE) {
            schemaArguments.push_back(contin_t(
                                          boost::lexical_cast<contin_t>(atomSpace.getName(hArgument)
                                                                              )
                                                     ) 
                                     );
        }
        else if (argumentType == VARIABLE_NODE) {
            schemaArguments.push_back(contin_t(
                                          boost::lexical_cast<contin_t>(atomSpace.getName(hArgument)
                                                                              )
                                                     ) 
                                     );

            if ( bChooseVariableBind ) {
                schemaArguments.push_back(variableBind);  
            }
            else {
                logger().error( "PsiRuleUtil::%s - Failed to find any valid variable binding for VarialbeNode '%s'", 
                                __FUNCTION__, 
                                atomSpace.getName(hArgument).c_str()
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

void PsiRuleUtil::initVarBindCandidates(const AtomSpace & atomSpace,  
                                        std::vector<std::string> & varBindCandidates)
{
    varBindCandidates.clear(); 
    const SpaceServer::SpaceMap& spaceMap = spaceServer().getLatestMap(); 
    spaceMap.findAllEntities( back_inserter(varBindCandidates) );
}

void PsiRuleUtil::initUnifier(variable_unifier & unifier,
                              const std::vector<std::string> & varBindCandidates)
{
    unifier.clear(); 

    foreach(const std::string & varBind, varBindCandidates) {
        unifier.insert(varBind, 
                       true  // Each variable binding is considered as valid initially. 
                             // Then the combo procedure would update its state (valid/invalid)
                             // after running the procedure. 
                      );
    }
} 

void PsiRuleUtil::updateVarBindCandidates(const variable_unifier & unifier, 
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

            logger().debug("PsiRuleUtil::%s - Found a valid variable binding '%s'", 
                           __FUNCTION__, 
                           iVarBind->first.c_str()
                          );
        }
    }
}

bool PsiRuleUtil::isSatisfied(const AtomSpace & atomSpace,
                              Procedure::ProcedureInterpreter & procedureInterpreter,
                              const Procedure::ProcedureRepository & procedureRepository,
                              Handle hPrecondition)
{
    std::vector<std::string> varBindCandidates;

    // Initialize the variable bindings with all the entities the pet encounters
    PsiRuleUtil::initVarBindCandidates(atomSpace, varBindCandidates);

    logger().debug( "PsiRuleUtil::%s Initialize the variable bindings ( size = %d ) with all the entities that the pet encounters.",
                    __FUNCTION__,
                    varBindCandidates.size()
                  );

    // Initialize the unifier used by combo interpreter
    //
    // Initially all the entities the pet encounters are considered as valid variable bindings.
    // Then the combo interpreter would update the states (valid/ invalid) of each binding.
    variable_unifier unifier;
    PsiRuleUtil::initUnifier(unifier, varBindCandidates);

    logger().debug( "PsiRuleUtil::%s Initialize the unifier ( size = %d )",
                    __FUNCTION__,
                    unifier.size()
                  );

    return isSatisfied(atomSpace,
            procedureInterpreter,
            procedureRepository,
            hPrecondition,
            unifier);
}

bool PsiRuleUtil::isSatisfied(const AtomSpace & atomSpace, 
                              Procedure::ProcedureInterpreter & procedureInterpreter, 
                              const Procedure::ProcedureRepository & procedureRepository, 
                              Handle hPrecondition, 
                              variable_unifier & unifier)
{
    // Variables used by combo interpreter
    std::vector <vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    vertex result; // vertex is actually of type boost::variant <...>

    // Check the input (hPrecondition)
    if ( atomSpace.getType(hPrecondition) != EVALUATION_LINK ||
         atomSpace.getArity(hPrecondition) != 2 ) {
        logger().warn( "PsiRuleUtil::%s - Precondition should be of type EvaluationLink and with two arity. But got %s.", 
                        __FUNCTION__,
                        atomSpace.atomAsString(hPrecondition).c_str()
                     );

        return false;    
    }
       
    // Get the PredicateNode or GroundedPredicateNode inside hPrecondition
    Handle hNode = atomSpace.getOutgoing(hPrecondition, 0); 

    // Get Precondition name
    std::string preconditionName = atomSpace.getName(hNode);

    // For PredicateNode, simply check its truth value
    if ( atomSpace.getType(hNode) == PREDICATE_NODE ) {

        logger().debug("PsiRuleUtil::%s - Got a PredicateNode: %s", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hNode).c_str()
                      );

        if ( atomSpace.getMean(hPrecondition) >= 0.001 )
            return true; 
    }
    // For GroundedPredicateNode, run the corresponding combo script, and then analyze the result
    else if ( atomSpace.getType(hNode) == GROUNDED_PREDICATE_NODE ) {

        logger().debug("PsiRuleUtil::%s - Got a GroundedPredicateNode: %s", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hNode).c_str()
                      );

        // Get schemaArguments
        Handle hListLink = atomSpace.getOutgoing(hPrecondition, 1); // Handle to ListLink containing arguments
        std::vector<std::string> emptyVarBindCandidates; 

        if ( !PsiRuleUtil::getSchemaArguments( atomSpace, hListLink, emptyVarBindCandidates, schemaArguments) )
            return false; 

        logger().debug("PsiRuleUtil::%s - Got %d arguments.", 
                       __FUNCTION__, 
                       schemaArguments.size()
                      );

        // Run the Procedure of the Precondition
        // TODO: If the get method fails, what happens?
        const Procedure::GeneralProcedure & procedure = procedureRepository.get(preconditionName);

        executingSchemaId = procedureInterpreter.runProcedure(procedure, schemaArguments, unifier);

        // Wait until the combo script has been done
        while ( !procedureInterpreter.isFinished(executingSchemaId) )
            procedureInterpreter.run(NULL);  

        logger().debug("PsiRuleUtil::%s - Finish executing combo procedure %s.", 
                       __FUNCTION__, 
                       preconditionName.c_str() 
                      );

        // Check if the the Procedure run successfully
        if ( procedureInterpreter.isFailed(executingSchemaId) ) {
            logger().warn( "PsiRuleUtil::%s - Failed to execute '%s'", 
                             __FUNCTION__, 
                             preconditionName.c_str()
                          );

            return false; 
        }

        result = procedureInterpreter.getResult(executingSchemaId);

        logger().debug(
                "PsiRuleUtil::%s - Got the result after executing combo procedure %s.", 
                       __FUNCTION__, 
                       preconditionName.c_str()
                      );

        // TODO fix that thing
        // see comment in combo/vertex.h
        if (combo::operator==(result,opencog::combo::id::logical_true) ) {
        // if (result == id::logical_true) {
            logger().debug( "PsiRuleUtil::%s - The Precondition '%s' is true.", 
                            __FUNCTION__, 
                            preconditionName.c_str()
                          );   		
            return true; 
        } else {
            logger().debug( "PsiRuleUtil::%s - The Precondition '%s' is false.", 
                             __FUNCTION__, 
                             preconditionName.c_str() 
                          );   		
            return false; 
        }
    }
    // For other types of Atom, return false
    else {
        logger().warn( "PsiRuleUtil::%s - First outgoing of Precondition should be of type PredicateNode or GroundedPredicateNode. But got type '%s' in '%s'.", 
                         __FUNCTION__,
                         classserver().getTypeName( atomSpace.getType(hNode)
                                                  ).c_str(), 
                         atomSpace.atomAsString(hPrecondition).c_str() 
                     );

        return false;    
    }// if
    return false;
}

bool PsiRuleUtil::allPreconditionsSatisfied(const AtomSpace & atomSpace, 
                                            Procedure::ProcedureInterpreter & procedureInterpreter, 
                                            const Procedure::ProcedureRepository & procedureRepository, 
                                            Handle hPsiRule, 
                                            std::vector<std::string> & varBindCandidates)
{
    Handle hGoalEvaluationLink, hActionExecutionLink, hPreconditionAndLink;
    std::vector<Handle> hPreconditionEvalutaionLinks;  

    // get the handles to preconditions
    if ( !PsiRuleUtil::splitPsiRule( atomSpace, 
                                     hPsiRule, 
                                     hGoalEvaluationLink, 
                                     hActionExecutionLink,
                                     hPreconditionAndLink
                                   ) ) 
        return false; 

    hPreconditionEvalutaionLinks = atomSpace.getOutgoing(hPreconditionAndLink);

    // Initialize the variable bindings with all the entities the pet encounters 
    PsiRuleUtil::initVarBindCandidates(atomSpace, varBindCandidates); 

    logger().debug( "PsiRuleUtil::%s Initialize the variable bindings ( size = %d ) with all the entities that the pet encounters.", 
                    __FUNCTION__, 
                    varBindCandidates.size()
                  ); 

    // Initialize the unifier used by combo interpreter
    //
    // Initially all the entities the pet encounters are considered as valid variable bindings. 
    // Then the combo interpreter would update the states (valid/ invalid) of each binding.
    variable_unifier unifier; 
    PsiRuleUtil::initUnifier(unifier, varBindCandidates);

    logger().debug( "PsiRuleUtil::%s Initialize the unifier ( size = %d )", 
                    __FUNCTION__, 
                    unifier.size()
                  ); 

    // Check all the  Preconditions one by one
    bool bAllPreconditionsSatisfied = true;

    foreach( Handle hPrecondition, hPreconditionEvalutaionLinks ) {
        logger().debug( "PsiRuleUtil::%s - Going to check the Precondition: %s", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(hPrecondition).c_str()
                      );

        if ( !PsiRuleUtil::isSatisfied( atomSpace,
                                        procedureInterpreter,
                                        procedureRepository, 
                                        hPrecondition,
                                        unifier
                                       ) ) {
            bAllPreconditionsSatisfied = false;
            break; 
        }
    }// foreach
   
    if (bAllPreconditionsSatisfied) {

        // Get all the valid variable bindings based on the unifier 
        // updated by the combo interpreter after running the combo procedure.
        PsiRuleUtil::updateVarBindCandidates(unifier, varBindCandidates);  

        logger().debug( "PsiRuleUtil::%s Update the variable bindings", 
                        __FUNCTION__
                      ); 

    }

    return bAllPreconditionsSatisfied; 
}

Procedure::RunningProcedureID PsiRuleUtil::applyPsiRule(const AtomSpace & atomSpace, 
                                                        Procedure::ProcedureInterpreter & procedureInterpreter, 
                                                        const Procedure::ProcedureRepository & procedureRepository, 
                                                        Handle hPsiRule,
                                                        const std::vector<std::string> & varBindCandidates)
{
    // Executing schema id when error happens
    const Procedure::RunningProcedureID errorExecutingSchemaId = 0; 

    // Get Goal, Action and Preconditions from Psi Rule  
    Handle evaluationLinkGoal, executionLinkAction, andLinkPrecondition;

    if ( !PsiRuleUtil::splitPsiRule( atomSpace, 
                                     hPsiRule, 
                                     evaluationLinkGoal, 
                                     executionLinkAction, 
                                     andLinkPrecondition
                                   ) 
       ) 
        return errorExecutingSchemaId; 

    // Variables used by combo interpreter
    std::vector <vertex> schemaArguments;
    Procedure::RunningProcedureID executingSchemaId;
    vertex result; // combo::vertex is actually of type boost::variant <...>

    // Get Action name
    std::string actionName = atomSpace.getName( atomSpace.getOutgoing(executionLinkAction, 0)
                                              );
    // Get combo arguments for Action
    Handle hListLink = atomSpace.getOutgoing(executionLinkAction, 1); // Handle to ListLink containing arguments

    if ( !PsiRuleUtil::getSchemaArguments(atomSpace, hListLink, varBindCandidates, schemaArguments) )
        return errorExecutingSchemaId; 

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

    logger().debug( "PsiRuleUtil::%s - Applying Psi rule: %s successfully ( currentSchemaId = %d )", 
		            __FUNCTION__,
                    atomSpace.atomAsString(hPsiRule).c_str(), 
                    executingSchemaId 
                  );
    logger().debug( "PsiRuleUtil::%s - New action: %s",
                    __FUNCTION__,
                    procedure.getName().c_str()
                  );

    return executingSchemaId; 
}

    /**
     * Get/ set previous/ current Demand Goal
     *
     * These information are stored in AtomSpace as follows:
     *
     * ReferenceLink
     *     ConceptNode "PreviousDemandGoal"
     *     EvaluationLink
     *         PredicateNode "XxxDemandGoal"
     *         ListLink (empty)
     *         
     * ReferenceLink
     *     ConceptNode "CurrentDemandGoal"
     *     EvaluationLink
     *         PredicateNode "XxxDemandGoal"
     *         ListLink (empty)
     *
     */
Handle PsiRuleUtil::getPreviousDemandGoal(const AtomSpace & atomSpace, Handle & referenceLink)
{
    // Get the (ConceptNode "PreviousDemandGoal")
    Handle hConceptNode = atomSpace.getHandle
                              ( CONCEPT_NODE,        // Type of the Atom wanted
                                "PreviousDemandGoal" // Name of the Atom wanted
                              );

    if ( hConceptNode == Handle::UNDEFINED ||
         atomSpace.getType(hConceptNode) != CONCEPT_NODE ) {

        logger().warn( "PsiRuleUtil::%s - Found no ConceptNode named 'PreviousDemandGoal'. Return UNDEFINED handle",
                       __FUNCTION__
                     );

        referenceLink = Handle::UNDEFINED; 
        return Handle::UNDEFINED; 
    }

    // Get the HandleSet to ReferenceLink
    std::vector<Handle> referenceLinkSet;

    atomSpace.getHandleSet
                  ( back_inserter(referenceLinkSet), // return value
                    hConceptNode,      // returned link should contain this node
                    REFERENCE_LINK,    // type of the returned link 
                    false              // subclass is not acceptable, 
                                       // i.e. returned link should be exactly of 
                  );                   // type REFERENCE_LINK

    if ( referenceLinkSet.size() != 1 ) {
        logger().error( "PsiRuleUtil::%s - Number of ReferenceLink containing '%s' should be exactly 1, but got %d", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(hConceptNode).c_str(), 
                        referenceLinkSet.size()
                      );

        referenceLink = Handle::UNDEFINED; 
        return Handle::UNDEFINED; 
    }

    // Get the Handle to previous DemandGoal (EvaluationLink)
    referenceLink = referenceLinkSet[0]; 
    Handle hDemandGoalEvaluationLink = atomSpace.getOutgoing(referenceLink, 1); 

    if ( atomSpace.getType(hDemandGoalEvaluationLink) != EVALUATION_LINK ||
         atomSpace.getArity(hDemandGoalEvaluationLink) != 2 ) {
        logger().error( "PsiRuleUtil::%s - The previous DemandGoal should be an EvaluationLink with 2 outgoing. But got '%s'", 
                         __FUNCTION__, 
                         atomSpace.atomAsString(hDemandGoalEvaluationLink).c_str()
                      );

        return Handle::UNDEFINED; 
    }

    // Return the Handle to previous Demand Goal (EvaluationLink)
    return hDemandGoalEvaluationLink; 
}

Handle PsiRuleUtil::getCurrentDemandGoal(const AtomSpace & atomSpace, Handle & referenceLink)
{
    // Get the (ConceptNode "CurrentDemandGoal")
    Handle hConceptNode = atomSpace.getHandle
                              ( CONCEPT_NODE,        // Type of the Atom wanted
                                "CurrentDemandGoal"  // Name of the Atom wanted
                              );

    if ( hConceptNode == Handle::UNDEFINED ||
         atomSpace.getType(hConceptNode) != CONCEPT_NODE ) {

        logger().warn( "PsiRuleUtil::%s - Found no ConceptNode named 'CurrentDemandGoal'. Return UNDEFINED handle",
                       __FUNCTION__
                     );

        referenceLink = Handle::UNDEFINED; 
        return Handle::UNDEFINED; 
    }

    // Get the HandleSet to ReferenceLink
    std::vector<Handle> referenceLinkSet;

    atomSpace.getHandleSet
                  ( back_inserter(referenceLinkSet), // return value
                    hConceptNode,      // returned link should contain this node
                    REFERENCE_LINK,    // type of the returned link 
                    false              // subclass is not acceptable, 
                                       // i.e. returned link should be exactly of 
                  );                   // type REFERENCE_LINK

    if ( referenceLinkSet.size() != 1 ) {
        logger().error( "PsiRuleUtil::%s - Number of ReferenceLink containing '%s' should be exactly 1, but got %d", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(hConceptNode).c_str(), 
                        referenceLinkSet.size()
                      );

        referenceLink = Handle::UNDEFINED; 
        return Handle::UNDEFINED; 
    }

    // Get the Handle to current DemandGoal (EvaluationLink)
    referenceLink = referenceLinkSet[0]; 
    Handle hDemandGoalEvaluationLink = atomSpace.getOutgoing(referenceLink, 1); 

    if ( atomSpace.getType(hDemandGoalEvaluationLink) != EVALUATION_LINK ||
         atomSpace.getArity(hDemandGoalEvaluationLink) != 2 ) {
        logger().error( "PsiRuleUtil::%s - The current DemandGoal should be an EvaluationLink with 2 outgoing. But got '%s'", 
                         __FUNCTION__, 
                         atomSpace.atomAsString(hDemandGoalEvaluationLink).c_str()
                      );

        return Handle::UNDEFINED; 
    }

    // Return the Handle to current Demand Goal (EvaluationLink)
    return hDemandGoalEvaluationLink; 
}

void PsiRuleUtil::setCurrentDemandGoal(AtomSpace & atomSpace, Handle hCurrentlySelectedDemandGoal)
{
    // Get Handles to old ReferenceLink and EvaluationLink
    Handle hCurrentDemandGoalConceptNode = atomSpace.addNode(CONCEPT_NODE, "CurrentDemandGoal"); 
    Handle hPreviousDemandGoalConceptNode = atomSpace.addNode(CONCEPT_NODE, "PreviousDemandGoal"); 

    Handle hOldCurrentDemandGoalReferenceLink = 
        AtomSpaceUtil::getReferenceLink(atomSpace, hCurrentDemandGoalConceptNode); 

    Handle hOldPreviousDemandGoalReferenceLink = 
        AtomSpaceUtil::getReferenceLink(atomSpace, hPreviousDemandGoalConceptNode); 

    Handle hOldCurrentDemandGoalEvaluationLink = 
        hOldCurrentDemandGoalReferenceLink != Handle::UNDEFINED ?
            atomSpace.getOutgoing(hOldCurrentDemandGoalReferenceLink, 1): 
            Handle::UNDEFINED; 

    Handle hOldPreviousDemandGoalEvaluationLink = 
        hOldPreviousDemandGoalReferenceLink != Handle::UNDEFINED ?
            atomSpace.getOutgoing(hOldPreviousDemandGoalReferenceLink, 1): 
            Handle::UNDEFINED; 

    // If currently selected Demand Goal doesn't change, return immediately
    if (hCurrentlySelectedDemandGoal == hOldCurrentDemandGoalEvaluationLink)
        return; 

    // Try to delete old ReferenceLink containing previous/ current DemandGoal 
    if ( hOldCurrentDemandGoalReferenceLink != Handle::UNDEFINED )
        atomSpace.removeAtom( hOldCurrentDemandGoalReferenceLink );

    if ( hOldPreviousDemandGoalReferenceLink != Handle::UNDEFINED ) 
        atomSpace.removeAtom( hOldPreviousDemandGoalReferenceLink );

    // Create ReferenceLink containing previous DemandGoal
    std::vector<Handle> outgoingSet;

    if ( hOldCurrentDemandGoalEvaluationLink != Handle::UNDEFINED ) {
        outgoingSet.clear(); 
        outgoingSet.push_back( atomSpace.addNode(CONCEPT_NODE, "PreviousDemandGoal") ); 
        outgoingSet.push_back( hOldCurrentDemandGoalEvaluationLink );

        atomSpace.addLink(REFERENCE_LINK, outgoingSet); 
    }

    // Create ReferenceLink containing current DemandGoal
    if ( hCurrentlySelectedDemandGoal != Handle::UNDEFINED )  {
        outgoingSet.clear(); 
        outgoingSet.push_back( atomSpace.addNode(CONCEPT_NODE, "CurrentDemandGoal") ); 
        outgoingSet.push_back( hCurrentlySelectedDemandGoal ); 

        atomSpace.addLink(REFERENCE_LINK, outgoingSet); 
    }        
}

