/*
 * TODO: 
 *
 * 1. Remove duplicated functions in AtomSpaceUtil 
 * 2. Make All the PsiActionSelectionAgent use PsiRuleUtil instead of AtomSpaceUtil
 *
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRuleUtil.h
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

#ifndef PSIRULEUTIL_H
#define PSIRULEUTIL_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/Procedure/ProcedureInterpreter.h>
#include <opencog/embodiment/Control/Procedure/ProcedureRepository.h>

namespace opencog { namespace oac {

/**
 * @class
 *
 * @brief Class with utility methods for related to Psi rules manipulation
 *
 * Each Rule here means a cognitive schematic, that is 
 *     Contex & Procedure ==> Goal
 *
 * Rules here has nothing to do with that in PLN, so don't confuse them!
 *
 * An OpenPsi Rule is represented in AtomSpace as below: 
 *
 * Note: AtTimeLink is missing currently
 *       ImplicationLink is used instead of PredictiveImplicationLink, since PLN doesn't support it while implementation
 *
 *     PredictiveImplicationLink
 *         AndLink
 *             AndLink
 *                 EvaluationLink
 *                     GroundedPredicateNode "precondition_1_name"
 *                     ListLink
 *                         Node:arguments
 *                         ...
 *                 EvaluationLink
 *                     PredicateNode         "precondition_2_name"
 *                     ListLink
 *                         Node:arguments
 *                         ...
 *                 ...
 *                            
 *             ExecutionLink
 *                 GroundedSchemaNode "schema_name"
 *                 ListLink
 *                     Node:arguments
 *                     ...
 *    
 *         EvaluationLink
 *             (SimpleTruthValue indicates how well the demand is satisfied)
 *             (ShortTermInportance indicates the urgency of the demand)
 *             PredicateNode: "goal_name" 
 *             ListLink
 *                 Node:arguments
 *                 ...
 *
 * For each Rule, there's only a Goal, an Action and a bunch of Preconditions. 
 * And all these Preconditions should be grouped in an AndLink.
 * If you want to use OrLink, then just split the Rule into several Rules.
 * For the efficiency and simplicity of the planer (backward chainging), NotLink is forbidden currently.  
*/
class PsiRuleUtil
{

public:
    /**
     * Split the Psi Rule into three components: Goal, Action and Preconditions
     *
     * @param atomSpace                    The AtomSpace
     * @param hPsiRule                     The Handle to the Psi Rule 
     * @param hGoalEvaluationLink          The Handle to the Goal (EvaluationLink) 
     * @param hActionExecutionLink         The Handle to the Action (ExecutionLink)
     * @param hPreconditionEvaluatioLinks  Handles to the Preconditions (EvaluationLinks)
     *
     * @return bool True if split the Psi Rule successfully, false if fails
     */
    static bool splitPsiRule(const AtomSpace & atomSpace,
                             const Handle hPsiRule, 
                             Handle & hGoalEvaluationLink,
                             Handle & hActionExecutionLink, 
                             Handle & hPreconditionAndLink
                            );
    /**
     * Check if the given Handle is a Psi Rule.
     *
     * @return  true if the given Handle indicating a Psi Rule
     *
     * @note  For the format of Psi Rules, please refer to "./opencog/embodiment/rules_core.scm"
     */
    static bool isHandleToPsiRule(const AtomSpace & atomSpace, Handle h);

    /**
     * Transfer the format of arguments within given ListLink to combo. 
     * 
     * @param  atomSpace          The AtomSpace
     * @parem  hListLink          Handle to ListLink that contains arguments
     * @param  varBindCandidates  All the possible variable bindings for the selected Psi Rule
     * @param  schemaArguments    Return the arguments that would be used while executing the combo procedure
     *
     * @return  The number of arguments got
     */
    static bool getSchemaArguments(const AtomSpace & atomSpace, 
                                   Handle hListLink, 
                                   const std::vector<std::string> & varBindCandidates, 
                                   std::vector <combo::vertex> & schemaArguments);

    /**
     * Initialize all the possible variable bindings in Psi Rule with all the entities the pet encounters
     *
     * @param atomSpace          The AtomSpace
     * @param varBindCandidates  All the possible variable bindings 
     */
    static void initVarBindCandidates(const AtomSpace & atomSpace, std::vector<std::string> & varBindCandidates);
 
    /**
     * Initialize the unifier with the given all the possible variable bindings
     *
     * @param unifier            
     * @param varBindCandidates  All the possible variable bindings
     *
     * @note  The combo::variable_unifier inherits from the type 'std::map<std::string, bool>', 
     *        recording each variable binding and the corresponding state (valid/invalid). 
     *        (see also './opencog/comboreduct/combo/variable_unifier.h')
     *
     *        We usually call this function before running a combo procedure that needs a unifier. 
     */
    static void initUnifier(combo::variable_unifier & unifier, const std::vector<std::string> & varBindCandidates);

    /**
     * Updating the possible variable bindings in Psi Rule, based on the result return by combo interpreter 
     *
     * @param unifier            The unifier after executing a combo function
     * @param varBindCandidates  All the possible variable bindings
     *
     * @note  We usually call this function after running a combo procedure that needs a unifier. 
     *
     *        When you call a combo procedure with a unifier, the combo procedure interpreter would update 
     *        the state of each possible variable binding automatically.
     *
     *        So after running the combo procedure, you can usually call this function,
     *        which would find variable bindings that are actually valid by simply checking their states. 
     */
    static void updateVarBindCandidates(const combo::variable_unifier & unifier, std::vector<std::string> & varBindCandidates);

    /**
     * Check if the given Precondition is satisfied. 
     *
     * @param  atomSpace             The AtomSpace
     * @param  procedureInterpreter  It is responsible for executing a given procedure, such as combo procedure
     * @param  procedureRepository   Where to search the procedure given its name
     * @param  hPrecondition         Handle to the Precondition, i.e. an EvaluationLink
     * @param  unifier               The combo interpreter would update the states of all the possible variable bindings 
     *                               within the unifier
     *
     * @return  true if the given Precondition is satisfied, otherwise returns false
     *
     * @note  If the EvaluationLink(hPrecondition)) contains a PredicateNode, 
     *        then we simply check the truth value of the EvaluationLink. 
     *
     *        If the EvaluationLink holds a GroundedPredicateNode, 
     *        we would run the corresponding combo procedure firstly, and then judge based on the execution result. 
     */
    static bool isSatisfied(const AtomSpace & atomSpace, 
                            Procedure::ProcedureInterpreter & procedureInterpreter, 
                            const Procedure::ProcedureRepository & procedureRepository, 
                            Handle hPrecondition, 
                            combo::variable_unifier & unifier);

    /**
     * Slightly easier-to-use version of the above, which creates unifier itself.
     */
    static bool isSatisfied(const AtomSpace & atomSpace,
                            Procedure::ProcedureInterpreter & procedureInterpreter,
                            const Procedure::ProcedureRepository & procedureRepository,
                            Handle hPrecondition);

    /**
     * Check if all the Preconditions of a given Psi Rule are satisfied. 
     * It is used by 'PsiActionSelectionAgent::pickUpPsiRule' method.
     *
     * @param  atomSpace             The AtomSpace
     * @param  procedureInterpreter  It is responsible for executing a given procedure, such as combo procedure
     * @param  procedureRepository   Where to search the procedure given its name
     * @param  hPsiRule              Handle to the a Psi Rule, i.e. an ImplicationLink
     * @param  varBindCandidates     All the possible variable bindings for the selected Psi Rule
     *
     * @return true if all the preconditions of the given Psi Rule are satisfied, otherwise return false                              
     */
    static bool allPreconditionsSatisfied(const AtomSpace & atomSpace, 
                                          Procedure::ProcedureInterpreter & procedureInterpreter, 
                                          const Procedure::ProcedureRepository & procedureRepository, 
                                          Handle hPsiRule, 
                                          std::vector<std::string> & varBindCandidates);

    /**
     * Apply the given Psi Rule. 
     *
     * @param  atomSpace             The AtomSpace
     * @param  procedureInterpreter  It is responsible for executing a given procedure, such as combo procedure
     * @param  procedureRepository   Where to search the procedure given its name
     * @param  psiRules
     * @param  varBindCandidates     All the possible variable bindings for the selected Psi Rule
     *
     * @return  true if success. 
     *
     * @note  Since each Psi Rule associates with a single Action, 
     *        applying a Psi Rule equals executing the corresponding Action.
     *
     *        While this method simply runs the Action, it will not get and analyze the result of the execution, 
     *        because it may take some time to finish the execution. 
     *
     *        'run' method is responsible for dealing with the result of execution during next "cognitive cycle" 
     *        after calling the 'applyPsiRule' method. 
     */
    static Procedure::RunningProcedureID applyPsiRule(const AtomSpace & atomSpace, 
                                                      Procedure::ProcedureInterpreter & procedureInterpreter, 
                                                      const Procedure::ProcedureRepository & procedureRepository, 
                                                      Handle hPsiRule,
                                                      const std::vector<std::string> & varBindCandidates);

    /**
     * Get/ set previous/ current Demand Goal (EvaluationLink)
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
    static Handle getPreviousDemandGoal(AtomSpace & atomSpace, Handle & referenceLink); 

    static Handle getCurrentDemandGoal(AtomSpace & atomSpace, Handle & referenceLink); 

    static void setCurrentDemandGoal(AtomSpace & atomSpace, Handle hCurrentlySelectedDemandGoal); 
}; // class

} } // namespace opencog::oac

#endif
