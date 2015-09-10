/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRelationUpdaterAgent.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-03-07
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
#include <boost/tokenizer.hpp>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>

#include "PsiRelationUpdaterAgent.h"
#include "PsiRuleUtil.h"


using namespace opencog;

PsiRelationUpdaterAgent::~PsiRelationUpdaterAgent()
{

}

PsiRelationUpdaterAgent::PsiRelationUpdaterAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

void PsiRelationUpdaterAgent::init()
{
    logger().debug( "PsiRelationUpdaterAgent::%s - Initializing the Agent [ cycle = %d ]",
                    __FUNCTION__, this->cycleCount);

    // Get novelty level parameters from configuration file
    // TODO: this should be set for multiple agents and would be preferable
    // if it is defined in the atomspace.
    this->noveltyInitLevel = config().get_double("PSI_NOVELTY_INIT_LEVEL");
    this->noveltyThreshold = config().get_double("PSI_NOVELTY_THRESHOLD");
    this->noveltyResetThreshold = config().get_double("PSI_NOVELTY_RESET_THRESHOLD");
    this->noveltyDecayFactor = config().get_double("PSI_NOVELTY_DECAY_FACTOR");

    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    // Get relation names from the configuration file
    std::string relationNames = config()["PSI_RELATIONS"];

    boost::char_separator<char> sep(", ");
    boost::tokenizer< boost::char_separator<char> > relationNamesTok (relationNames, sep);

    // Process Relations one by one
    for ( boost::tokenizer< boost::char_separator<char> >::iterator iRelationName = relationNamesTok.begin();
          iRelationName != relationNamesTok.end();
          iRelationName ++ ) {

        // Get corresponding PredicateNode
        Handle hRelationPredicateNode = atomSpace.get_handle(PREDICATE_NODE, *iRelationName);

        if ( hRelationPredicateNode==opencog::Handle::UNDEFINED ) {
            logger().warn("PsiRelationUpdaterAgent::%s - Failed to find PredicateNode for relation '%s' [ cycle = %d]",
                          __FUNCTION__, (*iRelationName).c_str(), this->cycleCount);
            continue;
        }

        // Get all the EvaluationLink containing hRelationPredicateNode
        std::vector<Handle> relationEvaluationLinkSet;

        hRelationPredicateNode->getIncomingSetByType(
                                back_inserter(relationEvaluationLinkSet),
                                EVALUATION_LINK,
                                false
                              );

        if ( relationEvaluationLinkSet.empty() ) {
            logger().warn("PsiRelationUpdaterAgent::%s - Failed to find EvaluationLink for relation '%s' [ cycle = %d]",
                          __FUNCTION__,
                          (*iRelationName).c_str(),
                          this->cycleCount
                         );
            continue;

        }

        // Process EvaluationLinks one by one
        for (Handle hRelationEvaluationLink : relationEvaluationLinkSet) {

            // Get all the ImplicatonLinks containing hRelationEvaluationLink
            std::vector<Handle> relationImplicationLinkSet;

            hRelationEvaluationLink->getIncomingSetByType(
                                    back_inserter(relationImplicationLinkSet),
                                    IMPLICATION_LINK,
                                    false
                                  );

            // Process ImplicatonLinks one by one
            // If it is a Psi Rule with NULL_ACTION, append it to instantRelationRules
            for (Handle hImplicationLink : relationImplicationLinkSet) {

                // Split the Psi Rule into Goal, Action and Preconditions
                Handle hGoalEvaluationLink, hActionExecutionLink, hPreconditionAndLink;

                if ( PsiRuleUtil::isHandleToPsiRule(atomSpace, hImplicationLink) ) {

                    PsiRuleUtil::splitPsiRule( atomSpace,
                                               hImplicationLink,
                                               hGoalEvaluationLink,
                                               hActionExecutionLink,
                                               hPreconditionAndLink
                                             );

                    // Check if this Psi Rule contains a NULL_ACTION
                    // About NULL_ACTION, please refer to
                    // './opencog/embodiment/rules_core.scm'
                    Handle hActionGroundedSchemaNode = atomSpace.get_outgoing(hActionExecutionLink, 0);

                    if ( atomSpace.get_name(hActionGroundedSchemaNode) == "DoNothing" ) {

                        this->instantRelationRules.push_back(hImplicationLink);

                        logger().debug("PsiRelationUpdaterAgent::%s - Found an instant (with NULL_ACTION) Psi Rule '%s' for relation '%s' [ cycle = %d ]",
                                       __FUNCTION__,
                                       atomSpace.atom_as_string(hImplicationLink).c_str(),
                                       (*iRelationName).c_str(),
                                       this->cycleCount
                                      );
                   }// if

                }// if

            }// foreach

        }// foreach

    }// for

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

Handle PsiRelationUpdaterAgent::getRelationEvaluationLink(AtomSpace & atomSpace,
                                                          const std::string & relationName,
                                                          Handle agentHandle,
                                                          Handle entityHandle
                                                         )
{
    // Get the Handle to relation (PredicateNode)
    Handle relationPredicateHandle = atomSpace.get_handle(PREDICATE_NODE, relationName);

    if (relationPredicateHandle == Handle::UNDEFINED) {
        logger().warn( "PsiRelationUpdaterAgent::%s - Failed to find the PredicateNode for relation '%s'. Don't worry it will be created automatically. [ cycle = %d ]",
                        __FUNCTION__,
                        relationName.c_str(),
                        this->cycleCount
                      );

        relationPredicateHandle = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, relationName, false);
    }

    // Get the Handle to ListLink that contains both pet and entity
    std::vector<Handle> listLinkOutgoing;

    listLinkOutgoing.push_back(agentHandle);
    listLinkOutgoing.push_back(entityHandle);

    Handle listLinkHandle = atomSpace.get_handle(LIST_LINK, listLinkOutgoing);

    if (listLinkHandle == Handle::UNDEFINED) {
        logger().debug( "PsiRelationUpdaterAgent::%s - Failed to find the ListLink containing both entity ( id = '%s' ) and pet ( id = '%s' ). Don't worry, it will be created automatically [ cycle = %d ].",
                        __FUNCTION__,
                        atomSpace.get_name(entityHandle).c_str(),
                        atomSpace.get_name(agentHandle).c_str(),
                        this->cycleCount
                      );

        AtomSpaceUtil::addLink(atomSpace, LIST_LINK, listLinkOutgoing, false);
    }

    // Get the Handle to EvaluationLink holding the relation between the pet and the entity
    std::vector<Handle> evaluationLinkOutgoing;

    evaluationLinkOutgoing.push_back(relationPredicateHandle);
    evaluationLinkOutgoing.push_back(listLinkHandle);

    Handle evaluationLinkHandle = atomSpace.get_handle(EVALUATION_LINK, evaluationLinkOutgoing);

    if (evaluationLinkHandle == Handle::UNDEFINED) {
        logger().debug( "PsiRelationUpdaterAgent::%s - Failed to find the EvaluationLink holding the '%s' relation between the pet ( id = '%s' ) and the entity ( id = '%s' ). Don't worry it would be created automatically [ cycle = %d ].",
                        __FUNCTION__,
                        relationName.c_str(),
                        atomSpace.get_name(agentHandle).c_str(),
                        atomSpace.get_name(entityHandle).c_str(),
                        this->cycleCount
                     );

        evaluationLinkHandle = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evaluationLinkOutgoing, false);
    }

    logger().debug("PsiRelationUpdaterAgent::%s - Get the  EvaluationLink: %s [cycle = %d]",
                    __FUNCTION__,
                    atomSpace.atom_as_string(evaluationLinkHandle).c_str(),
                    this->cycleCount
                   );

    return evaluationLinkHandle;
}

Handle PsiRelationUpdaterAgent::getEntityHandle(AtomSpace& atomSpace, const std::string & entityName)
{
    // TODO: What is responsible for creating these handles to entities?
    std::vector<Handle> entityHandleSet;

    classserver().foreachRecursive(
        [&](Type t)->void {
            Handle h(atomSpace.get_handle(t, entityName));
            if (h) entityHandleSet.push_back(h); }, OBJECT_NODE);

    if ( entityHandleSet.size() != 1 ) {
        logger().warn( "PsiRelationUpdaterAgent::%s - The number of entity ( id = '%s' ) registered in AtomSpace should be exactly 1. Got %d [ cycle = %d ]",
                       __FUNCTION__,
                       entityName.c_str(),
                       entityHandleSet.size(),
                       this->cycleCount
                     );
        return opencog::Handle::UNDEFINED;
    }

    return  entityHandleSet[0];
}

/*
void PsiRelationUpdaterAgent::updateEntityNovelty(opencog::CogServer * server)
{
    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    //  This is a temporary hack for defining a procssing-agent
    const std::string & agentId = "PsiAgent1";

    // Get Handle to the pet/agent
    Handle agentHandle = AtomSpaceUtil::getAgentHandle( atomSpace, agentId);

    if ( agentHandle == Handle::UNDEFINED ) {
        logger().warn("PsiRelationUpdaterAgent::%s - Failed to get the handle to the pet ( id = '%s' ) [ cycle = %d ]",
                        __FUNCTION__,
                        agentId.c_str(),
                        this->cycleCount
                     );
        return;
    }

    // Get SpaceMap
    const SpaceServer::SpaceMap & spaceMap  = spaceServer().getLatestMap();

    // Get all the entities
    std::vector<std::string> entities;
    spaceMap.findAllEntities(back_inserter(entities));

    // Get all the entities that next to the pet
    std::vector<std::string> entitiesNextToPet;

    for (std::string & entityId : entities) {
        // Get handle to entity
        Handle entityHandle = this->getEntityHandle(atomSpace, entityId);

        if ( entityHandle == opencog::Handle::UNDEFINED )
            continue;

        // Check if the entity is next to the pet
        if ( AtomSpaceUtil::isPredicateTrue(atomSpace, "next", entityHandle, agentHandle) ) {
            entitiesNextToPet.push_back(entityId);

            logger().debug("PsiRelationUpdaterAgent::%s - Entity (id = '%s') is next to the pet [cycle = %d]",
                            __FUNCTION__,
                            entityId.c_str(),
                            this->cycleCount
                          );
        }// if
    }// foreach

    // Create (entityId, novelty level) pairs if necessary
    for (std::string & entityId : entitiesNextToPet) {
        if ( entityNovelty.find(entityId) == entityNovelty.end() ) {

            // Create a new (entityId, novelty level) pair
            Novelty novelty(this->noveltyInitLevel);
            std::pair<std::string, Novelty> entityNoveltyPair(entityId, novelty);

            // Insert the new (entityId, novelty level) pair
            this->entityNovelty.insert(entityNoveltyPair);

            logger().debug("PsiRelationUpdaterAgent::%s - Create a (entityId, novelty) pair for entity '%s' [cycle = %d]",
                           __FUNCTION__,
                           entityId.c_str(),
                           this->cycleCount
                          );

        }// if
    }// foreach

    // Decrease all the novelty levels of the entities
    std::vector <std::string> entitiesCanBeRemoved;
    std::map<std::string, Novelty>::iterator iEntityNovelty;

    for ( iEntityNovelty = this->entityNovelty.begin();
           iEntityNovelty != this->entityNovelty.end();
           ++iEntityNovelty) {

        // Update the novelty levels of the entities
        std::string entityId = iEntityNovelty->first;
        Novelty & novelty = iEntityNovelty->second;

        novelty.updateNoveltyLevel(this->noveltyInitLevel, this->noveltyDecayFactor);

        // If the novelty level is below reset threshold, then it should be reset or removed,
        // which depends on whether the pet encounters (i.e. next to) the corresponding entity now
        if ( novelty.canBeReset(this->noveltyResetThreshold) ) {

            if ( std::find(entitiesNextToPet.begin(), entitiesNextToPet.end(), entityId) ==
                 entitiesNextToPet.end() ) {

                entitiesCanBeRemoved.push_back(entityId);

                logger().debug("PsiRelationUpdaterAgent::%s - novelty of entity '%s' will be removed [cycle = %d]",
                               __FUNCTION__,
                               entityId.c_str(),
                               this->cycleCount
                              );
            }
            else {
                novelty.resetNoveltyLevel(this->noveltyInitLevel);

                logger().debug("PsiRelationUpdaterAgent::%s - novelty level of entity '%s' has been reset to initial value %f [cycle = %d]",
                               __FUNCTION__,
                               entityId.c_str(),
                               this->noveltyInitLevel,
                               this->cycleCount
                              );
            }

        }// if
    }// foreach

    // Remove (entityId, novelty level) pairs if necessary
    for (std::string & entityIdRemoved : entitiesCanBeRemoved) {
        this->entityNovelty.erase(entityIdRemoved);
    }

    // Update the truth value of 'curious_about' relation based on novelty level
    bool bHasNovelty = false;

    for ( iEntityNovelty = this->entityNovelty.begin();
           iEntityNovelty != this->entityNovelty.end();
           ++iEntityNovelty) {

        // Get entitId and novelty instance
        std::string entityId = iEntityNovelty->first;
        Novelty & novelty = iEntityNovelty->second;

        // Get handle to entity
        Handle entityHandle = this->getEntityHandle(atomSpace, entityId);

        if ( entityHandle == opencog::Handle::UNDEFINED )
            continue;

        Handle hRelationEvaluationLink = this->getRelationEvaluationLink(
                                        atomSpace, "curious_about", agentHandle,
                                        entityHandle);

        // Update the truth value
        if ( novelty.isNovel(this->noveltyThreshold) ) {

            TruthValuePtr stvTrue = SimpleTruthValue::createTV(1, 1);
            atomSpace.setTV(hRelationEvaluationLink, stvTrue);

            bHasNovelty = true;

            logger().debug("PsiRelationUpdaterAgent::%s pet is 'curious_about' entity '%s' (novelty level = %f) [ cycle = %d ]",
                           __FUNCTION__,
                           entityId.c_str(),
                           novelty.getNoveltyLevel(),
                           this->cycleCount
                          );

        } else {
            TruthValuePtr stvFalse = SimpleTruthValue::createTV(0, 0);
            atomSpace.setTV(hRelationEvaluationLink, stvFalse);

            logger().debug("PsiRelationUpdaterAgent::%s pet is not longer 'curious_about' entity '%s' (novelty level = %f) [ cycle = %d ]",
                           __FUNCTION__,
                           entityId.c_str(),
                           novelty.getNoveltyLevel(),
                           this->cycleCount
                          );
        }// if

    }// foreach

    // Update the truth value of 'has_novelty' predicates in AtomSpace
    // TODO: 'has_novelty' predicate is probably obsolete

    if (bHasNovelty)
        AtomSpaceUtil::setPredicateValue(atomSpace, "has_novelty", SimpleTruthValue::createTV(1.0f, 0.0f), agentHandle);
    else
        AtomSpaceUtil::setPredicateValue(atomSpace, "has_novelty", SimpleTruthValue::createTV(0.0f, 0.0f), agentHandle);
}
*/
/*
void PsiRelationUpdaterAgent::updateEntityRelation(AtomSpace & atomSpace,
                                                   Handle agentHandle,
                                                   Procedure::ProcedureInterpreter & procedureInterpreter,
                                                   const Procedure::ProcedureRepository & procedureRepository)

{
    // Shuffle all the instant Psi Rules about Relation
    //
    // Note: Why? Because some Relations rely on other Relations, like multi-steps planning.
    //       For example, 'familiar_with' is the former step before 'know' and
    //       'curious_about' is the very basic step for other relations.
    //
    //       Randomly shuffle these instant relation rules before processing
    //       would give the pet/agent the chance to consider multi-step
    //       relations in different combinations.
    std::random_shuffle( this->instantRelationRules.begin(), this->instantRelationRules.end() );

    logger().debug("PsiRelationUpdaterAgent::%s - The list of instant relation rules has been randomly shuffled [cycle = %d]",
                   __FUNCTION__,
                   this->cycleCount
                  );

    // This vector has all possible objects/avatars ids to replace wildcard in Psi Rules
    //
    // TODO: We would not use wildcard later,
    //       and the vector here should be used to replace VariableNode in Psi Rules with ForAllLink
    std::vector<std::string> varBindCandidates;

    // Process instant relation rules one by one
    for (Handle hInstantRelationRule : this->instantRelationRules) {

        logger().debug("PsiRelationUpdaterAgent::%s - Going to check the preconditions of instant relation rule: %s [cycle = %d]",
                       __FUNCTION__,
                       atomSpace.atom_as_string(hInstantRelationRule).c_str(),
                       this->cycleCount
                      );

        // If all the instant relation rules are satisfied,
        // set the truth value of corresponding EvaluationLinks to true
        if ( PsiRuleUtil::allPreconditionsSatisfied( atomSpace,
                                                     procedureInterpreter,
                                                     procedureRepository,
                                                     hInstantRelationRule,
                                                     varBindCandidates
                                                   ) ) {

            // Split the Psi Rule into Goal, Action and Preconditions
            Handle hGoalEvaluationLink, hActionExecutionLink, hPreconditionAndLink;

            PsiRuleUtil::splitPsiRule( atomSpace,
                                       hInstantRelationRule,
                                       hGoalEvaluationLink,
                                       hActionExecutionLink,
                                       hPreconditionAndLink
                                     );

            // Get relation name
            Handle hRelationPredicateNode = atomSpace.get_outgoing(hInstantRelationRule, 0);
            std::string relationName = atomSpace.get_name(hRelationPredicateNode);

            // Set all the truth value of all the EvaluationLinks containing this relation to true
            for (std::string & entityId : varBindCandidates) {
                // Get handle to entity
                Handle entityHandle = this->getEntityHandle(atomSpace, entityId);

                if ( entityHandle == opencog::Handle::UNDEFINED )
                    continue;

                Handle hRelationEvaluationLink = this->getRelationEvaluationLink( atomSpace,
                                                                                );
                // Set the truth value to true
                // TODO: Actually, this not so correct, the truth value should not be a constant (1, 1)
                //       We should give the pet the ability to distinguish the intensity of relation,
                //       such as friend, good friend and best friend
                TruthValuePtr stvTrue = SimpleTruthValue::createTV(1, 1);
                atomSpace.setTV(hRelationEvaluationLink, stvTrue);

                logger().debug("PsiRelationUpdaterAgent::%s Updated the trutu value of '%s' [ cycle = %d ]",
                               __FUNCTION__,
                               atomSpace.atom_as_string(hRelationEvaluationLink).c_str(),
                               this->cycleCount
                              );

            }// foreach

        }// if

    }// foreach

}
*/
void PsiRelationUpdaterAgent::run()
{
    this->cycleCount ++;

    logger().debug( "PsiRelationUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // Get AtomSpace
    AtomSpace& atomSpace = _cogserver.getAtomSpace();

    // Get ProcedureInterpreter
    //Procedure::ProcedureInterpreter & procedureInterpreter = oac->getProcedureInterpreter();

    // Get Procedure repository
    //const Procedure::ProcedureRepository & procedureRepository = oac->getProcedureRepository();

    // Get agentId and petName
    // const std::string & petName = oac->getPet().get_name();
    const std::string & agentId = "PsiAgent1";

    // Get Handle to the pet
    Handle agentHandle = AtomSpaceUtil::getAgentHandle( atomSpace, agentId);

    if ( agentHandle == Handle::UNDEFINED ) {
        logger().warn("PsiRelationUpdaterAgent::%s - Failed to get the handle to the pet ( id = '%s' ) [ cycle = %d ]",
                        __FUNCTION__,
                        agentId.c_str(),
                        this->cycleCount
                     );
        return;
    }

    // Decide whether to update relations during this cognitive cycle (controlled by the modulator 'SecuringThreshold')
    /* float securingThreshold =
    AtomSpaceUtil::getCurrentModulatorLevel(atomSpace,
                                            SECURING_THRESHOLD_MODULATOR_NAME);
    */

// TODO: Uncomment the lines below once finish testing
//    if ( randGen().randfloat() < securingThreshold )
//    {
//        logger().debug(
//                "PsiRelationUpdaterAgent::%s - Skip updating the relations for this cognitive cycle [ cycle = %d ] ",
//                       __FUNCTION__,
//                       this->cycleCount
//                      );
//        return;
//    }

    // Initialize entity, relation lists etc.
    if ( !this->bInitialized )
        this->init();

    // Deal with 'curious_about' relation
    // TODO: relations are defined in terms of spatial-relations but this
    // doesn't make sense in a situation where the relation is that b/n
    // a mind-process/agent (like the dialogue system) and ???
    //
    // Maybe the relations should be defined in as part of a sub-goal of perception
    //this->updateEntityNovelty(&_cogserver);

    // Update other relations
    // Disabled for similar reason as updateEntityNovelty above
    //this->updateEntityRelation(atomSpace, agentHandle, procedureInterpreter, procedureRepository);
}
