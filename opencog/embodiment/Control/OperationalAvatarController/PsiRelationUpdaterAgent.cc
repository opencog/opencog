/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRelationUpdaterAgent.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-02-06
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
#include "PsiRelationUpdaterAgent.h"

#include<boost/tokenizer.hpp>

using namespace OperationalAvatarController;

extern int currentDebugLevel;

PsiRelationUpdaterAgent::~PsiRelationUpdaterAgent()
{

}

PsiRelationUpdaterAgent::PsiRelationUpdaterAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

void PsiRelationUpdaterAgent::init(opencog::CogServer * server) 
{
    logger().debug( "PsiRelationUpdaterAgent::%s - Initializing the Agent [ cycle = %d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    const AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
//    const std::string & petId = oac->getPet().getPetId();

    // Clear old relation list
    this->relationList.clear(); 

    // Get relation names from the configuration file
    std::string relationNames = config()["PSI_RELATIONS"];

    boost::tokenizer<> relationNamesTok (relationNames);

    for ( boost::tokenizer<>::iterator iRelationName = relationNamesTok.begin();
          iRelationName != relationNamesTok.end();
          iRelationName ++ ) {

        this->relationList.push_back(*iRelationName);

        logger().debug("PsiRelationUpdaterAgent::%s - Found relation '%s' from configuration file [ cycle = %d ].", 
                        __FUNCTION__, 
                        (*iRelationName).c_str(), 
                        this->cycleCount
                      );

    }

    logger().debug("PsiRelationUpdaterAgent::%s - Got %d relations from configuration file [ cycle = %d ].", 
                    __FUNCTION__, 
                    this->relationList.size(), 
                    this->cycleCount
                  );

    // Reset iterator to current processing relation
    this->iCurrentRelation = this->relationList.begin(); 

    // Clear old entity list
    this->entityList.clear(); 

    // Update entity list
    const SpaceServer::SpaceMap& spaceMap = atomSpace.getSpaceServer().getLatestMap(); 
    spaceMap.findAllEntities(back_inserter(this->entityList));

    logger().debug("PsiRelationUpdaterAgent::%s - Got %d entities from space map [ cycle = %d ].", 
                    __FUNCTION__, 
                    this->entityList.size(), 
                    this->cycleCount
                  );
  
    // Reset iterator to current processing entity
    this->iCurrentEntity = this->entityList.begin();

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

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiRelationUpdaterAgent::allocInferSteps()
{
    // Reset steps (inference resource) 
    // TODO: Modulators shall have impact on these values (By Zhenhua Cai, on 2011-02-05)
    //
    this->totalRemainSteps = 2000; 
    this->singleEntityRelationMaxSteps = 150;    

    logger().debug("PsiRelationUpdaterAgent::%s - Total inference steps remain = %d, Single entity relation maximum steps = %d [ cycle = %d ].", 
                    __FUNCTION__, 
                    this->totalRemainSteps, 
                    this->singleEntityRelationMaxSteps, 
                    this->cycleCount
                  );
}

void PsiRelationUpdaterAgent::updateEntityRelation(opencog::CogServer * server, Handle relationEvaluationLink,
                                                   int & steps)
{
    // The implementation below borrowed many source code from the function opencog::pln::infer ("PLNModule.cc")
    
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Create BITNodeRoot for the Goal (Target)
    pHandleSeq fakeHandles = ASW()->realToFakeHandle(relationEvaluationLink);

    pHandle fakeHandle = fakeHandles[0];

    logger().debug("PsiRelationUpdaterAgent::%s - Initialize ASW OK [ cycle = %d ]", 
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

    logger().debug("PsiRelationUpdaterAgent::%s - BITNodeRoot init ok [ cycle = %d ].", 
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

    // Print the result to the screen for debugging
    state->printResults();

    // Log information
    if ( result.empty() ) {
        logger().warn( "PsiRelationUpdaterAgent::%s - Failed to update the truth value of %s by PLN [ cycle = %d ].", 
                        __FUNCTION__, 
                        atomSpace.atomAsString(relationEvaluationLink).c_str(), 
                        this->cycleCount
                      );
    }
    else {
        logger().debug("PsiRelationUpdaterAgent::%s - Update the truth value of %s successfully by PLN [ cycle = %d ].", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(relationEvaluationLink).c_str(), 
                       this->cycleCount
                      );

    }// if
}

Handle PsiRelationUpdaterAgent::getRelationEvaluationLink(opencog::CogServer * server, 
                                                          const std::string & relationName,
                                                          Handle entityHandle, 
                                                          Handle petHandle)
{
    // TODO: Shall we create these Nodes and Links if we failed to find them in the AtomSpace?
    //       Or maybe we should create them in init method of the mind agent, 
    //       or in scheme scripts ('pet_rules.scm') 
    //       [By Zhenhua Cai, on 2011-02-06]
    //      

    // Get the AtomSpace
    AtomSpace & atomSpace = * ( server->getAtomSpace() ); 

    // Get the Handle to relation (PredicateNode)
    Handle relationPredicateHandle = atomSpace.getHandle(PREDICATE_NODE, relationName);

    if (relationPredicateHandle == Handle::UNDEFINED) {
        logger().warn(
            "PsiRelationUpdaterAgent::%s - Failed to find the PredicateNode for relation '%s' [ cycle = %d ].", 
                        __FUNCTION__, 
                        relationName.c_str(), 
                        this->cycleCount
                      );

        return opencog::Handle::UNDEFINED; 
    }

    // Get the Handle to ListLink that contains both entity and pet
    std::vector<Handle> listLinkOutgoing;

    listLinkOutgoing.push_back(entityHandle);
    listLinkOutgoing.push_back(petHandle);

    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, listLinkOutgoing);

    if (listLinkHandle == Handle::UNDEFINED) {
        logger().warn( "PsiRelationUpdaterAgent::%s - Failed to find the ListLink containing both entity ( id = '%s' ) and pet ( id = '%s' ) [ cycle = %d ].", 
                        __FUNCTION__, 
                        atomSpace.getName(entityHandle).c_str(),
                        atomSpace.getName(petHandle).c_str(),
                        this->cycleCount
                      );

        return opencog::Handle::UNDEFINED; 
    } 

    // Get the Handle to EvaluationLink holding the relation between the pet and the entity
    std::vector<Handle> evaluationLinkOutgoing; 

    evaluationLinkOutgoing.push_back(relationPredicateHandle);
    evaluationLinkOutgoing.push_back(listLinkHandle);

    Handle evaluationLinkHandle = atomSpace.getHandle(EVALUATION_LINK, evaluationLinkOutgoing);

    if (evaluationLinkHandle == Handle::UNDEFINED) {
        logger().warn( "PsiRelationUpdaterAgent::%s - Failed to find the EvaluationLink holding the '%s' relation between the pet ( id = '%s' ) and the entity ( id = '%s' ) [ cycle = %d ].", 
                        __FUNCTION__, 
                        relationName.c_str(), 
                        atomSpace.getName(petHandle).c_str(),
                        atomSpace.getName(entityHandle).c_str(),
                        this->cycleCount
                     );

        return opencog::Handle::UNDEFINED; 
    } 

    return evaluationLinkHandle;
}

Handle PsiRelationUpdaterAgent::getEntityHandle(opencog::CogServer * server, const std::string & entityName)
{
    // TODO: What is responsible for creating these handles to entities?

    AtomSpace & atomSpace = * ( server->getAtomSpace() ); 

    std::vector<Handle> entityHandleSet;

    atomSpace.getHandleSet( back_inserter(entityHandleSet),
                            OBJECT_NODE,
                            entityName,
                            true   // Use 'true' here, because OBJECT_NODE is the base class for all the entities
                          );

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

void PsiRelationUpdaterAgent::run(opencog::CogServer * server)
{
    this->cycleCount ++;

    logger().debug( "PsiRelationUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, 
                     this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId and petName
    const std::string & petName = oac->getPet().getName();
    const std::string & petId = oac->getPet().getPetId(); 

    // Get Handle to the pet
    Handle petHandle = AtomSpaceUtil::getAgentHandle( atomSpace, petName); 

    if ( petHandle == Handle::UNDEFINED ) {
        logger().warn("PsiRelationUpdaterAgent::%s - Failed to get the handle to the pet ( name = '%s' ) [ cycle = %d ]",
                        __FUNCTION__, 
                        petName.c_str(), 
                        this->cycleCount
                     );
        return;
    }

    // Check if map info data is available
    if ( atomSpace.getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn("PsiRelationUpdaterAgent::%s - There is no map info available yet [ cycle = %d ]", 
                        __FUNCTION__, 
                        this->cycleCount
                     );
        return;
    }

    // Check if the pet spatial info is already received
    if ( !atomSpace.getSpaceServer().getLatestMap().containsObject(petId) ) {
        logger().warn("PsiRelationUpdaterAgent::%s - Pet was not inserted in the space map yet [ cycle = %d ]", 
                      __FUNCTION__, 
                      this->cycleCount
                     );
        return;
    }

    // Initialize entity, relation lists etc.
    if ( !this->bInitialized )
        this->init(server);

    // Allocate inference steps (totalRemainSteps and singleEntityRelationMaxSteps)
    this->allocInferSteps(); 

    // Process each (relation, entity, pet) triple, which represented in AtomSpace as follows:
    //
    // EvaluationLink (truth value indicates the intensity of the relation between the entity and the pet)
    //     PredicateNode "relationName"
    //         ListLink
    //             entityHandle
    //             petHandle
    //
    while ( this->iCurrentEntity != this->entityList.end() && 
            this->totalRemainSteps > 0 ) {

        // If the entity is the pet itself, skip it
        if ( *iCurrentEntity == petId ) 
            continue; 

        // Get the Handle to the entity
        Handle entityHandle = this->getEntityHandle(server, *iCurrentEntity); 

        if ( entityHandle == opencog::Handle::UNDEFINED )
            continue; 

        // Process all the relations to the entity
        while ( this->iCurrentRelation != this->relationList.end() ) {
            // Get the Handle to the EvaluationLink holding the relation between the entity and the pet
            Handle relationEvaluationLink = this->getRelationEvaluationLink( server,
                                                                             *iCurrentRelation,
                                                                             entityHandle,
                                                                             petHandle
                                                                           ); 

            if ( relationEvaluationLink == opencog::Handle::UNDEFINED )
                continue; 

            // Update the truth value of the relation through PLN (backward chainer)
            int singleEntityRelationRemainSteps = this->singleEntityRelationMaxSteps;  

            this->updateEntityRelation(server, relationEvaluationLink, singleEntityRelationRemainSteps); 

            // Decrease total step remain
            this->totalRemainSteps -= (this->singleEntityRelationMaxSteps - singleEntityRelationRemainSteps);

            this->iCurrentRelation ++; 
        }// while

        this->iCurrentEntity ++;
        this->iCurrentRelation = this->relationList.begin();

    }// while

    // If all the entities have been processed, force the mind agent to reinitialize during next cognitive cycle
    if ( this->iCurrentEntity == this->entityList.end() )
        this->forceInitNextCycle();
}

