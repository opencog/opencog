/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiRelationUpdaterByPLNAgent.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-02-23
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
#include "PsiRelationUpdaterByPLNAgent.h"
#include "PsiRuleUtil.h"

#include<boost/tokenizer.hpp>

using namespace opencog::oac;

extern int currentDebugLevel;

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
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace
    AtomSpace& atomSpace = oac->getAtomSpace();

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

    // Set the truth value of relationEvaluationLink to stv(0 0), which would force PLN refresh anyway
    // TODO: Really? Shall we have to do this?
    SimpleTruthValue stvFalse(0, 0);

    atomSpace.setTV(relationEvaluationLink, stvFalse);

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
                                                          Handle petHandle, 
                                                          Handle entityHandle
                                                         )
{
    // Get the AtomSpace
    AtomSpace & atomSpace = * ( server->getAtomSpace() ); 

    // Get the Handle to relation (PredicateNode)
    Handle relationPredicateHandle = atomSpace.getHandle(PREDICATE_NODE, relationName);

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

    listLinkOutgoing.push_back(petHandle);
    listLinkOutgoing.push_back(entityHandle);

    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, listLinkOutgoing);

    if (listLinkHandle == Handle::UNDEFINED) {
        logger().debug( "PsiRelationUpdaterAgent::%s - Failed to find the ListLink containing both entity ( id = '%s' ) and pet ( id = '%s' ). Don't worry, it will be created automatically [ cycle = %d ].", 
                        __FUNCTION__, 
                        atomSpace.getName(entityHandle).c_str(),
                        atomSpace.getName(petHandle).c_str(),
                        this->cycleCount
                      );

        AtomSpaceUtil::addLink(atomSpace, LIST_LINK, listLinkOutgoing, false);
    } 

    // Get the Handle to EvaluationLink holding the relation between the pet and the entity
    std::vector<Handle> evaluationLinkOutgoing; 

    evaluationLinkOutgoing.push_back(relationPredicateHandle);
    evaluationLinkOutgoing.push_back(listLinkHandle);

    Handle evaluationLinkHandle = atomSpace.getHandle(EVALUATION_LINK, evaluationLinkOutgoing);

    if (evaluationLinkHandle == Handle::UNDEFINED) {
        logger().debug( "PsiRelationUpdaterAgent::%s - Failed to find the EvaluationLink holding the '%s' relation between the pet ( id = '%s' ) and the entity ( id = '%s' ). Don't worry it would be created automatically [ cycle = %d ].", 
                        __FUNCTION__, 
                        relationName.c_str(), 
                        atomSpace.getName(petHandle).c_str(),
                        atomSpace.getName(entityHandle).c_str(),
                        this->cycleCount
                     );

        AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK, evaluationLinkOutgoing, false);
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

void PsiRelationUpdaterAgent::run()
{
    this->cycleCount ++;

    logger().debug( "PsiRelationUpdaterAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace
    AtomSpace& atomSpace = oac->getAtomSpace();

    // Get petId and petName
    const std::string & petName = oac->getPet().getName();
    const std::string & petId = oac->getPet().getPetId(); 

    // Get Handle to the pet
    Handle petHandle = AtomSpaceUtil::getAgentHandle( atomSpace, petId); 

    if ( petHandle == Handle::UNDEFINED ) {
        logger().warn("PsiRelationUpdaterAgent::%s - Failed to get the handle to the pet ( id = '%s' ) [ cycle = %d ]",
                        __FUNCTION__, 
                        petId.c_str(), 
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

    // Decide whether to update relations during this cognitive cycle (controlled by the modulator 'SecuringThreshold')
    float securingThreshold = AtomSpaceUtil::getCurrentModulatorLevel(atomSpace,
                                                                      SECURING_THRESHOLD_MODULATOR_NAME, 
                                                                      petId
                                                                     );
// TODO: Uncomment the line below once finish testing
//    if ( randGen.randfloat() < securingThreshold ) 
    {
        logger().debug(
                "PsiRelationUpdaterAgent::%s - Skip updating the relations for this cognitive cycle [ cycle = %d ] ", 
                       __FUNCTION__, 
                       this->cycleCount
                      );
        return; 
    }

    // Initialize entity, relation lists etc.
    if ( !this->bInitialized )
        this->init();

    // Allocate inference steps (totalRemainSteps and singleEntityRelationMaxSteps)
    this->allocInferSteps(); 

    // Process (relation, pet, entity) triples, which are represented in AtomSpace as follows:
    //
    // EvaluationLink (truth value indicates the intensity of the relation between the entity and the pet)
    //     PredicateNode "relationName"
    //         ListLink
    //             petHandle
    //             entityHandle
    //
    
    std::vector<std::string>::iterator iEntity;
    std::vector<std::string>::iterator iRelation; 

    while ( this->totalRemainSteps > 0 ) {

        totalRemainSteps --; 	    
       
        // Randomly select an entity
        iEntity = this->entityList.begin() + randGen().randint( this->entityList.size() ); 

        // If the entity is the pet itself, skip it
        if ( *iEntity == petId ) 
            continue; 

        // Get the Handle to the entity
        Handle entityHandle = this->getEntityHandle(server, *iEntity); 

        if ( entityHandle == opencog::Handle::UNDEFINED )
            continue; 

        // Randomly select a relation
        iRelation = this->relationList.begin() + randGen().randint( this->relationList.size() );	

        // Get the Handle to the EvaluationLink holding the relation between the entity and the pet
        // If it doesn't exist, the function below would create one and return it. 
        Handle relationEvaluationLink = this->getRelationEvaluationLink( server,
                                                                         *iRelation,
								                                         petHandle, 
                                                                         entityHandle
                                                                       ); 

        // Update the truth value of the relation through PLN (backward chainer)
        int singleEntityRelationRemainSteps = this->singleEntityRelationMaxSteps;  

        this->updateEntityRelation(server, relationEvaluationLink, singleEntityRelationRemainSteps);        

        // Decrease total step remain
        this->totalRemainSteps -= (this->singleEntityRelationMaxSteps - singleEntityRelationRemainSteps);

    }// while

    // Since the perception would changes next cycle, the function below would inforce the mind agent 
    // reinitialize itself during next cycle
    this->forceInitNextCycle();
}

