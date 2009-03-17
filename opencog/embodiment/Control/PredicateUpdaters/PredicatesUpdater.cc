/** 
 * PredicatesUpdater.cc
 *  
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#include "PAI.h" 
#include "PredicatesUpdater.h"
#include "NearPredicateUpdater.h"
#include "IsSmallPredicateUpdater.h"
#include "IsNoisyPredicateUpdater.h"
#include "IsMovablePredicateUpdater.h"
#include "IsPooPlacePredicateUpdater.h"
#include "IsPeePlacePredicateUpdater.h"
#include "IsPickupablePredicateUpdater.h"
#include "PetPsychePredicatesUpdater.h"


// this time frame corresponde to one minute
static const unsigned long timeWindow = 600;

using namespace OperationalPetController;

PredicatesUpdater::PredicatesUpdater(SpaceServer &_spaceServer, const std::string &_petId) :
									spaceServer(_spaceServer), petId(_petId){
    logger().log(opencog::Logger::DEBUG, "%s - PetId: '%s'.", __FUNCTION__, _petId.c_str());

    // perceptual predicates
    updaters.push_back(new IsSmallPredicateUpdater(_spaceServer.getAtomSpace()));
    updaters.push_back(new IsNoisyPredicateUpdater(_spaceServer.getAtomSpace()));
    updaters.push_back(new IsMovablePredicateUpdater(_spaceServer.getAtomSpace()));
    updaters.push_back(new IsPickupablePredicateUpdater(_spaceServer.getAtomSpace()));
    updaters.push_back(new IsPooPlacePredicateUpdater(_spaceServer.getAtomSpace()));
    updaters.push_back(new IsPeePlacePredicateUpdater(_spaceServer.getAtomSpace()));
    // relation predicates
    updaters.push_back(new NearPredicateUpdater(_spaceServer));

    petPsychePredicatesUpdater = new PetPsychePredicatesUpdater(_spaceServer);
}

PredicatesUpdater::~PredicatesUpdater(){
  foreach(BasicPredicateUpdater* updater, updaters) {
    delete updater;
  }
  updaters.clear();
  delete this->petPsychePredicatesUpdater;
}		

void PredicatesUpdater::update(std::vector<Handle> objects, unsigned long timestamp){
	
  Handle petHandle = spaceServer.getAtomSpace().getHandle(SL_PET_NODE, petId);
  if ( petHandle == Handle::UNDEFINED ) {
    petHandle = spaceServer.getAtomSpace().getHandle(SL_HUMANOID_NODE, petId);
  } // if
  
  for(unsigned int i = 0; i < objects.size(); i++){
    
    // updating all predicates ...
    for(unsigned int j = 0; j < updaters.size(); j++){
      updaters[j]->update(objects[i], petHandle, timestamp );
    }
    
  } // for

  if (objects.size() > 0) {
      petPsychePredicatesUpdater->update( Handle::UNDEFINED, petHandle, timestamp );
  }
}
