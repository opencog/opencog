/** 
 * IsPeePlacePredicateUpdater.cc
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include "AtomSpaceUtil.h"
#include "IsPeePlacePredicateUpdater.h"

using namespace OperationalPetController;
using namespace opencog;

IsPeePlacePredicateUpdater::IsPeePlacePredicateUpdater(AtomSpace &atomSpace) :
							BasicPredicateUpdater(atomSpace){
}

IsPeePlacePredicateUpdater::~IsPeePlacePredicateUpdater(){
}

void IsPeePlacePredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp ){

 	// an is_pee_place predicate is already assigned for this object, just
	// return. This function is used to keep the predicates consistent
	// over time
	if(isUpdated(object, "is_pee_place")){
		return;
	}

    MAIN_LOGGER.log(LADSUtil::Logger::FINE, "IsPeePlace - Updating is_pee_place for obj %s.",
                   atomSpace.getName(object).c_str());

	// truth value - mean equals 0.0 --> not pee place
	//				 mean equals 1.0 --> is pee place 
	SimpleTruthValue tv(0.0, 1.0);

	// only structures are considered pee place
	if(atomSpace.getType(object) == SL_STRUCTURE_NODE){
 		tv.setMean(1.0);
	}
	// all other objects are not pee places

	AtomSpaceUtil::setPredicateValue(atomSpace, "is_pee_place", tv, object);
} 
