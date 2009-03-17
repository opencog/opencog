/** 
 * IsPooPlacePredicateUpdater.cc
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include "AtomSpaceUtil.h"
#include "IsPooPlacePredicateUpdater.h"

using namespace OperationalPetController;
using namespace opencog;

IsPooPlacePredicateUpdater::IsPooPlacePredicateUpdater(AtomSpace &atomSpace) :
							BasicPredicateUpdater(atomSpace){
}

IsPooPlacePredicateUpdater::~IsPooPlacePredicateUpdater(){
}

void IsPooPlacePredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp ){

 	// an is_poo_place predicate is already assigned for this object, just
	// return. This function is used to keep the predicates consistent
	// over time
	if(isUpdated(object, "is_poo_place")){
		return;
	}

    logger().log(opencog::Logger::FINE, "IsPooPlace - Updating is_poo_place for ob %s.",
                   atomSpace.getName(object).c_str());

	// truth value - mean equals 0.0 --> not poo place
	//				 mean equals 1.0 --> is poo place 
	SimpleTruthValue tv(0.0, 1.0);

	// only structures are considered poo place
	if(atomSpace.getType(object) == SL_STRUCTURE_NODE){
 		tv.setMean(1.0);
	}
	// all other objects are not poo places

        AtomSpaceUtil::setPredicateValue(atomSpace, "is_poo_place", tv, object);
} 
