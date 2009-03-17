/** 
 * IsNoisyPredicateUpdater.cc
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include "AtomSpaceUtil.h"
#include "IsNoisyPredicateUpdater.h"

using namespace OperationalPetController;
using namespace opencog;


IsNoisyPredicateUpdater::IsNoisyPredicateUpdater(AtomSpace &atomSpace) :
						 BasicPredicateUpdater(atomSpace){
}

IsNoisyPredicateUpdater::~IsNoisyPredicateUpdater(){
}

void IsNoisyPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp ){
	
	// an is_edible predicate is already assigned for this object, just
	// return. This function is used to keep the predicates consistent
	// over time
	if(isUpdated(object, "is_noisy")){
		return;
	}

	logger().log(opencog::Logger::FINE, "IsNoisy - Updating is_noisy for obj %s.", 
	 			   atomSpace.getName(object).c_str());	

	// truth value - mean equals 0.0 --> not noisy
	//               mean equals 1.0 --> is  noisy 
	SimpleTruthValue tv(0.0, 1.0);
	
	// 1. agents (pet, humanoid or avatar) are noisy since they can produce sounds
	if(atomSpace.getType(object) == SL_AVATAR_NODE ||
	   atomSpace.getType(object) == SL_PET_NODE || 
	   atomSpace.getType(object) == SL_HUMANOID_NODE){	   
		tv.setMean(1.0);
	}
	
	// 2. all other objects are not (for starters)
	AtomSpaceUtil::setPredicateValue(atomSpace, "is_noisy", tv, object);
}
