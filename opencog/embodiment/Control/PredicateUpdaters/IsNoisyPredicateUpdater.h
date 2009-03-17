/** 
 * IsNoisyPredicateUpdater.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef ISNOISYPREDICATEUPDATER_H_
#define ISNOISYPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace OperationalPetController {

class IsNoisyPredicateUpdater : public OperationalPetController::BasicPredicateUpdater {

	public:
		IsNoisyPredicateUpdater(AtomSpace &atomSpace);
		~IsNoisyPredicateUpdater();
		
		void update(Handle object, Handle pet, unsigned long timestamp );
		
	
}; // class
} // namespace

#endif /*ISNOISYPREDICATEUPDATER_H_*/
