/** 
 * IsPeePlacePredicateUpdater.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef ISPEEPLACEPREDICATEUPDATER_H_
#define ISPEEPLACEPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace OperationalPetController{

class IsPeePlacePredicateUpdater : public OperationalPetController::BasicPredicateUpdater {

	public:
		IsPeePlacePredicateUpdater(AtomSpace &atomSpace);
		~IsPeePlacePredicateUpdater();
	
		void update(Handle object, Handle pet, unsigned long timestamp );
	
}; // class; 
} // namespace

#endif /*ISDRINKABLE_H_*/
