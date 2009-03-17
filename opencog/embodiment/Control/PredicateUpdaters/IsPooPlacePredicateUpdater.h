/** 
 * IsPooPlacePredicateUpdater.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef ISPOOPLACEPREDICATEUPDATER_H_
#define ISPOOPLACEPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace OperationalPetController{

class IsPooPlacePredicateUpdater : public OperationalPetController::BasicPredicateUpdater {

	public:
		IsPooPlacePredicateUpdater(AtomSpace &atomSpace);
		~IsPooPlacePredicateUpdater();
	
		void update(Handle object, Handle pet, unsigned long timestamp );
	
}; // class; 
} // namespace

#endif /*ISDRINKABLE_H_*/
