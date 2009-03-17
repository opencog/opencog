/** 
 * IsMovablePredicateUpdater.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef ISMOVABLEPREDICATEUPDATER_H_
#define ISMOVABLEPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace OperationalPetController{

class IsMovablePredicateUpdater : public OperationalPetController::BasicPredicateUpdater {
	
	public:
		IsMovablePredicateUpdater(AtomSpace &atomSpace);
		~IsMovablePredicateUpdater();
		
		void update(Handle object, Handle pet, unsigned long timestamp );
	
}; // class
}  // namespace

#endif /*ISMOVABLEPREDICATEUPDATER_H_*/
