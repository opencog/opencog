/** 
 * IsPickupablePredicateUpdater.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef ISPICKUPABLEPREDICATEUPDATER_H_
#define ISPICKUPABLEPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace OperationalPetController{
	
class IsPickupablePredicateUpdater : public OperationalPetController::BasicPredicateUpdater {
	
	private:
	
		/**
		 * Return true if the object is movable and false otherwise. An object
		 * to be pickable must be movable, but the oposite isn't true.
		 * 
		 * @param object The handle of the object
		 */
		bool isMovable(Handle Object);
	
	public:
		IsPickupablePredicateUpdater(AtomSpace &atomSpace);
		~IsPickupablePredicateUpdater();
		
		void update(Handle object, Handle pet, unsigned long timestamp );
	
}; // class 	
}  // namespace

#endif /*ISPICKUPABLEPREDICATEUPDATER_H_*/
