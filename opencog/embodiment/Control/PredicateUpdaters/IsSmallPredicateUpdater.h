/** 
 * IsSmallPredicateUpdater.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef ISSMALLPREDICATEUPDATER_H_
#define ISSMALLPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace OperationalPetController {

class IsSmallPredicateUpdater : public OperationalPetController::BasicPredicateUpdater {
	
	protected:
		
		/**
		 * Return the size of the object. For 2D maps the size is considered as
	 	 * the width * length product. For 3D maps the size is considered as 
	 	 * the width * length * height product.
	 	 * 
	 	 * NOTE: for now only 2D maps are considered
	 	 * 
	 	 * @param object. The handle to the object to be evaluated.
		 */
		double getSize(Handle object);
	
	public:
		
		IsSmallPredicateUpdater(AtomSpace & atomSpace);
		~IsSmallPredicateUpdater();

		/**
		 * Update is_small predicate for the given object. Since the
		 * predicate relates to the object size relative to the pet
		 * size, that is, an object small for a pet could be not
		 * small for other ones
		 * 
		 * @param pet The handle of the pet (used to get its size)
		 * @param object The handle of the object, used to get it's category 
		 *        and size.
		 */  
		void update(Handle object, Handle pet, unsigned long timestamp );

}; // class
} // namespace

#endif /*ISSMALLPREDICATEUPDATER_H_*/
