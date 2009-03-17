/** 
 * PredicateUpdater.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef BASICPREDICATEUPDATER_H_
#define BASICPREDICATEUPDATER_H_

#include <LADSUtil/Logger.h>
#include <opencog/atomspace/AtomSpace.h>
#include "PredefinedProcedureNames.h"

using namespace opencog;

namespace OperationalPetController{

  class BasicPredicateUpdater{
	
  protected: 
    AtomSpace& atomSpace;
		
    /**
     * Return the handle of all of the EvalLink for the predicate in 
     * question with the given object. If there is no such predicate 
     * then Handle::UNDEFINED is returned.
     * 
     * @param object The handle of the object 
     * @param predicateName A string representation of the predicate 
     */
    Handle getPredHandle(Handle object, std::string predicateName);	

    /**
     * Get the handle of the SL_OBJECT_NODE (or subtype) for the given name.
     * 
     * @return Handle::UNDEFINED if no handle is found, the exatcly handle if
     * just one handle is found or the first handle if more than on handle
     * is found
     */
    Handle getHandle(std::string objName);

  public:
		
    /*
     * Constructor and destructor;
     */
    BasicPredicateUpdater(AtomSpace& _atomSpace) : atomSpace(_atomSpace){}
    virtual  ~BasicPredicateUpdater(){}
		
    /**
     * Update is_X predicate based on the object in question. The pet
     * handle may be used to update predicates relative to the 
     * the pet.
     *
     * @param object The handle of the object 
     * @param pet The handle of the pet
     */
    virtual void update(Handle object, Handle pet, unsigned long timestamp );		
		
    /**
     * Return true if there is already a is_X predicate created for the given
     * object handle.
     * 
     * @param object The handle of the object 
     * @param predicateName A string representation of the predicate
     */
    bool isUpdated(Handle object, std::string predicateName);
		
	
  }; // class
} // namespace

#endif 
