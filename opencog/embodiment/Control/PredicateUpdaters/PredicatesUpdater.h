/** 
 * PredicatesUpdater.h
 * 
 * Update all is_X predicates once a XML message has been processed by the
 * PAI component.
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef PREDICATESUPDATER_H_
#define PREDICATESUPDATER_H_

#include "BasicPredicateUpdater.h"
#include "PetInterface.h"
#include <opencog/atomspace/AtomSpace.h>

#include <vector>  

using namespace opencog;

namespace OperationalPetController{

  class PredicatesUpdater {
	
  private:
		
    /**
     * holds all predicate updaters to be called when the update action
     * takes place
     */		
    std::vector<BasicPredicateUpdater *> updaters;
    BasicPredicateUpdater* petPsychePredicatesUpdater;
		
    SpaceServer &spaceServer;
    std::string petId;

  public:
		
    PredicatesUpdater(SpaceServer &_spaceServer, const std::string &_petId);

    ~PredicatesUpdater();
		
    /**
     * Update the predicates based on the objects that were
     * created or changed via a PVPMessage processed by the 
     * PAI component.
     * 
     * @param objects A std::vector containing the handles of
     * 		  all SL_OBJECT_NODES that were updated 
     * @param timestamp The current timestamp in the virtual world.
     */
    void update(std::vector<Handle> objects, unsigned long timestamp); 
	
  };// class	
} // namespace

#endif /*PREDICATEUPDATER_H_*/
