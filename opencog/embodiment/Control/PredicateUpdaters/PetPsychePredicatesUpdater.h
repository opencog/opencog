/** 
 * PetPsychePredicatesUpdater.h
 * 
 * Author: Samir Ara�jo
 * Copyright(c) Vetta Labs, 2008
 */
#ifndef PETPSYCHEPREDICATESUPDATER_H
#define PETPSYCHEPREDICATESUPDATER_H

#include "BasicPredicateUpdater.h"
#include "SpaceServer.h"
#include "Triangle.h"

namespace OperationalPetController {

  /**
   * This class is used to update all the predicates related with PetPsyche 
   */
  class PetPsychePredicatesUpdater : public BasicPredicateUpdater {  

  private: 
      unsigned long latestSimWorldTimestamp;
	  SpaceServer& spaceServer;
      Spatial::Math::Triangle createFieldOfViewTriangle(Handle agent);
	  
  public:
    
    PetPsychePredicatesUpdater(SpaceServer& _spaceServer);
    
    virtual ~PetPsychePredicatesUpdater( );
    
    void update( Handle object, Handle pet, unsigned long timestamp );
    
  }; // class

} // OperationalPetController

#endif // PETPSYCHEPREDICATESUPDATER_H
