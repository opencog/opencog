/*
 * opencog/embodiment/Control/PredicateUpdaters/PetPsychePredicatesUpdater.h
 *
 * Copyright (C) 2007-2008 Samir Araujo
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef PETPSYCHEPREDICATESUPDATER_H
#define PETPSYCHEPREDICATESUPDATER_H

#include "BasicPredicateUpdater.h"
#include <opencog/atomspace/SpaceServer.h>
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
