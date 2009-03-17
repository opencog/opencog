/** 
 * NearPredicateUpdater.h
 * 
 * Author: Ari Heljakka, Welter Luigi 
 * Copyright(c), 2007
 */
#ifndef NEARPREDICATEUPDATER_H_
#define NEARPREDICATEUPDATER_H_

/**
 * This class is used to update the near predicates whenever an object 
 * changes its position in the latest SpaceMap. 
 */
#include "BasicPredicateUpdater.h"
#include "SpaceServer.h"

namespace OperationalPetController{

class NearPredicateUpdater : public OperationalPetController::BasicPredicateUpdater {

    private:
    	SpaceServer& spaceServer;

    public:

        NearPredicateUpdater(SpaceServer& _spaceServer);
        ~NearPredicateUpdater();

        void update(Handle object, Handle pet, unsigned long timestamp );

}; // class
}  // namespace

#endif 

