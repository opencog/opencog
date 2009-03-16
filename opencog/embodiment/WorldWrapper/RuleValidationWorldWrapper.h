/**
 * RuleValidationWorldWrapper.h
 *
 * Author(s):
 *    Nil Geisweiller
 * Created : Mon Nov 3 2007
 */
#ifndef _VIRUTAL_WORLD_WRAPPER_H
#define _VIRTUAL_WORLD_WRAPPER_H

#include "WorldWrapper.h"
#include "VirtualWorldState.h"
#include "util/exceptions.h"

namespace WorldWrapper {

class RuleValidationWorldWrapper : public WorldWrapperBase {
    
    public:

        RuleValidationWorldWrapper(VirtualWorldData::VirtualWorldState & virtualWorld);//, opencog::RandGen& _rng);
        ~RuleValidationWorldWrapper();

        // override
        bool isPlanFinished() const;

        // override
        bool isPlanFailed() const;

        // override
        bool sendSequential_and(sib_it from, sib_it to); 

       
        // override
        combo::vertex evalPerception(pre_it per,
                combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

        // override
        combo::vertex evalIndefiniteObject(combo::indefinite_object io,
                combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    private:

        VirtualWorldData::VirtualWorldState& virtualWorld;
        //opencog::RandGen& rng;
};
}

#endif
