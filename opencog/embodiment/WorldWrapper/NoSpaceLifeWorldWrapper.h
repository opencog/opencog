/**
 * NoSpaceLifeWorldWrapper.h
 *
 * Author(s):
 *    Nil Geisweiller
 * Created : Wed Dec 5 2007
 */
#ifndef _NOSPACELIFEWORLDWRAPPER_H
#define _NOSPACELIFEWORLDWRAPPER_H

#include "comboreduct/combo/vertex.h"

#include "WorldWrapper.h"
#include "NoSpaceLife.h"

namespace WorldWrapper
{

using namespace ImaginaryLife;
using namespace behavior;
using namespace combo;

/**
 * NoSpaceLife world wrapper
 */

class NoSpaceLifeWorldWrapper : public WorldWrapperBase
{

public:

    /**
     * ctor, dtor
     * cbd is the composite behavior description to imitate
     */
    NoSpaceLifeWorldWrapper(SpaceServer& spaceServer,
                            const string& petName,
                            const string& ownerName,
                            const string& avatarName,
                            const CompositeBehaviorDescription& cbd,
                            const Temporal& exemplarTemporal,
                            opencog::RandGen& _rng);
    ~NoSpaceLifeWorldWrapper();

    /**
     * return true is the current action plan is finished
     * false otherwise or if there is no action plan
     */
    bool isPlanFinished() const;

    /**
     * return true if the plan has failed
     * false otherwise
     * pre-condition : the plan is finished
     */
    bool isPlanFailed() const;

    /**
     * Send a sequence of sequential_and actions [from, to)
     * return true iff the plan is actually executed
     */
    bool sendSequential_and(sib_it from, sib_it to);

    /**
     * evaluate a perception
     */
    combo::vertex evalPerception(pre_it per,
                                 combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    /**
     * evaluate an indefinite object
     */
    combo::vertex evalIndefiniteObject(combo::indefinite_object io,
                                       combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    /**
     * getNoSpaceLife
     */
    NoSpaceLife& getNoSpaceLife();

private:
    bool _isFailed;
    bool _isFinished;

    NoSpaceLife _noSpaceLife;
    SpaceServer& _spaceServer;

    const string& _petName;
    const string& _ownerName;
    const string& _avatarName; //avatar to imitate, to take its view point

    opencog::RandGen& rng;
};

}

#endif
