/**
 * NoSpaceLifeWorldWrapper.cc
 *
 * Author(s):
 *    Nil Geisweiller
 * Created : Wed Dec 5 2007
 */

#include "NoSpaceLifeWorldWrapper.h"
#include "WorldWrapperUtil.h"
#include "PetComboVocabulary.h"

#define LOOK_IN_THE_PAST true

namespace WorldWrapper
{

using namespace PetCombo;

/**
 * ctor, dtor
 */
NoSpaceLifeWorldWrapper::NoSpaceLifeWorldWrapper(SpaceServer& spaceServer,
                                                 const string& petName,
                                                 const string& ownerName,
                                                 const string& avatarName,
                                                 const CompositeBehaviorDescription& cbd,
                                                 const Temporal& et,
                                                 opencog::RandGen& _rng)
    : _isFailed(false), _isFinished(true),
      _noSpaceLife(spaceServer, petName, ownerName, avatarName, cbd, et, _rng),
      _spaceServer(spaceServer), _petName(petName), _ownerName(ownerName),
      _avatarName(avatarName), rng(_rng) {}

NoSpaceLifeWorldWrapper::~NoSpaceLifeWorldWrapper() {}

/**
 * public methods
 */

bool NoSpaceLifeWorldWrapper::isPlanFinished() const
{
    return _isFinished;
}

bool NoSpaceLifeWorldWrapper::isPlanFailed() const
{
    return _isFailed;
}

bool NoSpaceLifeWorldWrapper::sendSequential_and(sib_it from, sib_it to)
{
    //evaluate only non random indefinite objects
    //the random definite objects will be evaluated
    //directly inside NoSpaceLife because it contains the
    //Behavior Description that is to be tried to be fitted
    unsigned int simulated_time = _noSpaceLife.getCurrentTime();
    for (sib_it sib = from; sib != to; ++sib) {
        opencog::cassert(TRACE_INFO, is_builtin_action(*sib));
        for (sib_it arg = sib.begin(); arg != sib.end(); ++arg) {
            opencog::cassert(TRACE_INFO,
                              is_definite_object(*arg)
                              || is_indefinite_object(*arg)
                              || is_contin(*arg),
                              "In the current implementation builtin_action"
                              " arguments must be definite_object,"
                              " indefinite_object or contin");
            if (is_indefinite_object(*arg)) {
                indefinite_object io = get_indefinite_object(*arg);
                //check that it's not a random indefinite object and evaluate it
                if (!is_random(io)) {
                    opencog::cassert(TRACE_INFO, arg.is_childless());
                    Handle h = _noSpaceLife.getCurrentMapHandle();
                    opencog::cassert(TRACE_INFO, h != Handle::UNDEFINED,
                                      "A SpaceMap must exists");
                    //eval indefinite object, put _avatarName as selfName to get
                    //avatar_to_imitate's view point
                    *arg = WorldWrapperUtil::evalIndefiniteObject(rng, h,
                                                                  simulated_time,
                                                                  _spaceServer,
                                                                  _avatarName,
                                                                  _ownerName,
                                                                  io);
                }
            }
        }
    }
    return _noSpaceLife.processSequential_and(from, to);
}

combo::vertex NoSpaceLifeWorldWrapper::evalPerception(pre_it it, combo::variable_unifier& vu)
{
    Handle h = _noSpaceLife.getCurrentMapHandle();
    unsigned int simulated_time = _noSpaceLife.getCurrentTime();
    opencog::cassert(TRACE_INFO, h != Handle::UNDEFINED, "A SpaceMap must exists");
    //eval perception, put _avatarName as selfName to get
    //avatar_to_imitate's view point
    return WorldWrapperUtil::evalPerception(rng, h, simulated_time,
                                            _spaceServer, _avatarName,
                                            _ownerName, it, LOOK_IN_THE_PAST);
}

combo::vertex NoSpaceLifeWorldWrapper::evalIndefiniteObject(combo::indefinite_object io, combo::variable_unifier& vu)
{
    Handle h = _noSpaceLife.getCurrentMapHandle();
    unsigned int simulated_time = _noSpaceLife.getCurrentTime();
    opencog::cassert(TRACE_INFO, h != Handle::UNDEFINED, "A SpaceMap must exists");
    //eval indefinite object, put _avatarName as selfName to get
    //avatar_to_imitate's view point
    return WorldWrapperUtil::evalIndefiniteObject(rng, h,
            simulated_time,
            _spaceServer,
            _avatarName, _ownerName, io,
            LOOK_IN_THE_PAST);
}

NoSpaceLife& NoSpaceLifeWorldWrapper::getNoSpaceLife()
{
    return _noSpaceLife;
}

}
