/*
 * opencog/embodiment/WorldWrapper/NoSpaceLifeWorldWrapper.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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


#include "NoSpaceLifeWorldWrapper.h"
#include "WorldWrapperUtil.h"
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#define LOOK_IN_THE_PAST true

namespace opencog { namespace world {

using namespace AvatarCombo;

/**
 * ctor, dtor
 */
NoSpaceLifeWorldWrapper::NoSpaceLifeWorldWrapper(AtomSpace& atomSpace,
        const string& petName,
        const string& ownerName,
        const string& avatarName,
        const CompositeBehaviorDescription& cbd,
        const Temporal& et)
        : _isFailed(false), _isFinished(true),
        _noSpaceLife(atomSpace, petName, ownerName, avatarName, cbd, et),
        _atomSpace(atomSpace), _petName(petName), _ownerName(ownerName),
        _avatarName(avatarName) {}

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
        OC_ASSERT(is_builtin_action(*sib));
        for (sib_it arg = sib.begin(); arg != sib.end(); ++arg) {
            OC_ASSERT(
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
                    OC_ASSERT(arg.is_childless());
                    Handle h = _noSpaceLife.getCurrentMapHandle();
                    OC_ASSERT(h != Handle::UNDEFINED,
                                     "A SpaceMap must exists");
                    //eval indefinite object, put _avatarName as selfName to get
                    //avatar_to_imitate's view point
                    *arg = WorldWrapperUtil::evalIndefiniteObject(h,
                            simulated_time,
                            _atomSpace,
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
    OC_ASSERT(h != Handle::UNDEFINED, "A SpaceMap must exists");
    //eval perception, put _avatarName as selfName to get
    //avatar_to_imitate's view point
    return WorldWrapperUtil::evalPerception(h, simulated_time,
                                            _atomSpace, _avatarName,
                                            _ownerName, it, LOOK_IN_THE_PAST);
}

combo::vertex NoSpaceLifeWorldWrapper::evalIndefiniteObject(combo::indefinite_object io, combo::variable_unifier& vu)
{
    Handle h = _noSpaceLife.getCurrentMapHandle();
    unsigned int simulated_time = _noSpaceLife.getCurrentTime();
    OC_ASSERT(h != Handle::UNDEFINED, "A SpaceMap must exists");
    //eval indefinite object, put _avatarName as selfName to get
    //avatar_to_imitate's view point
    return WorldWrapperUtil::evalIndefiniteObject(h,
            simulated_time,
            _atomSpace,
            _avatarName, _ownerName, io,
            LOOK_IN_THE_PAST);
}

NoSpaceLife& NoSpaceLifeWorldWrapper::getNoSpaceLife()
{
    return _noSpaceLife;
}

} } // namespace opencog::world
