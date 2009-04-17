/*
 * opencog/embodiment/Control/PredicateUpdaters/IsMovablePredicateUpdater.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include "IsMovablePredicateUpdater.h"

using namespace opencog;
using namespace OperationalPetController;

IsMovablePredicateUpdater::IsMovablePredicateUpdater(AtomSpace &atomSpace) :
        BasicPredicateUpdater(atomSpace)
{
}

IsMovablePredicateUpdater::~IsMovablePredicateUpdater()
{
}

void IsMovablePredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp )
{

    // an is_movable predicate is already assigned for this object, just
    // return. This function is used to keep the predicates consistent
    // over time
    if (isUpdated(object, "is_movable")) {
        return;
    }

    logger().log(opencog::Logger::FINE, "IsMovable - Updating is_movable for obj %s.",
                 atomSpace.getName(object).c_str());


    // 1. pets and structures are not movable
    if (atomSpace.getType(object) == PET_NODE ||
            atomSpace.getType(object) == STRUCTURE_NODE) {
        return;
    }


    // truth value - mean equals 0.0 --> not smaller than pet
    //     mean equals 1.0 --> is  smaller than pet
    SimpleTruthValue tv(0.0, 1.0);

    // 2. all avatars are movable - TV set to 1.0
    if (atomSpace.getType(object) == AVATAR_NODE) {
        tv.setMean(1.0);
        AtomSpaceUtil::addPropertyPredicate(atomSpace, "is_movable", object, tv, true);
        return;
    }

    // 3. other objects, if they are small, they are
    //    movable
    if (AtomSpaceUtil::isPredicateTrue(atomSpace, "is_small", object)) {
        tv.setMean(1.0);
    } else {
        tv.setMean(0.0);
    }
    AtomSpaceUtil::addPropertyPredicate(atomSpace, "is_movable", object, tv, true);
}
