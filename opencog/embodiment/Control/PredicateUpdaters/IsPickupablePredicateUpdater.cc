/*
 * opencog/embodiment/Control/PredicateUpdaters/IsPickupablePredicateUpdater.cc
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

#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include "IsPickupablePredicateUpdater.h"

using namespace OperationalPetController;
using namespace opencog;

IsPickupablePredicateUpdater::IsPickupablePredicateUpdater(AtomSpace &atomSpace) :
        BasicPredicateUpdater(atomSpace)
{
}

IsPickupablePredicateUpdater::~IsPickupablePredicateUpdater()
{
}

void IsPickupablePredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp )
{

    // an is_pickupable predicate is already assigned for this object, just
    // return. This test is used to keep the predicates consistent
    // over time
    if (isUpdated(object, "is_pickupable")) {
        return;
    }

//    logger().log(opencog::Logger::FINE, "IsPickupable - Updating is_pickupable for ob %s.",
//                    atomsSpace.getName(object).c_str());

    // truth value - mean equals 0.0 --> not smaller than pet
    //               mean equals 1.0 --> is  smaller than pet
    SimpleTruthValue tv(0.0, 1.0);

    // 1. to be pickupable an object must be movable and
    //    small. If these requeriments aren't satisfied then
    //    they are not moveable
    if (AtomSpaceUtil::isPredicateTrue(atomSpace, "is_movable", object) &&
            AtomSpaceUtil::isPredicateTrue(atomSpace, "is_small", object) &&
            atomSpace.getType(object) == ACCESSORY_NODE &&
            !AtomSpaceUtil::isPredicateTrue(atomSpace, "is_drinkable", object)) {
        tv.setMean(1.0);
    }
    AtomSpaceUtil::setPredicateValue(atomSpace, "is_pickupable", tv, object);

    logger().log(opencog::Logger::DEBUG, "IsPickupablePredicateUpdater - Is element %s pickupable: %s",
                 atomSpace.getName(object).c_str(),
                 ( tv.getMean( ) ? "t" : "f" ) );
}
