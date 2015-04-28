/*
 * opencog/embodiment/Control/PredicateUpdaters/IsNoisyPredicateUpdater.cc
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
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/types/atom_types.h>

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include "IsNoisyPredicateUpdater.h"

using namespace opencog::oac;
using namespace opencog;


IsNoisyPredicateUpdater::IsNoisyPredicateUpdater(AtomSpace &atomSpace) :
        BasicPredicateUpdater(atomSpace)
{
}

IsNoisyPredicateUpdater::~IsNoisyPredicateUpdater()
{
}

void IsNoisyPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp )
{

    // an is_edible predicate is already assigned for this object, just
    // return. This function is used to keep the predicates consistent
    // over time
    if (isUpdated(object, "is_noisy")) {
        return;
    }

    logger().fine("IsNoisy - Updating is_noisy for obj %s.",
                 atomSpace.getName(object).c_str());

    // truth value - mean equals 0.0 --> not noisy
    //               mean equals 1.0 --> is  noisy
    TruthValuePtr tv = SimpleTruthValue::createTV(0.0, 1.0);

    // 1. agents (pet, humanoid or avatar) are noisy since they can produce sounds
    if ( atomSpace.getType(object) == AVATAR_NODE ||
         atomSpace.getType(object) == PET_NODE ||
         atomSpace.getType(object) == HUMANOID_NODE) {
        
        tv = SimpleTruthValue::createTV(1.0, 1.0);
        
        std::string objectName = atomSpace.getName( object );

        // 1.1 create a frame for this predicate
        std::map<std::string, Handle> elements;
        elements["Noisy_event"] = atomSpace.addNode( SEME_NODE, "walking" );
        elements["Sound"] = atomSpace.addNode( SEME_NODE, "scratch" );
        elements["Sound_source"] = atomSpace.addNode( SEME_NODE, objectName );
        elements["Degree"] = atomSpace.addNode( SEME_NODE, "medium" );
        
        AtomSpaceUtil::setPredicateFrameFromHandles(atomSpace, "#Make_noise",
            objectName + "_is_noisy", elements, tv );
    } // if

    // 2. all other objects are not (for starters)
    AtomSpaceUtil::setPredicateValue(atomSpace, "is_noisy", tv, object);
}
