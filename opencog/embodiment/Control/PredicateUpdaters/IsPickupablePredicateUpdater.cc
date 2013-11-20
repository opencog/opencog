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

#include <opencog/util/oc_assert.h>

#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/types/atom_types.h>

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

#include "IsPickupablePredicateUpdater.h"

using namespace opencog::oac;
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

//    logger().fine("IsPickupable - Updating is_pickupable for ob %s.",
//                    atomsSpace.getName(object).c_str());

    // truth value - mean equals 0.0 --> not smaller than pet
    //               mean equals 1.0 --> is  smaller than pet
    TruthValuePtr tv(SimpleTruthValue::createTV(0.0, 1.0));

    // 1. to be pickupable an object must be movable and
    //    small. If these requeriments aren't satisfied then
    //    they are not moveable
    if (AtomSpaceUtil::isPredicateTrue(atomSpace, "is_movable", object) and
        AtomSpaceUtil::isPredicateTrue(atomSpace, "is_small", object) and
        atomSpace.getType(object) == ACCESSORY_NODE and
        not AtomSpaceUtil::isPredicateTrue(atomSpace, "is_drinkable", object))
    {
        tv = SimpleTruthValue::createTV(1.0, 1.0);

        std::string objectName = atomSpace.getName( object );
        std::string agentName = atomSpace.getName( pet );

        Handle isSmallPredicate = atomSpace.getHandle( 
            PREDICATE_NODE, agentName + "_" + objectName + "_is_small" );

        if ( isSmallPredicate != Handle::UNDEFINED ) {

            Handle comparison = 
                atomSpace.getHandle( DEFINED_FRAME_NODE, "#Evaluative_comparison" );
            OC_ASSERT(comparison != Handle::UNDEFINED,
                      "#Evaluative_comparison wasn't defined yet. Please load the predicates-frames.scm file" );

            HandleSeq isSmallFrame;
            isSmallFrame.push_back( isSmallPredicate );
            isSmallFrame.push_back( comparison );

            Handle isSmallLink = atomSpace.getHandle( INHERITANCE_LINK, isSmallFrame );
            OC_ASSERT(isSmallLink != Handle::UNDEFINED,
                      "is_small frame wasn't defined for the object '%s'", objectName.c_str( ) );

            Handle objectSemeNode =  atomSpace.getHandle( SEME_NODE, objectName );
            OC_ASSERT(objectSemeNode != Handle::UNDEFINED,
                      "A SemeNode wasn't yet defined for the object '%s'",
                      objectName.c_str( ) );
        
            HandleSeq isMovableFrame;
            isMovableFrame.push_back( objectSemeNode );
            isMovableFrame.push_back( Handle::UNDEFINED );

            HandleSeq parentHandles;
            Type types[] = { SEME_NODE, CONCEPT_NODE };
            
            atomSpace.getHandlesByOutgoing( back_inserter(parentHandles),
                                    isMovableFrame, &types[0], NULL, 2, INHERITANCE_LINK, false );

            OC_ASSERT(parentHandles.size( ) == 1, 
                      "Invalid number of InheritanceLinks which defines the object type. %d but should be 1", 
                      parentHandles.size( ) );
        
        
            HandleSeq isPickupable;
            isPickupable.push_back( parentHandles[0] ); // is_movable
            isPickupable.push_back( isSmallLink ); // is_small
        
            atomSpace.addLink( EVALUATION_LINK, isPickupable );
        } // if

    }
    AtomSpaceUtil::setPredicateValue(atomSpace, "is_pickupable", tv, object);
    
    logger().debug("IsPickupablePredicateUpdater - Is element %s pickupable: %s",
                 atomSpace.getName(object).c_str(),
                 ( tv->getMean( ) ? "t" : "f" ) );
}
