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

    logger().fine("IsMovable - Updating is_movable for obj %s.",
                 atomSpace.getName(object).c_str());
    

    // 2. all avatars are movable - TV set to 1.0
    Type objectType = atomSpace.getType(object);
    std::string objectName = atomSpace.getName(object);
    Handle objectSemeNode = atomSpace.addNode( SEME_NODE, objectName );
    
    HandleSeq inheritsFrom;
    inheritsFrom.push_back( objectSemeNode );

    bool isMovable = false;
    if ( objectType == PET_NODE ) {
        isMovable = true;
        Handle petConceptNode = atomSpace.getHandle( CONCEPT_NODE, "Pet" );
        inheritsFrom.push_back( petConceptNode );

    } else if ( objectType == AVATAR_NODE ) {
        isMovable = true;
        Handle avatarConceptNode = atomSpace.getHandle( CONCEPT_NODE, "Avatar" );
        inheritsFrom.push_back( avatarConceptNode );

    } else if ( objectType == HUMANOID_NODE ) {
        isMovable = true;
        Handle humanoidConceptNode = atomSpace.getHandle( CONCEPT_NODE, "Humanoid" );
        inheritsFrom.push_back( humanoidConceptNode );

    } else if ( objectType == ACCESSORY_NODE ) {
        isMovable = true;
        Handle movableObjectConceptNode = atomSpace.getHandle( CONCEPT_NODE, "Item" );
        inheritsFrom.push_back( movableObjectConceptNode );

    } // else if

    SimpleTruthValue tv(0.0, 1.0);
    if ( isMovable ) {
        tv.setMean( 1.0 );

        if ( inheritsFrom[1] != Handle::UNDEFINED ) {
            Handle link = atomSpace.addLink( INHERITANCE_LINK, inheritsFrom );
            atomSpace.setLTI( link, 1 );
            atomSpace.setTV( link, tv );
        } // if
    } // if
    AtomSpaceUtil::setPredicateValue(atomSpace, "is_movable", tv, object);

}
