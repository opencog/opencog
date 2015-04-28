/*
 * opencog/embodiment/Control/PredicateUpdaters/IsPooPlacePredicateUpdater.cc
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
#include <opencog/spacetime/atom_types.h>


#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include "IsPooPlacePredicateUpdater.h"

using namespace opencog::oac;
using namespace opencog;

IsPooPlacePredicateUpdater::IsPooPlacePredicateUpdater(AtomSpace &atomSpace) :
        BasicPredicateUpdater(atomSpace)
{
}

IsPooPlacePredicateUpdater::~IsPooPlacePredicateUpdater()
{
}

void IsPooPlacePredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp )
{
    // an is_poo_place predicate is already assigned for this object, just
    // return. This function is used to keep the predicates consistent
    // over time
    if (isUpdated(object, "is_poo_place")) {
        return;
    }

    logger().fine("IsPooPlace - Updating is_poo_place for ob %s.",
                 atomSpace.getName(object).c_str());

    // truth value - mean equals 0.0 --> not poo place
    //     mean equals 1.0 --> is poo place
    TruthValuePtr tv = SimpleTruthValue::createTV(0.0, 1.0);

    // only structures are considered poo place
    if (atomSpace.getType(object) == STRUCTURE_NODE) {
        std::string objectName = atomSpace.getName(object);
        Handle objectSemeNode = atomSpace.addNode( SEME_NODE, objectName );        
        Handle staticObjectConceptNode = atomSpace.addNode( CONCEPT_NODE, "Structure" );
        
        HandleSeq inheritsFrom;
        inheritsFrom.push_back( objectSemeNode );        
        inheritsFrom.push_back( staticObjectConceptNode );
        Handle link = atomSpace.addLink( INHERITANCE_LINK, inheritsFrom );
        
        tv = SimpleTruthValue::createTV(1.0, 1.0);
        atomSpace.setLTI( link, 1);
        atomSpace.setTV( link, tv );
    }
    // all other objects are not poo places

    AtomSpaceUtil::setPredicateValue(atomSpace, "is_poo_place", tv, object);
}
