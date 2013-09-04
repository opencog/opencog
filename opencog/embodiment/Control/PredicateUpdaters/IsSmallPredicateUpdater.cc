/*
 * opencog/embodiment/Control/PredicateUpdaters/IsSmallPredicateUpdater.cc
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

#include <boost/lexical_cast.hpp>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/types/atom_types.h>

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include "IsSmallPredicateUpdater.h"

using namespace opencog::oac;
using namespace opencog;

IsSmallPredicateUpdater::IsSmallPredicateUpdater(AtomSpace & atomSpace) :
        BasicPredicateUpdater(atomSpace)
{
}

IsSmallPredicateUpdater::~IsSmallPredicateUpdater()
{
}

/**
 * Size predicate template
 *
 * EvalLink
 *   PredicateNode:"size"
 *   ListLink
 *      ObjectNode:"<obj_id">
 *      NumberNode:"<length>"
 *      NumberNode:"<width>"
 *      NumberNode:"<height>"
 */
double IsSmallPredicateUpdater::getSize(Handle object)
{
    double length = 0.0;
    double width  = 0.0;
    double height  = 0.0;

    bool result = AtomSpaceUtil::getSizeInfo(atomSpace, object, length, width, height);
    if (!result) {
        return 0.0;
    }

    return (length * width * height);
}


void IsSmallPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp )
{
    // an is_small predicate is already assigned for this object, just
    // return. This function is used to keep the predicates consistent
    // over time
    if (isUpdated(object, "is_small")) {
        return;
    }

    logger().fine("IsSmall - Updating is_small for obj %s.",
                 atomSpace.getName(object).c_str());

    // truth value - mean equals 0.0 --> not smaller than pet
    //                 mean equals 1.0 --> is  smaller than pet
    SimpleTruthValue tv(0.0, 1.0);

    // while there are no size information some assumptions
    // will guide the is_small predicate
    

    /*

    // 1. all avatars are considered bigger than any pet
    // -> it cannot true if the agent is an humanoid
    if (atomSpace.getType(object) == AVATAR_NODE) {
        tv.setMean(0.0);
        AtomSpaceUtil::setPredicateValue(atomSpace, "is_small", tv, object);
        return;
    }
    
    // 2. all accessories are considered smaller than the pet
    // -> maybe, but an humanoid accessory can be greater the the pet, 
    // so lets ignore this rule for now
    if (atomSpace.getType(object) == ACCESSORY_NODE) {
        tv.setMean(1.0);
        AtomSpaceUtil::setPredicateValue(atomSpace, "is_small", tv, object);
        return;
    }
    */

    // 3. compare the size of the pet and the object and compare
    // an object must has its volume lesser than 50% of the agent volume
    // to be considered small

    double agentVolume = getSize(pet);
    double objectVolume = getSize(object);
    if ( objectVolume/agentVolume < 0.9 ) {
        tv.setMean(1.0);

        std::string objectName = atomSpace.getName( object );
        std::string agentName = atomSpace.getName( pet );

        std::map<std::string, Handle> agentSizeElements;
        agentSizeElements["Dimension"] = atomSpace.addNode( CONCEPT_NODE, "Volume" );
        agentSizeElements["Object"] = atomSpace.addNode( SEME_NODE, agentName );
        agentSizeElements["Measurement"] = atomSpace.addNode( NUMBER_NODE, 
           boost::lexical_cast<std::string>( agentVolume ) );

        AtomSpaceUtil::setPredicateFrameFromHandles(atomSpace, "#Dimension",
            agentName + "_dimension", agentSizeElements, tv );


        std::map<std::string, Handle> objectSizeElements;
        objectSizeElements["Dimension"] = atomSpace.addNode( CONCEPT_NODE, "Volume" );
        objectSizeElements["Object"] = atomSpace.addNode( SEME_NODE, objectName );
        objectSizeElements["Measurement"] = atomSpace.addNode( NUMBER_NODE, 
           boost::lexical_cast<std::string>( objectVolume ) );

        AtomSpaceUtil::setPredicateFrameFromHandles(atomSpace, "#Dimension",
            objectName + "_dimension", objectSizeElements, tv );


        std::map<std::string, Handle> comparisonElements;
        comparisonElements["Attribute"] = objectSizeElements["Dimension"];
        comparisonElements["Profiled_attribute"] = agentSizeElements["Dimension"];
        comparisonElements["Standard_attribute"] = objectSizeElements["Dimension"];
        comparisonElements["Profiled_item"] = agentSizeElements["Object"];
        comparisonElements["Standard_item"] = objectSizeElements["Object"];
        comparisonElements["Value"] = atomSpace.addNode( NUMBER_NODE, 
           boost::lexical_cast<std::string>( objectVolume / agentVolume ) );

        AtomSpaceUtil::setPredicateFrameFromHandles(atomSpace, "#Evaluative_comparison",
            agentName + "_" + objectName + "_is_small", comparisonElements, tv );

    } else {
        tv.setMean(0.0);
    } // if
    AtomSpaceUtil::setPredicateValue(atomSpace, "is_small", tv, object);
}
