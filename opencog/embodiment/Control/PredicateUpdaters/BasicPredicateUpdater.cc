/*
 * opencog/embodiment/Control/PredicateUpdaters/BasicPredicateUpdater.cc
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


#include <opencog/atomspace/Link.h>
#include <opencog/spacetime/atom_types.h>

#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>

#include "BasicPredicateUpdater.h"

using namespace opencog::oac;
using namespace opencog;

Handle BasicPredicateUpdater::getPredHandle(Handle object, std::string predicateName)
{
    HandleSeq seq0;
    seq0.push_back(object);

    // testing if there is a predicate already
    Handle predicateHandle = atomSpace.getHandle(PREDICATE_NODE, predicateName);
    if (predicateHandle == Handle::UNDEFINED) {
        logger().fine("BasicPredUpdater - Predicate '%s' not found.",
                     predicateName.c_str());
        return Handle::UNDEFINED;
    }

    // testing if there is a list link already
    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, seq0);
    if (listLinkHandle == Handle::UNDEFINED) {
        logger().fine("BasicPredUpdater - Obj %s has no ListLink.",
                     atomSpace.getName(object).c_str());
        return Handle::UNDEFINED;
    }

    HandleSeq seq;
    seq.push_back(predicateHandle);
    seq.push_back(listLinkHandle);

    std::vector<Handle> allHandles;
    atomSpace.getHandlesByOutgoing(back_inserter(allHandles), seq, NULL, NULL, 2, EVALUATION_LINK, false);

    if (allHandles.size() != 1) {
        return Handle::UNDEFINED;
    }
    return *(allHandles.begin());
}

Handle BasicPredicateUpdater::getHandle(std::string objName)
{
    HandleSeq objHandle;
    atomSpace.getHandlesByName(back_inserter(objHandle), objName, OBJECT_NODE, true);

    // found no handle - ERROR
    if (objHandle.size() < 1) {
        logger().error("BasicPredUpdater - Found no Handle for SpaceMap object %s.",
                     objName.c_str());
        return Handle::UNDEFINED;
    }

    // found more than one handle - WARNING, return the first one
    // TODO: In this case, it could return the one with the more specific Object type.
    else if (objHandle.size() > 1) {
        logger().warn("BasicPredUpdater - Found more than one Handle for SpaceMap object %s. Returning the first one.", 
                       objName.c_str()
                     );
        unsigned int i;
        for ( i = 0; i < objHandle.size( ); ++i ) {
            logger().warn("BasicPredUpdater - id = %s handle = %i",
                           atomSpace.getName(objHandle[i]).c_str( ), 
                           atomSpace.getType(objHandle[i]) 
                         );
        } // for
    }

    // found exactly one handle
    return objHandle[0];
}

bool BasicPredicateUpdater::isUpdated(Handle object, std::string predicateName)
{

    if (getPredHandle(object, predicateName) == Handle::UNDEFINED) {
        return false;
    }
    return true;
}

void BasicPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp)
{
    logger().warn("BasicPredUpdater::update - Virtual method. Subclasses should implement it.");
}

void BasicPredicateUpdater::update(std::vector<Handle> & objects, Handle pet, unsigned long timestamp)
{
    for (Handle object : objects) {
        this->update(object, pet, timestamp); 
    }
}    

