/*
 * opencog/embodiment/Control/PredicateUpdaters/NearPredicateUpdater.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari Heljakka, Welter Luigi
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

#ifndef NEARPREDICATEUPDATER_H_
#define NEARPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"
#include <opencog/atomspace/AtomSpace.h>

namespace opencog { namespace oac {

/**
 * This class is used to update the near predicates whenever an object
 * changes its position in the latest SpaceMap. All EvaluationLinks are recorded
 * within AtTimeLinks. NOTE: The EvaluationLink is given the current TruthValue,
 * as is the EvaluationLink.
 * @todo Uses the (previously) separate code to handle other spatial relationships. This should be unified more.
 * @todo computeAllSpatialRelations could be made much more efficient through avoiding redundant computation.
 * @todo computeAllSpatialRelations should check which objects are near to each other using the spatial grid system.
 * @todo computeAllSpatialRelations doesn't handle the "between" relation (connected to the above)
 */
class NearPredicateUpdater : public BasicPredicateUpdater
{

public:

    NearPredicateUpdater(AtomSpace& _atomSpace);
    ~NearPredicateUpdater();

    virtual void update(Handle object, Handle pet, unsigned long timestamp );

    virtual void computeAllSpatialRelations(Handle objectA, Handle observer, unsigned long timestamp);

protected:

    void setPredicate( const Handle& entityA, const Handle& entityB, const std::string& predicateName, float mean, unsigned long timestamp );

    unsigned long lastTimestamp;

    boost::unordered_set<std::string, boost::hash<std::string> > processedEntities;



}; // class

} } // namespace opencog::oac

#endif

