/*
 * opencog/embodiment/Control/PredicateUpdaters/SpatialPredicateUpdater.h
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

#ifndef SPATIALPREDICATEUPDATER_H_
#define SPATIALPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spatial/Entity.h>

namespace opencog { namespace oac {

using namespace spatial;

/**
 * This class is used to update predicates of spatial relations whenever an object
 * changes its position in the latest SpaceMap. All EvaluationLinks are recorded
 * within AtTimeLinks.
 *
 * NOTE: The EvaluationLink is given the current TruthValue, as is the EvaluationLink.
 */
class SpatialPredicateUpdater : public BasicPredicateUpdater
{

public:

    SpatialPredicateUpdater(AtomSpace & _atomSpace);
    ~SpatialPredicateUpdater();
    virtual void update(Handle object, Handle pet, unsigned long timestamp);

private:

    void computeDistanceSpatialRelations(const SpaceServer::SpaceMap & spaceMap,
                                         std::vector <std::string> & entities,
                                         Handle object, 
                                         unsigned long timestamp
                                        ); 

    /**
     * @todo
     *
     * For the moment it actually calculate all the spatial relations. It's not 
     * very efficient but fast enough in current situations. This could be improved 
     * a lot. Because many other relations could be figured out from the very basic 
     * "left", "above" relations and distance. 
     */
    void computeDirectionalSpatialRelations(const SpaceServer::SpaceMap & spaceMap,
                                            std::vector <std::string> & entities,
                                            Handle objectA, Handle observer, 
                                            unsigned long timestamp
                                           );

    void addRelationsToAtomSpace(std::list<Entity::SPATIAL_RELATION> & relations, 
                                 std::string entityA_id, 
                                 std::string entityB_id, 
                                 std::string entityC_id, 
                                 AtomSpace & atomSpace, unsigned long timestamp
                                ); 

    void addSymmetricalRelation(const Handle & entityA, const Handle & entityB, 
                                const std::string & predicateName,
                                float mean, unsigned long timestamp
                               );

    unsigned long lastTimestamp;

    boost::unordered_set<std::string, boost::hash<std::string> > processedEntities;

};// class SpatialPredicateUpdater

}// namespace opencog::oac
}// namespace opencog

#endif

