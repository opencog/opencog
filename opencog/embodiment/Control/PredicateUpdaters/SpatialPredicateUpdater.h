/*
 * opencog/embodiment/Control/PredicateUpdaters/SpatialPredicateUpdater.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari Heljakka, Welter Luigi
 *
 * Updated: by Zhenhua Cai, on 2011-10-24
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

    void update(std::vector<Handle> & objects, Handle pet, unsigned long timestamp);

private:

    unsigned long lastTimestamp; 

    typedef std::vector <Entity::SPATIAL_RELATION> SPATIAL_RELATION_VECTOR;
    typedef SPATIAL_RELATION_VECTOR::iterator SPATIAL_RELATION_VECTOR_ITER; 

    /**
     * Cache all the 2-size spatial relations, which would greatly accelerate 
     * 3-size spatial relations (only 'between' relation for the moment) 
     */
    class SpatialRelationCache
    {
        public:

            /**
             * Check whether spatial relations between A and B has been cached.  
             */
            bool isCached(std::string entityA_id, std::string entityB_id); 

            /**
             * Return spatial relations between A and B
             *
             * @return True, if get relations successfully, otherwise false
             */
            bool getRelation(std::string entityA_id, std::string entityB_id, 
                             SPATIAL_RELATION_VECTOR & relation);

            /**
             * Add spatial relations between A and B to the cache. 
             */
            void addRelation(std::string entityA_id, std::string entityB_id, 
                             const SPATIAL_RELATION_VECTOR & relation); 

            /**
             * Clear the cache. 
             */
            void clear(); 

        private: 

            // key: entityA_id + entityB_id, value: vector of relations
            boost::unordered_map <std::string, SPATIAL_RELATION_VECTOR> _entityRelationMap;

    }; // class SpatialRelationCache

    // Cache all the 2-size spatial relations. 
    // 3-size spatial relations calculation would rely on this cache. 
    SpatialRelationCache spatialRelationCache; 

    /**
     * Calculate all the 2-size spatial relations. 
     *
     * @spaceMap 
     *
     * @objects  Handles to latest changed objects in the virtual world. 
     *           Only spatial relations related to these objects are updated. 
     *           It is returned by PAI::processMapInfo
     *
     * @entities All entities in spaceMap involved in spatial relations. 
     *           'blocks' are not considered when computing spatial relations. 
     *
     * @observer Usually just the pet itself. 
     *
     * @timestamp 
     */
    void compute2SizeSpatialRelations(const SpaceServer::SpaceMap & spaceMap, 
                                      std::vector<Handle> & objects, 
                                      std::vector <std::string> & entities, 
                                      Handle observer, 
                                      unsigned long timestamp
                                     ); 

    /**
     * Calculate all the 3-size spatial relations (only 'between' for the moment)
     *
     * @spaceMap 
     *
     * @objects  Handles to latest changed objects in the virtual world. 
     *           Only spatial relations related to these objects are updated. 
     *           It is returned by PAI::processMapInfo
     *
     * @entities All entities in spaceMap involved in spatial relations. 
     *           'blocks' are not considered when computing spatial relations. 
     *
     * @observer Usually just the pet itself. 
     *
     * @timestamp 
     */
    void compute3SizeSpatialRelations(const SpaceServer::SpaceMap & spaceMap, 
                                      std::vector<Handle> & objects, 
                                      std::vector <std::string> & entities, 
                                      Handle observer, 
                                      unsigned long timestamp
                                     ); 

    /**
     * Return the spatial relations of B to A given that of A to B. 
     * Use this function to remove redundant calculation. 
     */
    SPATIAL_RELATION_VECTOR swapRelations(SPATIAL_RELATION_VECTOR & relations); 

    /**
     * Return whether A is between B and C given relations between A and B, A and C
     */
    bool isBetween(const SPATIAL_RELATION_VECTOR & relationsAB, 
                   const SPATIAL_RELATION_VECTOR & relationsAC
                  ); 

    /**
     * Add spatial relations to the AtomSpace. 
     *
     * It could be 2-size relations from A to B or 3-size relations among A, B and C. 
     */
    void addSpatialRelations(const SPATIAL_RELATION_VECTOR & relations, 
                             AtomSpace & atomSpace, unsigned long timestamp, 
                             Handle objetA, 
                             Handle objectB, 
                             Handle objectC = Handle::UNDEFINED
                            ); 

};// class SpatialPredicateUpdater

}// namespace opencog::oac
}// namespace opencog

#endif

