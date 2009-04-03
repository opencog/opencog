/*
 * opencog/spatial/SuperEntity.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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
#ifndef _SPATIAL_SUPERENTITY_H_
#define _SPATIAL_SUPERENTITY_H_

#include <ext/hash_map>

#include <list>

#include "Entity.h"
#include "Math/LineSegment.h"
#include "Math/Rectangle.h"

#include <opencog/util/exceptions.h>

#include <boost/shared_ptr.hpp>

namespace Spatial {
    /**
     * When two or more entities are too near, they become a single obstacle.
     * A super entity represents an obstacle created these near obstacles.
     */

    class SuperEntity;
    typedef boost::shared_ptr<SuperEntity> SuperEntityPtr;

    class SuperEntity {
    public:
  
        /**
         * A subentity is an entity representation (cache) inside SuperEntity
         */
        class SubEntity {
        public:
            long id;
            // bounding rectangle used for intersection tests
            Math::Rectangle rectangle;
            // original entity edges
            std::list<Math::LineSegment> edges;
            // split edges. each original edge can be split into several pieces that will be stored in array below
            std::list<Math::LineSegment> splitEdges[4];
                    
            SubEntity( long id, const Math::Rectangle& rectangle, const std::list<Math::LineSegment>& edges ) :
                id(id), rectangle( rectangle ) {
                this->edges = edges;
                reset( );
            } // if 

            /**
             * Put all split edges into a single Edge list and return it
             */
            std::list<Math::LineSegment> getSplitEdges( void ) {
                std::list<Math::LineSegment> response;
                std::back_insert_iterator<std::list<Math::LineSegment> > ii( response );
                unsigned int i;
                for( i = 0; i < 4; ++i ) {
                    std::copy( splitEdges[i].begin( ), splitEdges[i].end( ), ii );
                } // for
                return response;
            }

            /**
             * Clear the cached split edges array and put the original edges into the first position of each
             * list element of the splitEdges array
             */
            void reset( void ) {
                std::list<Math::LineSegment>::iterator it;;
                unsigned int i;
                for( it = edges.begin( ), i = 0; i < 4 && it != edges.end( ); ++i, ++it ) {
                    splitEdges[i].clear( );
                    Math::LineSegment seg = *it;
                    seg.pointA.z = 0;
                    seg.pointB.z = 0;
                    splitEdges[i].push_back( seg );
                } // for
            } // if

            virtual ~SubEntity( void ) { };

        }; // SubEntity
    
        typedef boost::shared_ptr<SubEntity> SubEntityPtr;
        typedef __gnu_cxx::hash_map<long, SubEntityPtr, __gnu_cxx::hash<long> > LongSubEntityPtrHashMap;  

        /**
         * Create a super entity. If the parameters are entities that did not intersect 
         * each other an InvalidParamException will be raised to notify the error
         */
        SuperEntity( const EntityPtr& entity1, const EntityPtr& entity2 ) throw (opencog::InvalidParamException);

        virtual ~SuperEntity( void );

        /**
         * Check if a point is inside the whole SuperEntity (Inside at least 
         * one of the n entities that compounds these super entity
         */
        bool isInside( const Math::Vector3& point ) const;    


        /**
         * Get a list of the super entity corners
         */
        std::list<Math::Vector3> getCorners( void ) const;

        /**
         * Merge an entity into this superentity. It will return true if merged or false if the given entity does not 
         * intersects the super entity
         */
        bool merge( const EntityPtr& entity );
        /**
         * Merge a super entity into this superentity. It will return true if merged or false 
         * if the given superentity does not intersects the super entity
         */
        bool merge( const SuperEntityPtr& entity );

        /**
         * Check if the given id belongs to an entity which is part of this superentity
         */
        bool containsEntity( long id ) const;
    

        /**
         * Remove an entity from the superEntity. It will throw an InvalidParamException if these
         * operation results in an invalid superentity (just one entity or far entities)
         */
        void removeEntity( long id ) throw (opencog::InvalidParamException);

        /**
         * Create a copy of this super entity
         */
        SuperEntityPtr clone( void ) const;
    
        /**
         * Getter for the current SuperEntity edges;
         */
        inline const std::list<Math::LineSegment>& getEdges( void ) const {
            return this->segments;
        };

        std::vector<long> getSubEntitiesIds( void ) const;

    private:

        inline SuperEntity( void ) { }; // used to clone the object
    
        /**
         * Recompute the whole subentities edges
         */
        bool rebuild( void );

        bool mergeSubEntity( const SubEntityPtr& subEntity );

        /**
         * Compute the edges resulted from a intersection of two entities.
         * Return true if the entities overlap, false otherwise
         */
        bool splitEdges( const SubEntityPtr& subEntity1, const SubEntityPtr& subEntity2 );

        SubEntityPtr createSubEntity( const EntityPtr& entity );

        LongSubEntityPtrHashMap subEntities;
        std::list<Math::LineSegment> segments;    

    }; // SuperEntity

} // Spatial

#endif // _SPATIAL_SUPERENTITY_H_
