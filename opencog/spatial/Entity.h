/*
 * opencog/spatial/Entity.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#ifndef _SPATIAL_ENTITY_H_
#define _SPATIAL_ENTITY_H_

#include <vector>
#include <string>
#include <set>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>

#include <opencog/spatial/math/Dimension3.h>
#include <opencog/spatial/math/BoundingBox.h>
#include <opencog/spatial/math/Vector3.h>
#include <opencog/spatial/math/Quaternion.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        class Entity;
        typedef boost::shared_ptr<Entity> EntityPtr;

        /**
         * Each element inside the virtual world is an entity.
         * This is an abstract class used to define all the kinds of entities.
         */
        class Entity
        {
        public:
            enum AXIS_LIMITS
            {
                XMIN = 0, XMAX,
                YMIN, YMAX,
                ZMIN, ZMAX
            };

            class LimitRelation
            {
            public:
                enum RELATION_AXIS
                {
                    X = 0,
                    Y,
                    Z
                };

                LimitRelation( const Entity* entityA = NULL, const Entity* entityB = NULL ) :
                    entityA( entityA ), entityB( entityB ), limitsA(6), limitsB(6), relations(3)
                {
                }

                virtual ~LimitRelation( void ) { }

                const Entity* entityA;
                const Entity* entityB;

                std::vector<std::set<math::Vector3> > limitsA;
                std::vector<std::set<math::Vector3> > limitsB;
                std::vector<unsigned int> relations;
            };

            typedef boost::variant<std::string, int, double, bool> PropertyValueType;

            /**
             * All the types of entities
             */
            enum ENTITY_TYPE
            {
                STATIC,
                MOVABLE,
                HUMANOID_AGENT,
                PET_AGENT,
                INVISIBLE
            };

            /**
             * Types of properties
             */
            enum PROPERTY
            {
                TYPE = 0,
                EDIBLE,
                DRINKABLE,
                PET_HOME,
                FOOD_BOWL,
                WATER_BOWL,
                OBSTACLE,
                ENTITY_CLASS,

                NUMBER_OF_PROPERTIES
            };

            typedef boost::unordered_map< PROPERTY, PropertyValueType, boost::hash<PROPERTY> > PropertyHashMap;

            /**
             * Comparator class used to measure the distance between two objects
             */
            class EntityDistanceComparator
            {
            private:
                const EntityPtr referenceEntity;

                /**
                 * Use a given entity as reference.
                 * The computed distance will be the referenceEntity and a given different entity
                 * @param referenceObject
                 */
                EntityDistanceComparator( const EntityPtr& referenceObject ) :
                    referenceEntity(referenceObject)
                {
                }

                virtual ~EntityDistanceComparator( void )
                {
                }

                /*
                 *
                 */
                bool operator()( const EntityPtr& o1, const EntityPtr& o2) const
                {
                    double distanceToObject1 = referenceEntity->distanceTo(*o1);
                    double distanceToObject2 = referenceEntity->distanceTo(*o2);
                    if ( distanceToObject1 < distanceToObject2 ) {
                        return true;
                    }
                    else {
                        return false;
                    }
                }
            };

            /**
             * Simple Copy constructor
             * @param object
             */
            Entity( const EntityPtr& entity );

            /**
             * Parameterized constructor
             * @param id
             * @param name
             * @param position
             * @param width
             * @param height
             * @param length
             * @param orientation
             */
            Entity( long id, const std::string& name, const math::Vector3& position,
                const math::Dimension3& dimension, const math::Quaternion& orientation,
                    double radius = 0.0 );

            inline virtual ~Entity( void )
            {
            }

            /**
             * Returns a vector pointing to the current entity direction (facing to)
             * @return
             */
            math::Vector3 getDirection( void ) const;

            /**
             * Getter for entity bounding box
             * @return
             */
            inline const math::BoundingBox& getBoundingBox( void ) const
            {
                return this->boundingBox;
            }

            /**
             * Getter for the current entity position
             * @return
             */
            inline const math::Vector3& getPosition( void ) const
            {
                return this->position;
            }

            /**
             * Getter for the current entity orientation
             * @return
             */
            inline const math::Quaternion& getOrientation( void ) const
            {
                return this->orientation;
            }

            /**
             * Getter for dimension
             * @return
             */
            inline const math::Dimension3& getDimension( void ) const
            {
                return this->dimension;
            }

            /**
             * Return the entity id
             * @return
             */
            inline const long& getId() const
            {
                return this->id;
            }

            /**
             * Return the entity name
             * @return
             */
            inline const std::string& getName( void ) const
            {
                return this->name;
            }

            /**
             * Helper method to get the entity width
             * @return
             */
            inline double getWidth( void ) const
            {
                return this->dimension.width;
            }

            /**
             * Helper method to get the entity height
             * @return
             */
            inline double getHeight(void ) const
            {
                return this->dimension.height;
            }

            /**
             * Helper method to get the entity length
             * @return
             */
            inline double getLength( void ) const
            {
                return this->dimension.length;
            }

            inline double getExpansionRadius( void ) const
            {
                return this->expansionRadius;
            }

            /**
             * Setter of the property map
             * @param property
             * @param value
             */
            void setProperty( PROPERTY property, PropertyValueType value );

            /**
             * Boolean getter of the property map
             * @param property
             * @return
             */
            bool getBooleanProperty( PROPERTY property ) const;

            /**
             * String getter of the property map
             * @param property
             * @return
             */
            std::string getStringProperty( PROPERTY property ) const;

            /**
             * Double getter of the property map
             * @param property
             * @return
             */
            double getDoubleProperty( PROPERTY property ) const;

            /**
             * Int getter of the property map
             * @param property
             * @return
             */
            int getIntProperty( PROPERTY property ) const;

            virtual ENTITY_TYPE getType( void ) const = 0;

            virtual EntityPtr clone( void ) const = 0;

            /**
             * Overloaded operators
             */
            bool operator==( const EntityPtr& entity ) const;
            bool operator!=( const EntityPtr& entity ) const;
            bool operator==( const Entity& entity ) const;
            bool operator!=( const Entity& entity ) const;

            bool intersects( const Entity& other ) const;

            /* (non-Javadoc)
             * @see java.lang.Object#toString()
             */
            std::string toString( ) const;

            double distanceTo( const Entity& entity,
                               math::Vector3* pointInA = NULL,
                               math::Vector3* pointInB = NULL,
                               LimitRelation* = NULL ) const;

            /**
             * This method computes the limits of two objects.
             * The limits are the points of the objects located
             * at its extremities on each of X,Y,Z axis
             *
             * Here are the possible types of limits
             *
             ********************* ********************* **********************
             *                     *                     *
             * 1) |--A--|          * 2)         |--A--|  * 4) |--A--|
             *            |--B--|  *    |--B--|          *        |--B--|
             *                     *                     *
             ********************* ********************* **********************
             *                     *                     *
             * 8)     |--A--|      * 16) |--A--|         * 32)       |--A--|
             *    |--B--|          *           |--B--|   *     |--B--|
             *                     *                     *
             ********************* ********************* **********************
             *                     *                     *
             * 64) |--A--|         * 128)  |--A--|       * 256) |----A----|
             *     |--B--|         *     |----B----|     *        |--B--|
                                   *                     *
             ********************* ********************* **********************
             *                     *
             * 512) |--A--|        * 1024) |----A----|
             *      |----B----|    *       |--B--|
             *                     *
             *          |--A--|    *       |----A----|
             *      |----B----|    *           |--B--|
             *                     *
             ******************************************************************
             *
             * Note that the number of the limit is
             * the code used to classify the limits relation between
             * the objects.
             *
             * i.e. a returning vector with the following configuration:
             *      relations[0] = 1024 relations[1] = 16 relations[2] = 16
             * means that in the X axis the relation 1024 was found and in the Y and Z axis
             * the relation is 16
             *
             * @param entityB The second entity which will have its limits computed
             * @return Entity::LimitRelation A vector containing the codes of the relations in each three Axis
             */
            LimitRelation computeObjectsLimits( const Entity& entityB ) const;

            /*
             * Extract the spatial relations between two objects
             *
             * @param observer The observer entity
             * @param besideDistance A distance used as threshold for considering
             *                       an object beside or not another
             * @param entityB The entity used as reference entity
             * @return std::vector<SPATIAL_RELATION> a vector of all spatial relations
             *         between entityA (this entity) and entityB (reference entity)
             *
             * @note entityA is 'this' entity, that is the entity launches this function.
             */
            /* std::vector<SPATIAL_RELATION> computeSpatialRelations( const Entity& observer,
                                                                   double besideDistance,
                                                                   const Entity& entityB ) const;
            */
            /*
             * Finds the list of spatial relationships that apply to the three entities.
             * Currently this can only be BETWEEN, which states that A is between B and C
             *
             * @param observer The observer entity
             * @param besideDistance A distance used as threshold for considering
             *                       an object beside or not another
             * @param entityB First reference entity
             * @param entityC Second reference entity
             *
             * @return std::vector<SPATIAL_RELATION> a vector of all spatial relations
             *         among entityA (this entity), entityB (first reference) and entityC
             *         (second reference)
             *
             * @note entityA is 'this' entity, that is the entity launches this function.
             */
            /*std::vector<SPATIAL_RELATION> computeSpatialRelations( const Entity& observer,
                                                                   double besideDistance,
                                                                   const Entity& entityB,
                                                                   const Entity& entityC ) const;
*/
            /*
             * Return a string description of the relation
             */
         //   static std::string spatialRelationToString( SPATIAL_RELATION relation );

        protected:
            long id;
            std::string name;

            math::Dimension3 dimension;

            // the entity is positioned at a specific point on space
            math::Vector3 position;
            // the entity is oriented to a specific point on space
            math::Quaternion orientation;

            double expansionRadius;

            // bounding box dimensions
            math::BoundingBox boundingBox;

            // properties map
            PropertyHashMap properties;

        }; // Entity

        typedef boost::unordered_map<long, EntityPtr, boost::hash<long> > LongEntityPtrHashMap;

    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_ENTITY_H_
