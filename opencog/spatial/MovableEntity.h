/*
 * opencog/spatial/MovableEntity.h
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

#ifndef _SPATIAL_MOVABLEENTITY_H_
#define _SPATIAL_MOVABLEENTITY_H_

#include <opencog/spatial/Entity.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        class MovableEntity;
        typedef boost::shared_ptr<MovableEntity> MovableEntityPtr;
        
        class MovableEntity : public Entity
        {
        public:
            
            /**
             * Simple Copy constructor
             * @param object
             */
            inline MovableEntity( const MovableEntity& object ) : 
                Entity( object.id, object.name, object.position, object.dimension, 
                    object.orientation, object.expansionRadius ), 
                        velocity( object.getVelocity() ) { };
           
            /**
             * Custom constructor
             * @param id
             * @param name
             * @param position
             * @param dimension
             * @param orientation
             */
            inline MovableEntity( long id, const std::string& name, 
                const math::Vector3& position, const math::Dimension3& dimension, 
                    const math::Quaternion& orientation, double radius = 0.0 ) : 
                        Entity( id, name, position, dimension, orientation, radius ), 
                velocity( 0 ) 
            { 
            }
            
            /**
             * Update it's internal structures after a move or rotation action
             */
            void update( void ) 
            {
                if ( needUpdate ) {
                    boundingBox.update( );
                } // if
            }
            
            /**
             * Rotate this entity by a given quaternion
             * @param rotation
             */
            void rotate( const math::Quaternion& rotation );

            /**
             * Defines a new position for this entity
             * @param position
             */
            void setPosition(const math::Vector3& position);

            /**
             * Defines a new orientation for this entity
             * @param orientation
             */
            void setOrientation(const math::Quaternion& orientation);

            /**
             * Getter to the entity velocity
             * @return
             */
            inline double getVelocity( void ) const 
            {
                return this->velocity;
            }

            /**
             * Set the velocity of the entity
             * @param velocity
             */
            inline void setVelocity(double velocity) 
            {
                this->velocity = velocity;
            }

            virtual inline ENTITY_TYPE getType( void ) const 
            {
                return Entity::MOVABLE;
            }

            virtual inline EntityPtr clone( void ) const 
            {
                MovableEntityPtr clone( new MovableEntity( *this ) );
                clone->setVelocity( getVelocity( ) );
                clone->properties = this->properties;
                return clone;
            }

        private:
            double velocity;
            bool needUpdate;

        }; // MovableEntity

    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_MOVABLEENTITY_H_
