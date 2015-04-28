/*
 * opencog/spatial/StaticEntity.h
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

#ifndef _SPATIAL_STATICENTITY_H_
#define _SPATIAL_STATICENTITY_H_

#include <opencog/spatial/Entity.h>
namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        class StaticEntity;

        typedef boost::shared_ptr<StaticEntity> StaticEntityPtr;

        class StaticEntity : public Entity
        {
        public:
            /**
             * Custom constructor
             * @param entity
             */
            StaticEntity(const StaticEntity& object ) : 
                Entity( object.id, object.name, object.position, 
                    object.dimension, object.orientation, object.expansionRadius ) 
            {
            }

            /**
             * Custom constructor
             * @param id
             * @param name
             * @param position
             * @param dimension
             * @param orientation
             */
            StaticEntity( long id, const std::string& name, const math::Vector3& position, 
                const math::Dimension3& dimension, const math::Quaternion& orientation, 
                    double radius = 0.0 ) : 
                        Entity( id, name, position, dimension, orientation, radius ) 
            {
            }

            ENTITY_TYPE getType(void ) const 
            {
                return Entity::STATIC;
            }

            virtual inline EntityPtr clone( void ) const 
            {
                StaticEntityPtr clone( new StaticEntity( *this ) );
                clone->properties = this->properties;
                return clone;
            }

        }; // StaticEntity
        
    } // spatial
} // opencog

#endif // _SPATIAL_STATICENTITY_H_
