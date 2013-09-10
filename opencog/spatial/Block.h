/*
 * opencog/spatial/Block.h
 *
 * Copyright (C) 2002-2011 OpenCog Foundation
 * All Rights Reserved
 * Author(s): Troy Huang
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

#ifndef _SPATIAL_BLOCK_H_
#define _SPATIAL_BLOCK_H_

#include <opencog/spatial/Entity.h>
namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        class Block;

        typedef boost::shared_ptr<Block> BlockPtr;

        class Block : public Entity
        {
        public:
            /**
             * Custom constructor
             * @param entity
             */
            Block(const Block& object ) : 
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
            Block( long id, const std::string& name, const math::Vector3& position, 
                const math::Dimension3& dimension, const math::Quaternion& orientation, 
                    double radius = 0.0 ) : 
                        Entity( id, name, position, dimension, orientation, radius ) 
            {
                this->calculateSolidBoundingBox();
            }

            inline ~Block()
            {
                delete this->solidBoundingBox;
            }

            ENTITY_TYPE getType(void ) const 
            {
                return Entity::STATIC;
            }

            virtual inline EntityPtr clone( void ) const 
            {
                BlockPtr clone( new Block( *this ) );
                clone->properties = this->properties;
                return clone;
            }

            /**
             * Calculate the solid bounding box by removing extra boundary.
             */
            void calculateSolidBoundingBox();

            inline const math::BoundingBox& getSolidBoundingBox(void) const
            {
                return *(this->solidBoundingBox);
            }

        protected:
            // The real bounding box of the object without adding extra 
            // boundary of agent's radius.
            math::BoundingBox* solidBoundingBox;
        }; // Block
        
    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_STATICENTITY_H_
