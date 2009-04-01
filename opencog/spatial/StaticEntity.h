/*
 * opencog/spatial/StaticEntity.h
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
#ifndef STATICENTITY_H
#define STATICENTITY_H

#include "Entity.h"
namespace Spatial {

  class StaticEntity;

  typedef boost::shared_ptr<StaticEntity> StaticEntityPtr;

  class StaticEntity : public Entity {
  public:
    /**
     * Custom constructor
     * @param entity
     */
    StaticEntity(const StaticEntity& object ) : Entity( object.id, object.name, object.position, object.dimension, object.orientation, object.expansionRadius ) {
    }

    /**
     * Custom constructor
     * @param id
     * @param name
     * @param position
     * @param dimension
     * @param orientation
     */
    StaticEntity(  long id, const std::string& name, const Math::Vector3& position, const Math::Dimension3& dimension, const Math::Quaternion& orientation, double radius = 0.0 ) : Entity( id, name, position, dimension, orientation, radius ) {
    }
	
    ENTITY_TYPE getType(void ) const {
      return Entity::STATIC;
    }

    virtual inline EntityPtr clone( void ) const {
      StaticEntityPtr clone( new StaticEntity( *this ) );
      clone->properties = this->properties;
      return clone;
    }

  }; // StaticEntity

}; // Spatial

#endif // STATICENTITY_H
