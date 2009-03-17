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
