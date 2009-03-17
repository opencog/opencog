#ifndef PETAGENT_H
#define PETAGENT_H

#include "Agent.h"

namespace Spatial {

  class PetAgent;
  typedef boost::shared_ptr<PetAgent> PetAgentPtr;

  class PetAgent : public Agent {
  public:

    inline PetAgent( const PetAgent& agent ) : Agent( agent.id, agent.name, agent.position, agent.dimension, agent.orientation, agent.expansionRadius ) {
    }
    
    inline PetAgent( long id, const std::string& name, const Math::Vector3& position, const Math::Dimension3& dimension, const Math::Quaternion& orientation, double radius = 0.0 ) : Agent( id, name, position, dimension, orientation, radius ) {
    }
    
    inline ENTITY_TYPE getType( void ) const {
      return Entity::PET_AGENT;
    }

    virtual inline EntityPtr clone( void ) const {
      PetAgentPtr clone( new PetAgent( *this ) );
      clone->properties = this->properties;
      return clone;
    }

  }; // PetAgent

}; // Spatial

#endif // PETAGENT_H

