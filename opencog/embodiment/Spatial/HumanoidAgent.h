#ifndef HUMANOIDAGENT_H
#define HUMANOIDAGENT_H

#include "Agent.h"

namespace Spatial {


  class HumanoidAgent;
  typedef boost::shared_ptr<HumanoidAgent> HumanoidAgentPtr;

  class HumanoidAgent : public Agent {
  public:

    inline HumanoidAgent( const HumanoidAgent& agent ) : Agent( agent.id, agent.name, agent.position, agent.dimension, agent.orientation, agent.expansionRadius ) {
    }
    
    inline HumanoidAgent( long id, const std::string& name, const Math::Vector3& position, const Math::Dimension3& dimension, const Math::Quaternion& orientation, double radius = 0.0 ) : Agent( id, name, position, dimension, orientation, radius ) {
    }
    
    inline ENTITY_TYPE getType( void ) const {
      return Entity::HUMANOID_AGENT;
    }

    virtual inline EntityPtr clone( void ) const {
      HumanoidAgentPtr clone( new HumanoidAgent( *this ) );
      clone->properties = this->properties;
      return clone;
    }


  }; // HumanoidAgent

}; // Spatial

#endif // HUMANOIDAGENT_H

