#ifndef AGENT_H
#define AGENT_H

#include "MovableEntity.h"

namespace Spatial {

  class Agent;
  typedef boost::shared_ptr<Agent> AgentPtr;

  class Agent : public MovableEntity {
  public:

    inline Agent( const Agent& agent ) : MovableEntity( agent.id, agent.name, agent.position, agent.dimension, agent.orientation, agent.expansionRadius ) {
    }
    
    inline Agent( long id, const std::string& name, const Math::Vector3& position, const Math::Dimension3& dimension, const Math::Quaternion& orientation, double radius = 0.0 ) : MovableEntity( id, name, position, dimension, orientation, radius ) {
    }


    /**
     * Get a point that indicate the position of the agent's eye
     * @return
     */
    Math::Vector3 getEyePosition( void ) {
      return this->getPosition() + ( getDirection( ) * ( this->getLength()/2 ) ) + ( Math::Vector3::Y_UNIT * (this->getHeight()/2) );
    }
		
    virtual ENTITY_TYPE getType( void ) const = 0;

    virtual EntityPtr clone( void ) const = 0;


  }; // Agent

}; // Spatial

#endif // AGENT_H
