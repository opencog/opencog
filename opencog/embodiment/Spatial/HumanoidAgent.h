/*
 * opencog/embodiment/Spatial/HumanoidAgent.h
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

