/*
 * opencog/spatial/PetAgent.h
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

#ifndef _SPATIAL_PETAGENT_H_
#define _SPATIAL_PETAGENT_H_

#include <opencog/spatial/Agent.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        class PetAgent;
        typedef boost::shared_ptr<PetAgent> PetAgentPtr;

        class PetAgent : public Agent
        {
        public:

            inline PetAgent( const PetAgent& agent ) : 
                Agent( agent.id, agent.name, agent.position, 
                   agent.dimension, agent.orientation, agent.expansionRadius ) 
            {
            }

            inline PetAgent( long id, const std::string& name, 
                const math::Vector3& position, const math::Dimension3& dimension, 
                    const math::Quaternion& orientation, double radius = 0.0 ) : 
                Agent( id, name, position, dimension, orientation, radius ) 
            {
            }

            inline ENTITY_TYPE getType( void ) const 
            {
                return Entity::PET_AGENT;
            }

            virtual inline EntityPtr clone( void ) const 
            {
                PetAgentPtr clone( new PetAgent( *this ) );
                clone->properties = this->properties;
                return clone;
            }

        }; // PetAgent

    } // spatial
} // opencog

#endif // _SPATIAL_PETAGENT_H_

