/*
 * opencog/embodiment/Control/OperationalAvatarController/BaseAgentModeHandler.h
 *
 * Copyright (C) 2009 Novamente LLC
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
#ifndef BASE_AGENT_MODE_HANDLER
#define BASE_AGENT_MODE_HANDLER

#include <opencog/spatial/VisibilityMap.h>
#include <queue>

#include "../AgentModeHandler.h"

namespace opencog { namespace oac {

class Pet;
/**
 * If a given mode doesn't need a handler, use a base handler to be returned by AvatarInterface
 */
class BaseAgentModeHandler : public opencog::control::AgentModeHandler
{
public:

    BaseAgentModeHandler( Pet* agent );

    virtual inline ~BaseAgentModeHandler( void ) { };

    void handleCommand( const std::string& name, const std::vector<std::string>& arguments );

    inline const std::string& getModeName( void ) {
        return this->modeName;
    }

    void update( void );

protected:
    const std::string modeName;
    std::queue<std::vector<std::string> > commandsQueue;
    Pet* agent;
};

} } // namespace opencog::oac

#endif // BASE_AGENT_MODE_HANDLER
