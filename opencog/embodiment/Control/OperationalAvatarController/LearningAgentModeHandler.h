/*
 * opencog/embodiment/Control/OperationalAvatarController/LearningAgentModeHandler.h
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

#ifndef LEARNING_AGENT_MODE_HANDLER_H
#define LEARNING_AGENT_MODE_HANDLER_H

#include "BaseAgentModeHandler.h"

namespace opencog { namespace oac {

class Pet;
/**
 * Handle commands used by Learning mode
 */
class LearningAgentModeHandler : public BaseAgentModeHandler
{
public:
    LearningAgentModeHandler( Pet* agent );
    inline virtual ~LearningAgentModeHandler( void ) { };

    void handleCommand( const std::string& name, const std::vector<std::string>& arguments );

    inline const std::string& getModeName( void ) {
        return this->modeName;
    }

protected:
    const std::string modeName;
    Pet* agent;
};

} } // namespace opencog::oac

#endif // LEARNING_AGENT_MODE_HANDLER_H
