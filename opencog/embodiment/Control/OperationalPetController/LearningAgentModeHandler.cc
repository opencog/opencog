/*
 * opencog/embodiment/Control/MessagingSystem/LearningAgentModeHandler.cc
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
#include "LearningAgentModeHandler.h"
#include "util/Logger.h"
#include "Pet.h"

using namespace OperationalPetController;

LearningAgentModeHandler::LearningAgentModeHandler( Pet* agent ) :
        modeName( "LEARNING_MODE" ), agent(agent)
{
}

void LearningAgentModeHandler::handleCommand( const std::string& name, const std::vector<std::string>& arguments )
{
}

void LearningAgentModeHandler::update( void )
{
}
