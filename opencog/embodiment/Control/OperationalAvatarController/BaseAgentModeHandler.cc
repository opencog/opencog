/*
 * opencog/embodiment/Control/OperationalAvatarController/BaseAgentModeHandler.cc
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

/**
 * TODO: Get rid of AgentModeHandler. 
 *       How? There are two options.  
 *
 *       1. Move contents in handleCommand to PAI or
 *       2. Move contents in handleCommand to LanguageComprehension
 */

#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include <opencog/util/Logger.h>
#include <opencog/spacetime/SpaceServer.h>

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

#include "BaseAgentModeHandler.h"
#include "Pet.h"


using namespace opencog::oac;

BaseAgentModeHandler::BaseAgentModeHandler( Pet* agent ) :
    modeName( "PLAYING_MODE" ), agent( agent )
{
}

void BaseAgentModeHandler::handleCommand( const std::string& name, const std::vector<std::string>& arguments )
{
    if ( name == "requestedCommand" ) {
        if ( arguments.size( ) == 0 ) {
            logger().debug("BaseAgentModeHandler - command[requestedCommand] Invalid number of arguments: 0" );
            return;
        } // if
        logger().debug("BaseAgentModeHandler - Scheduling command" );
        this->commandsQueue.push( arguments );

    } 
    else if ( name == "instruction" ) {
        if ( arguments.size( ) != 3 ) {
            logger().debug("BaseAgentModeHandler::%s - Invalid instruction number of arguments: %d", __FUNCTION__, arguments.size( ) );
            return;
        } // if
        
        std::vector<std::string> tokens;
        boost::split( tokens, arguments[0], boost::is_any_of(" ") );
        boost::to_lower(tokens[0]);
        
        if (tokens[0] == "save_map") {
            this->agent->saveSpaceMapFile();
        } else if (tokens[0] == "save_vismap") {
            this->agent->getCurrentModeHandler( ).handleCommand( "saveVisMap", std::vector<std::string>() );
        } // else if

    }
    else if ( name == "notifyMapUpdate" ) {
        this->agent->sendMapToVisualDebuggerClients( spaceServer( ).getLatestMap( ) );
    } 
}

void BaseAgentModeHandler::update( void )
{
    if ( !this->commandsQueue.empty( ) && !this->agent->isRequestedCommandNotReaded( ) ) {
        std::vector<std::string> arguments = this->commandsQueue.front( );
        std::string commandName = arguments[0];
        arguments.erase(arguments.begin( ) );
        this->agent->setRequestedCommand( commandName, arguments );
        this->commandsQueue.pop( );
        logger().debug("BaseAgentModeHandler - Sending requested command to be evaluated. Command %s, # of arguments %d, remaining requests %d",
                       commandName.c_str( ), arguments.size( ), this->commandsQueue.size() );
    } // if
}
