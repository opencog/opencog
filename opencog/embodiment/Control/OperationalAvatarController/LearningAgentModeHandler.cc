/*
 * opencog/embodiment/Control/OperationalAvatarController/LearningAgentModeHandler.cc
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

/**
 * TODO: Get rid of AgentModeHandler. 
 *       How? There are two options.  
 *
 *       1. Move contents in handleCommand to PAI or
 *       2. Move contents in handleCommand to LanguageComprehension
 */

#include "LearningAgentModeHandler.h"
#include <opencog/util/Logger.h>
#include "Pet.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

using namespace opencog::oac;

LearningAgentModeHandler::LearningAgentModeHandler( Pet* agent ) :
    BaseAgentModeHandler( agent ), modeName( "LEARNING_MODE" ), agent(agent)
{
}

void LearningAgentModeHandler::handleCommand( const std::string& name, const std::vector<std::string>& arguments )
{
    BaseAgentModeHandler::handleCommand( name, arguments );

    if ( name == "instruction" ) {
        if ( arguments.size( ) != 3 ) {
            logger().debug("LearningAgentModeHandler::%s - Invalid instruction. %d arguments", __FUNCTION__, arguments.size() );
            return;
        } // if
        std::vector<std::string> tokens;
        boost::split( tokens, arguments[0], boost::is_any_of(" ") );
        unsigned long timestamp = boost::lexical_cast<unsigned long>( arguments[1] );

        if ( tokens.size( ) == 3 && (tokens[1] == "WILL" || tokens[1] == "DO" || tokens[1] == "DOES")) {
            std::string exemplarAvatarId = tokens[0];
            if ( exemplarAvatarId == "I" ) {
                exemplarAvatarId = this->agent->getOwnerId( );
            } else {
                exemplarAvatarId = AtomSpaceUtil::getObjIdFromName( this->agent->getAtomSpace( ), exemplarAvatarId );
                if ( exemplarAvatarId.empty( ) ) {
                    logger().debug("LearningAgentModeHandler::%s - found no avatar with name %s", __FUNCTION__, tokens[0].c_str());
                    return;
                } // if
            } // else
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[2] );
            this->agent->startExemplar( commandStatement, timestamp );
            return;

        } else if ( tokens.size( ) == 2 && tokens[0] == "DONE" ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[1] );   
            this->agent->endExemplar(commandStatement, timestamp);            
            return;

        } else if ( tokens.size( ) == 2 && tokens[0] == "STOP" ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[2] );           
            this->agent->stopExecuting(commandStatement, timestamp);
            return;

        } else if ( tokens.size( ) == 3 && tokens[0] == "STOP" && ( tokens[1] == "LEARN" || tokens[1] == "LEARNING" ) ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[2] );           
            this->agent->stopLearning(commandStatement, timestamp);
            return;

        } else if ( tokens[0] == "TRY" ) {
            std::vector<std::string> commandStatement;
            if ( tokens.size( ) > 1 && tokens[1] != "AGAIN" ) {
                commandStatement.push_back( tokens[1] );
            } // if
            this->agent->trySchema(commandStatement, timestamp);
            return;

        } else {
            logger().debug("LearningAgentModeHandler::%s - Invalid instruction %s", __FUNCTION__, arguments[0].c_str( ) );
        } // else

    } // if
}
