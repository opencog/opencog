/*
 * opencog/embodiment/Control/OperationalAvatarController/DefaultAgentModeHandler.cc
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

#include "DefaultAgentModeHandler.h"
#include "Pet.h"

#include <opencog/util/Logger.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>

using namespace opencog;
using namespace opencog::oac;

DefaultAgentModeHandler::DefaultAgentModeHandler( Pet* agent ) :
    BaseAgentModeHandler( agent ), modeName( "PLAYING_MODE" ), 
    agent( agent ), visibilityMap(0)
{
}

void DefaultAgentModeHandler::handleCommand( const std::string& name, const std::vector<std::string>& arguments )
{
    BaseAgentModeHandler::handleCommand( name, arguments );

    if ( name == "instruction" ) {
        if ( arguments.size( ) != 3 ) {
            logger().debug("DefaultAgentModeHandler::%s - Invalid instruction. %d arguments", __FUNCTION__, arguments.size() );
            return;
        } // if
        std::vector<std::string> tokens;
        boost::split( tokens, arguments[0], boost::is_any_of(" ") );
        
        if ( tokens[0] == "LEARN" ) {
            logger().debug("DefaultAgentModeHandler::%s - Parsing LEARN instruction[%s]",
                           __FUNCTION__, arguments[0].c_str( ) );

            std::string avatarId = this->agent->getOwnerId( );
            if ( tokens.size( ) == 4 && tokens[2] == "WITH" ) {
                std::string avatarId = AtomSpaceUtil::getObjIdFromName( this->agent->getAtomSpace(), tokens[3] );
                if ( avatarId.empty() ) {
                    logger().debug("DefaultAgentModeHandler::%s - found no avatar with name %s", 
                                   __FUNCTION__, tokens[3].c_str());
                    // There is no object/avatar with such name
                    return;
                } // if                
            } // with
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[1] );
            unsigned long timestamp = boost::lexical_cast<unsigned long>( arguments[1] );
            this->agent->setExemplarAvatarId( avatarId );
            this->agent->startLearning( commandStatement, timestamp );
            return;

        } else if ( tokens[0] == "DO" ) {
            if ( tokens.size( ) < 2 ) {
                logger().debug("DefaultAgentModeHandler::%s - invalid DO command. it should be DO <something> <args>, but was: %s", 
                               __FUNCTION__, arguments[0].c_str());
                return;
            } // if
            std::vector<std::string> command;
            command.push_back( tokens[1] );
            unsigned int i;
            for( i = 2; i < tokens.size( ); ++i ) {
                std::string param = AtomSpaceUtil::getObjIdFromName( this->agent->getAtomSpace(), tokens[i] );
                command.push_back( param.length( ) > 0 ? param : tokens[i] );
            } // for
            handleCommand( "requestedCommand", command );
            return;

        } else if ( tokens.size( ) > 2 && tokens[0] == "LETS" && tokens[1] == "PLAY" && 
                    ( tokens[2] == "SH" || ( tokens.size( ) == 4 && tokens[2] == "SCAVENGER" && tokens[3] == "HUNT") ) ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( "lets_play_scavenger_hunt" );
            handleCommand( "receivedOwnerCommand", commandStatement );
            logger().debug("DefaultAgentModeHandler::%s - Starting to play scavenger hunt", __FUNCTION__ );
            return;

        } else {
            logger().debug("DefaultAgentModeHandler::%s - Invalid instruction %s", __FUNCTION__, arguments[0].c_str( ) );
            return;
        } // else
        
    } else if ( name == "visibilityMap" ) {
        if ( arguments.size( ) == 0 ) {
            logger().debug("DefaultAgentModeHandler - Invalid visibility map signal, string 0 signed" );
            return;
        } // if
        logger().debug("DefaultAgentModeHandler - Parsing visibility map signal" );

        spatial::VisibilityMap* visibilityMap = getVisibilityMap( );
        if ( !visibilityMap ) {
            return;
        } // if

        std::vector<std::string> rows;
        boost::algorithm::split( rows, arguments[0], boost::algorithm::is_any_of(";") );
        logger().debug("DefaultAgentModeHandler - Parsed %d rows", rows.size( ) );

        unsigned int i;
        for ( i = 0; i < rows.size( ); ++i ) {
            std::stringstream parser( rows[i] );
            int row;
            parser >> row;
            int col = -1;
            std::vector<int> range;
            logger().debug("DefaultAgentModeHandler - Parsing row %d", row );
            while ( !parser.eof( ) ) {
                parser >> col;
                if ( col == -1 ) {
                    continue;
                } // if
                range.push_back( col );
            } // while
            logger().debug("DefaultAgentModeHandler - Parsed %d columns for row %d", range.size( ), row );
            if ( range.size( ) == 0 || ( range.size( ) % 2 ) != 0 ) {
                logger().error("DefaultAgentModeHandler - The columns number of a visibility map signal must be pair and greater than 0. row: %d", row );
                return;
            } // if

            unsigned int j;
            int k;
            for ( j = 0; j < range.size( ); j += 2 ) {
                logger().debug("DefaultAgentModeHandler - Setting visibility for tiles range %d - %d", range[j], range[j+1] );
                for ( k = range[j]; k <= range[j+1]; ++k ) {
                    try {
                        const spatial::VisibilityMap::TilePtr& tile = visibilityMap->getTile( row, k );
                        logger().debug("DefaultAgentModeHandler - Setting visibility for tile row: %d col: %d", row, k );
                        tile->setVisibility( true );
                    } catch ( opencog::NotFoundException& ex ) {
                        logger().error("DefaultAgentModeHandler - There was an attempt to access an invalid tile row: %d col: %d", row, k );
                    } // catch
                } // for
            } // for

        } // for
    } else if ( name == "saveVisMap" ) {
        static int visMapCounter = 0;
        std::stringstream visMapName;
        visMapName << "visMap_defaultMode_";
        visMapName << this->agent->getPetId( );
        visMapName << "_";
        visMapName << visMapCounter++;
        visMapName << ".bin";
        spatial::VisibilityMap::saveToFile( visMapName.str( ), *getVisibilityMap( ) );

    } else if ( this->agent->getType( ) == "humanoid" ) { // WARNING: keep this elseif as the last one before simple else
        if ( name == "receivedOwnerCommand" && arguments[0] == "lets_play_scavenger_hunt" ) {
            agent->setMode( SCAVENGER_HUNT );
        } else if ( name == "receivedGroupCommand" && arguments[1] == "lets_play_scavenger_hunt" &&
                    arguments[0] != this->agent->getOwnerId( ) /* the owner cannot send a group command to call to play */ ) {
            agent->setMode( SCAVENGER_HUNT );
            agent->getCurrentModeHandler( ).handleCommand( name, arguments );
        } // else if

    } else {
        logger().debug("DefaultAgentModeHandler - invalid command: %s", name.c_str() );
    } // else
}

spatial::VisibilityMap* DefaultAgentModeHandler::getVisibilityMap( void )
{
    if ( this->visibilityMap != 0 ) {
        return this->visibilityMap;
    } // if

    Handle spaceMapHandle = this->agent->getAtomSpace().getSpaceServer().getLatestMapHandle();
    if (spaceMapHandle == Handle::UNDEFINED) {
        logger().debug("ScavengerHuntAgentModeHandler - There is no space map loaded at this moment" );
        return 0;
    } // if

    unsigned int numberOfTilesPerSide =
        static_cast<unsigned int>(opencog::config().get_int( "MAP_XDIM" )) / 4;

    const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
    spatial::math::Vector3 minimumExtent( spaceMap.xMin( ), 0, spaceMap.yMin( ) );
    spatial::math::Vector3 maximumExtent( spaceMap.xMax( ), 0, spaceMap.yMax( ) );
    this->visibilityMap = new spatial::VisibilityMap( minimumExtent, maximumExtent, numberOfTilesPerSide );

    return this->visibilityMap;
}
