/*
 * opencog/embodiment/Control/MessagingSystem/DefaultAgentModeHandler.cc
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
#include "DefaultAgentModeHandler.h"
#include "Pet.h"
#include "util/Logger.h"
#include "AtomSpaceUtil.h"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "NetworkElement.h"

using namespace OperationalPetController;

DefaultAgentModeHandler::DefaultAgentModeHandler( Pet* agent ) : 
  modeName( "PLAYING_MODE" ), agent( agent ), visibilityMap(0) { 
}

void DefaultAgentModeHandler::handleCommand( const std::string& name, const std::vector<std::string>& arguments ) {   
  if ( name == "visibilityMap" ) {
    if ( arguments.size( ) == 0 ) {
      logger().log( opencog::Logger::DEBUG, "DefaultAgentModeHandler - Invalid visibility map signal, string 0 signed" );
      return;
    } // if    
    logger().log( opencog::Logger::DEBUG, "DefaultAgentModeHandler - Parsing visibility map signal" );

    Spatial::VisibilityMap* visibilityMap = getVisibilityMap( );
    if ( !visibilityMap ) {
      return;
    } // if

    std::vector<std::string> rows;
    boost::algorithm::split( rows, arguments[0], boost::algorithm::is_any_of(";") );
    logger().log( opencog::Logger::DEBUG, "DefaultAgentModeHandler - Parsed %d rows", rows.size( ) );

    unsigned int i;
    for( i = 0; i < rows.size( ); ++i ) {
      std::stringstream parser( rows[i] );
      int row;
      parser >> row;
      int col = -1;
      std::vector<int> range;
      logger().log( opencog::Logger::DEBUG, "DefaultAgentModeHandler - Parsing row %d", row );
      while( !parser.eof( ) ) {
	parser >> col;
	if ( col == -1 ) {
	  continue;
	} // if	
	range.push_back( col );
      } // while
      logger().log( opencog::Logger::DEBUG, "DefaultAgentModeHandler - Parsed %d columns for row %d", range.size( ), row );
      if ( range.size( ) == 0 || ( range.size( ) % 2 ) != 0 ) {
	logger().log( opencog::Logger::ERROR, "DefaultAgentModeHandler - The columns number of a visibility map signal must be pair and greater than 0. row: %d", row );
	return;
      } // if
      
      unsigned int j;
      int k;      
      for( j = 0; j < range.size( ); j += 2 ) {
	logger().log( opencog::Logger::DEBUG, "DefaultAgentModeHandler - Setting visibility for tiles range %d - %d", range[j], range[j+1] );
	for( k = range[j]; k <= range[j+1]; ++k ) {
	  try {
	    const Spatial::VisibilityMap::TilePtr& tile = visibilityMap->getTile( row, k );
	    logger().log( opencog::Logger::DEBUG, "DefaultAgentModeHandler - Setting visibility for tile row: %d col: %d", row, k );
	    tile->setVisibility( true );
	  } catch( opencog::NotFoundException& ex ) {
	    logger().log( opencog::Logger::ERROR, "DefaultAgentModeHandler - There was an attempt to access an invalid tile row: %d col: %d", row, k );
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
    Spatial::VisibilityMap::saveToFile( visMapName.str( ), *getVisibilityMap( ) );

  } else if ( this->agent->getType( ) == "humanoid" ) { // WARNING: keep this elseif as the last one before simple else
    if ( name == "receivedOwnerCommand" && arguments[0] == "lets_play_scavenger_hunt" ) {
      agent->setMode( SCAVENGER_HUNT );
    } else if ( name == "receivedGroupCommand" && arguments[1] == "lets_play_scavenger_hunt" && 
                arguments[0] != this->agent->getOwnerId( ) /* the owner cannot send a group command to call to play */ ) {
      agent->setMode( SCAVENGER_HUNT );
      agent->getCurrentModeHandler( ).handleCommand( name, arguments );  
    } // else if

  } else {
    logger().log(opencog::Logger::DEBUG, "DefaultAgentModeHandler - invalid command: %s", name.c_str() );
  } // else
}

Spatial::VisibilityMap* DefaultAgentModeHandler::getVisibilityMap( void ) {
  if ( this->visibilityMap != 0 ) {
    return this->visibilityMap;
  } // if

  Handle spaceMapHandle = this->agent->getAtomSpace().getSpaceServer().getLatestMapHandle();
  if (spaceMapHandle == Handle::UNDEFINED) {
    logger().log(opencog::Logger::DEBUG, "ScavengerHuntAgentModeHandler - There is no space map loaded at this moment" );
    return 0;
  } // if

  unsigned int numberOfTilesPerSide = 
    static_cast<unsigned int>( atoi( MessagingSystem::NetworkElement::parameters.get( "MAP_XDIM" ).c_str( ) ) ) / 4;

  const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
  Spatial::Math::Vector3 minimumExtent( spaceMap.xMin( ), 0, spaceMap.yMin( ) );
  Spatial::Math::Vector3 maximumExtent( spaceMap.xMax( ), 0, spaceMap.yMax( ) );
  this->visibilityMap = new Spatial::VisibilityMap( minimumExtent, maximumExtent, numberOfTilesPerSide );

  return this->visibilityMap;
}

void DefaultAgentModeHandler::update( void ) {
}
