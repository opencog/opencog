/*
 * opencog/embodiment/Control/OperationalAvatarController/ScavengerHuntAgentModeHandler.cc
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

#include "ScavengerHuntAgentModeHandler.h"
#include "RuleEngine.h"
#include "Pet.h"

#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/AvatarInterface.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/util/Logger.h>

#include <vector>
#include <cstdlib>
#include <sstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

using namespace opencog::oac;

const unsigned int ScavengerHuntAgentModeHandler::NUMBER_OF_AREAS = 9;
const std::string ScavengerHuntAgentModeHandler::RED_TEAM_BASE("red_base");
const std::string ScavengerHuntAgentModeHandler::BLUE_TEAM_BASE("blue_base");

ScavengerHuntAgentModeHandler::ScavengerHuntAgentModeHandler( Pet* agent ) :
    BaseAgentModeHandler( agent ),
    modeName( "SCAVENGER_HUNT_MODE" ), agent(agent), visibilityMap(0), agentState(0),
    previousAgentState(0), exploringArea(NUMBER_OF_AREAS), playing(false),
    runningProcedureId(0), needToCollectProcedureId(false)
{
}

void ScavengerHuntAgentModeHandler::changeState( int from, int to )
{
    if ( from == to ) {
        // the agent is already in that state
        return;
    } // if

    Handle agentHandle = AtomSpaceUtil::getAgentHandle( agent->getAtomSpace( ), agent->getPetId( ) );
    if ( agentHandle == Handle::UNDEFINED ) {
        return;
    } // if
    std::string fromStr;
    std::string toStr;
    {
        std::stringstream parser;
        parser << from;
        fromStr = parser.str( );
    }
    {
        std::stringstream parser;
        parser << to;
        toStr = parser.str( );
    }

    Handle fromStateNodeHandle = agent->getAtomSpace( ).addNode( NUMBER_NODE, fromStr );
    Handle toStateNodeHandle = agent->getAtomSpace( ).addNode( NUMBER_NODE, toStr );

    AtomSpaceUtil::setPredicateValue( agent->getAtomSpace( ), "agentModeState",
                                      SimpleTruthValue( 0.0, 1.0 ), agentHandle, fromStateNodeHandle );

    AtomSpaceUtil::setPredicateValue( agent->getAtomSpace( ), "agentModeState",
                                      SimpleTruthValue( 1.0, 1.0 ), agentHandle, toStateNodeHandle );

    logger().debug("ScavengerHuntAgentModeHandler - Changed state from %d to %d", from, to );

    this->previousAgentState = this->agentState;
    this->agentState = to;
}

void ScavengerHuntAgentModeHandler::handleCommand( const std::string& name, const std::vector<std::string>& arguments )
{

    BaseAgentModeHandler::handleCommand( name, arguments );

    std::stringstream args;
    std::copy(arguments.begin(), arguments.end(), std::ostream_iterator<std::string>(args, ", "));


    std::vector<std::string> lowerCaseArguments;
    unsigned int i;
    for ( i = 0; i < arguments.size( ); ++i ) {
        lowerCaseArguments.push_back( arguments[i] );
        boost::to_lower( lowerCaseArguments[i] );
    } // for

    logger().debug("ScavengerHuntAgentModeHandler - handlingCommand[%s] params[%s]", name.c_str( ), args.str( ).c_str( ) );

    if ( name == "instruction" ) {
        if ( arguments.size( ) != 3 ) {
            logger().debug("LearningAgentModeHandler::%s - Invalid instruction. %d arguments", __FUNCTION__, arguments.size() );
            return;
        } // if
        std::vector<std::string> tokens;
        boost::split( tokens, arguments[0], boost::is_any_of(" ") );
        //long timestamp = boost::lexical_cast<long>( arguments[1] );        
        
        if ( tokens.size( ) > 2 && tokens[0] == "GO" && ( tokens[1] == "FIND" || tokens[1] == "AFTER" ) && 
             ( tokens[2] == "TREASURE" || tokens[2] == "TREASURES" ) ) {
            std::vector<std::string> commandStatement;
            if ( tokens.size( ) == 4 ) {
                commandStatement.push_back( tokens[3] );
            } // if
            this->agent->getCurrentModeHandler( ).handleCommand( "goFindTreasure", commandStatement );
            return;
        } else if ( tokens[0] == "FOLLOW" && tokens[1] == "ME" && ( tokens.size( ) == 3 || tokens.size( ) == 4 ) ) {
            if ( tokens.size( ) == 3 ) {
                std::vector<std::string> commandStatement;
                commandStatement.push_back( tokens[2] );
                commandStatement.push_back( arguments[2] );
                this->agent->getCurrentModeHandler( ).handleCommand( "followMe", commandStatement );
                return;
            } else if ( tokens.size( ) == 4 ) {
                std::vector<std::string> commandStatement;
                commandStatement.push_back( tokens[3] );
                commandStatement.push_back( arguments[2] );
                this->agent->getCurrentModeHandler( ).handleCommand( "followMeAndCollectTreasures", commandStatement );
                return;
            } // else if
        } else if ( tokens[0] == "COME" && tokens[1] == "HERE" && tokens.size( ) == 3 ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[2] );
            commandStatement.push_back( arguments[2] );
            this->agent->getCurrentModeHandler( ).handleCommand( "comeHere", commandStatement );
            return;
        } else if ( tokens[0] == "EXPLORE" && tokens[1] == "AREA" && tokens.size( ) == 4  ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[2] );
            commandStatement.push_back( tokens[3] );
            commandStatement.push_back( arguments[2] );
            this->agent->getCurrentModeHandler( ).handleCommand( "exploreArea", commandStatement );
            return;
        } else if ( tokens[0] == "WAIT" && tokens.size( ) == 2  ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[1] );
            commandStatement.push_back( arguments[2] );
            this->agent->getCurrentModeHandler( ).handleCommand( "wait", commandStatement );
            return;
        } else if ( tokens[0] == "REGROUP" && tokens[1] == "TEAM" && tokens.size( ) == 3 ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[2] );
            commandStatement.push_back( arguments[2] );
            this->agent->getCurrentModeHandler( ).handleCommand( "regroupTeam", commandStatement );
            return;
        } else if ( tokens[0] == "SPY" && tokens.size( ) == 3 ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( tokens[1] );
            commandStatement.push_back( tokens[2] );
            commandStatement.push_back( arguments[2] );
            this->agent->getCurrentModeHandler( ).handleCommand( "spy", commandStatement );
            return;
        } else if ( tokens.size( ) == 2 && tokens[0] == "I" && ( tokens[1] == "BLUE" || tokens[1] == "RED" ) && tokens[2] == "TEAM"  ) {
            std::vector<std::string> commandStatement;
            commandStatement.push_back( arguments[2] );
            commandStatement.push_back( "lets_play_scavenger_hunt" );
            commandStatement.push_back( (tokens[1] == "BLUE") ? "1" : "0" ); // 0 = RED, 1 = BLUE
            commandStatement.push_back( "1.0" ); // greatest rand double
            this->agent->getCurrentModeHandler( ).handleCommand( "receivedGroupCommand", commandStatement );
            return;

        } else {
            logger().debug("ScavengerHuntAgentModeHandler::%s - Invalid instruction %s", __FUNCTION__, arguments[0].c_str( ) );            
        } // else
        
    } else if ( name == "groupCommandDone" ) {
        if ( arguments[0] == "lets_play_scavenger_hunt" ) {
            if ( !this->playing ) {
                resetGame( );
                this->playing = true;

                addPlayer( this->agent->getPetId( ), atoi( arguments[1].c_str( ) ), atof( arguments[2].c_str( ) ) );

                this->redBaseId = AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), RED_TEAM_BASE );
                this->blueBaseId = AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), BLUE_TEAM_BASE );
            } // if
            changeState( this->agentState, 11 ); // wait for answers
        } else if ( arguments[0] == "desired_area_to_explore" ) {
            unsigned int chosenArea = static_cast<unsigned int>(atof( arguments[1].c_str() ) * 1000.0) % NUMBER_OF_AREAS;
            selectArea( this->agent->getPetId( ), chosenArea, atof( arguments[2].c_str() ) );
            this->elapsedTicks = 0;
            changeState( this->agentState, 1303 ); // wait for a short period for answers


        } else if ( arguments[0] == "game_started" ) {
            this->elapsedTicks = 2;
            changeState( this->agentState, 1317 ); // this state is used only by this mode handler
        } else if ( arguments[0] == "explored_area" ) {
            unsigned int exploredArea = 0;
            std::stringstream parser;
            parser << arguments[1];
            parser >> exploredArea;
            this->exploredAreas[exploredArea] = true;
            this->exploringArea = NUMBER_OF_AREAS;
            this->elapsedTicks = 2;
            changeState( this->agentState, 1317 ); // go back to work
        } // else

    } else if ( name == "receivedGroupCommand" ) {
        if ( arguments[1] == "lets_play_scavenger_hunt" ) {
            if ( !this->playing ) {
                resetGame( );
                this->playing = true;

                if ( this->agentState != 1304 && this->agentState != 11 ) { // i've not sent yet my call to play
                    changeState( this->agentState, 2 ); // answer the request accepting to play the game
                } // if

                this->redBaseId = AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), RED_TEAM_BASE );
                this->blueBaseId = AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), BLUE_TEAM_BASE );

                addPlayer( this->agent->getPetId( ), atoi( arguments[1].c_str( ) ), atof( arguments[2].c_str( ) ) );

            } // if
            addPlayer( arguments[0], atoi( arguments[2].c_str( ) ), atof( arguments[3].c_str( ) ) );

        } else if ( arguments[1] == "desired_area_to_explore" ) {
            unsigned int chosenArea = static_cast<unsigned int>(atof( arguments[2].c_str() ) * 1000.0) % NUMBER_OF_AREAS;
            selectArea( arguments[0], chosenArea, atof( arguments[3].c_str( ) ) );
            this->elapsedTicks = 0;
            changeState( this->agentState, 1303 ); // wait for a short period for answers

        } else if ( arguments[1] == "game_started" ) {
            changeState( this->agentState, 1317 ); // game started

        } else if ( arguments[1] == "explored_area" ) { // other agent finished the exploration of a given area
            unsigned int exploredArea = 0;
            std::stringstream parser;
            parser << arguments[2];
            parser >> exploredArea;
            this->exploredAreas[exploredArea] = true;
            if ( exploredArea == this->exploringArea ) {
                this->exploringArea = NUMBER_OF_AREAS;
                this->elapsedTicks = 2;
                changeState( this->agentState, 1317 ); // go back to work
            } // if
        } // else if

    } else if ( name == "gotoDone" ) {
        if ( !this->needToCollectProcedureId ) {
            this->needToCollectProcedureId = true;
        } // if

        if ( this->previousAgentState == 11 ) {
            changeState( this->agentState, 1304 ); // wait for game start
        } else {
            this->elapsedTicks = 2;
            changeState( this->agentState, 1317 );
        } // else
    } else if ( name == "goFindTreasure" ) { // owner gave game start command
        if ( arguments.size( ) > 0 && AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[0] ) == this->agent->getPetId( ) ) {
            changeState( this->agentState, 1317 ); // game started just for me
        } else {
            changeState( this->agentState, 1316 ); // communicate other players
        } // else

        setProperty( "following", "" );
        setProperty( "waiting", "" );
        setProperty( "collectTreasures", "" );

    } else if ( name == "followCustomPathDone" ) {
        this->elapsedTicks = 2;
        changeState( this->agentState, 1317 ); // back to the main game loop
    } else if ( name == "visibilityMap" ) {

        if ( arguments.size( ) == 0 ) {
            logger().debug("ScavengerHuntAgentModeHandler - Invalid visibility map signal, string 0 signed" );
            return;
        } // if
        logger().debug("ScavengerHuntAgentModeHandler - Parsing visibility map signal" );

        spatial::VisibilityMap* visibilityMap = getVisibilityMap( );
        if ( !visibilityMap ) {
            return;
        } // if

        std::vector<std::string> rows;
        boost::algorithm::split( rows, arguments[0], boost::algorithm::is_any_of(";") );

        unsigned int i;
        for ( i = 0; i < rows.size( ); ++i ) {
            std::stringstream parser( rows[i] );
            int row;
            parser >> row;
            int col = -1;
            std::vector<int> range;
            while ( !parser.eof( ) ) {
                parser >> col;
                if ( col == -1 ) {
                    continue;
                } // if
                range.push_back( col );
            } // while

            if ( range.size( ) == 0 || ( range.size( ) % 2 ) != 0 ) {
                logger().error("ScavengerHuntAgentModeHandler - The columns number of a visibility map signal must be pair and greater than 0. row: %d", row );
                return;
            } // if

            unsigned int j;
            int k;
            for ( j = 0; j < range.size( ); j += 2 ) {
                for ( k = range[j]; k <= range[j+1]; ++k ) {
                    const spatial::VisibilityMap::TilePtr& tile = visibilityMap->getTile( row, k );
                    if ( tile ) {
                        tile->setVisibility( true );
                    } else {
                        logger().error("ScavengerHuntAgentModeHandler - There was an attempt to access an invalid tile row: %d col: %d", row, k );
                    } // else
                } // for
            } // for
        } // for

    } else if ( name == "followMe" ) {
        if ( AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[0] ) == this->agent->getPetId( ) &&
                isTeamMember( arguments[1], this->myTeamCode ) ) {
            changeState( this->agentState, 1319 ); // follow owner
            setProperty( "following", "yes" );
            setProperty( "customObject", arguments[1] );
            setProperty( "collectTreasures", "" );
            setProperty( "waiting", "" );

            setFollowingPosition( );
        } // if
    } else if ( name == "followMeAndCollectTreasures" ) {
        if ( AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[0] ) == this->agent->getPetId( ) &&
                isTeamMember( arguments[1], this->myTeamCode ) ) {
            changeState( this->agentState, 1319 ); // follow owner
            setProperty( "following", "yes" );
            setProperty( "customObject", arguments[1] );
            setProperty( "collectTreasures", "yes" );
            setProperty( "waiting", "" );

            setFollowingPosition( );
        } // if

    } else if ( name == "comeHere" ) {
        if ( AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[0] ) == this->agent->getPetId( ) &&
                isTeamMember( arguments[1], this->myTeamCode ) ) {
            changeState( this->agentState, 1319 ); // follow owner
            setProperty( "customObject", arguments[1] );
            setProperty( "following", "" );
            setProperty( "waiting", "" );
            this->elapsedTicks = 0;
            setFollowingPosition( );
        } // if
    } else if ( name == "exploreArea" ) {
        if ( AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[1] ) == this->agent->getPetId( ) &&
                isTeamMember( arguments[2], this->myTeamCode ) ) {
            unsigned int targetArea = std::atoi( arguments[0].c_str( ) );
            if ( targetArea >= 0 && targetArea < NUMBER_OF_AREAS && !exploredAreas[targetArea] ) {
                this->exploringArea = targetArea;
                changeState( this->agentState, 1317 );
                setProperty( "waiting", "" );
                setProperty( "following", "" );
            } // if
        } // if
    } else if ( name == "wait" ) {
        if ( AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[0] ) == this->agent->getPetId( ) &&
                isTeamMember( arguments[1], this->myTeamCode ) ) {
            setProperty( "waiting", "yes" );
            setProperty( "following", "" );
            changeState( this->agentState, 1320 );
        } // if
    } else if ( name == "regroupTeam" ) {
        if ( ( (lowerCaseArguments[0] == "red" && this->myTeamCode == 0) ||
                (lowerCaseArguments[0] == "blue" && this->myTeamCode == 1) ) &&
                isTeamMember( arguments[1], this->myTeamCode ) ) {
            setProperty( "waiting", "" );
            setProperty( "following", "" );
            changeState( this->agentState, 1310 );
        } // if
    } else if ( name == "spy" ) {
        if ( AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[1] ) == this->agent->getPetId( ) &&
                isTeamMember( arguments[2], this->myTeamCode ) ) {
            setProperty( "waiting", "" );
            setProperty( "following", "" );
            std::string targetId = AtomSpaceUtil::getObjIdFromName( agent->getAtomSpace( ), arguments[0] );

            if ( std::find( this->teamPlayers[this->myTeamCode].begin( ), this->teamPlayers[this->myTeamCode].end( ), targetId ) == this->teamPlayers[this->myTeamCode].end( ) ) {
                // target is not from my team, so spy him/her
                //arguments[0]; // agent to spy
                logger().debug("ScavengerHuntAgentModeHandler - Spying[%s] Id[%s]", arguments[0].c_str( ), targetId.c_str( ) );
                setProperty( "spying", targetId );
                setProperty( "customObject", targetId );
                changeState( this->agentState, 1321 ); //spy an agent
            } // if
        } // if

    } else if ( name == "saveVisMap" ) {
        static int visMapCounter = 0;
        std::stringstream visMapName;
        visMapName << "visMap_scavengerHuntMode_";
        visMapName << this->agent->getPetId( );
        visMapName << "_";
        visMapName << visMapCounter++;
        visMapName << ".bin";
        spatial::VisibilityMap::saveToFile( visMapName.str( ), *getVisibilityMap( ) );
    } // else if
}

spatial::VisibilityMap* ScavengerHuntAgentModeHandler::getVisibilityMap( void )
{
    if ( this->visibilityMap != NULL ) {
        return this->visibilityMap;
    } // if

    Handle spaceMapHandle = this->agent->getAtomSpace().getSpaceServer().getLatestMapHandle();
    if (spaceMapHandle == Handle::UNDEFINED) {
        logger().debug("ScavengerHuntAgentModeHandler - There is no space map loaded at this moment" );
        return NULL;
    } // if

    unsigned int numberOfTilesPerSide =
        static_cast<unsigned int>(opencog::config().get_int( "MAP_XDIM" )) / 4;

    const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
    spatial::math::Vector3 minimumExtent( spaceMap.xMin( ), 0, spaceMap.yMin( ) );
    spatial::math::Vector3 maximumExtent( spaceMap.xMax( ), 0, spaceMap.yMax( ) );
    this->visibilityMap = new spatial::VisibilityMap( minimumExtent, maximumExtent, numberOfTilesPerSide );

    return this->visibilityMap;
}

void ScavengerHuntAgentModeHandler::update( void )
{
    BaseAgentModeHandler::update( );
    logger().debug("ScavengerHuntAgentModeHandler - Updating... Current state[%d] Previous state[%d] ElapsedTicks[%d]", this->agentState, this->previousAgentState, this->elapsedTicks );

    switch ( this->agentState ) {
    case 0: { // game started by the agent's owner
        changeState( this->agentState, 1 );
    }
    break;

    case 11: { // wait for answer
        if ( this->elapsedTicks > 7 ) {
            if ( myTeamCode == 0 ) {
                setProperty( "customObject", redBaseId );
            } else {
                setProperty( "customObject", blueBaseId );
            } // else

            changeState( this->agentState, 1301 );
        } // if
        ++this->elapsedTicks;
    }
    break;

    case 1306: {
        if ( isAgentHoldingTreasure( ) || !isAgentNearTreasure( ) ) {
            // agent grabbed the treasure or other agent grabbed it first
            this->elapsedTicks = 0;
            changeState( this->agentState, 1317 ); // go back to work
        } // if
    }
    break;

    case 1311: {
        if ( !isAgentHoldingTreasure( ) ) { // agent dropped the treasure
            this->elapsedTicks = 2;
            changeState( this->agentState, 1317 ); // go back to work
            setProperty( "treasure", "" );
        } // if
    }
    break;

    case 1305: {
        // the treasure exists yet?
        try {
            const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
            spaceMap.getEntity( getPropertyValue( "treasure" ) );
            // yeah! keep going after it
        } catch ( opencog::NotFoundException& ex ) {
            // no... so, forget about it
            setProperty( "treasure", "" );
            changeState( this->agentState, 1317 );
        } // catch
    }
    break;
    

    case 1317: {

        if ( isExecutingSchema( ) ) {
            this->elapsedTicks = 0;
        } else if ( isAgentHoldingTreasure( ) ) {
            if ( isAgentNearBase( ) ) {
                logger().debug("ScavengerHuntAgentModeHandler - Agent is holding a treasure and is near it's base" );
                changeState( this->agentState, 1311 ); // drop treasure
            } else {
                logger().debug("ScavengerHuntAgentModeHandler - Agent is holding a treasure but is not near it's base" );
                if ( myTeamCode == 0 ) {
                    setProperty( "customObject", redBaseId );
                } else {
                    setProperty( "customObject", blueBaseId );
                } // else
                changeState( this->agentState, 1301 ); // go back to its base

            } // else
        } else if ( isAgentNearTreasure( ) ) {
            changeState( this->agentState, 1306 ); // grab treasure

        } else if ( getPropertyValue( "following" ) == "yes" ) {
            changeState( this->agentState, 1319 ); // following something
            setFollowingPosition( );

        } else if ( getPropertyValue( "spying" ).length( ) > 0 ) {
            changeState( this->agentState, 1321 ); // spying someone

        } else if ( treasureWasFound( ) ) {
            changeState( this->agentState, 1305 ); // go to after it

        } else if ( isThereAreaToInspect( ) ) {
            if ( this->exploringArea != NUMBER_OF_AREAS ) { // i was exploring an area?

                if ( imInsideTargetArea( ) ) {
                    if ( isThereObjectToInspectInsideTargetArea( ) ) {
                        changeState( this->agentState, 1307 ); // explore object

                    } else if ( isThereHiddenTilesInsideTargetArea( ) ) {
                        changeState( this->agentState, 1309 ); // look at hidden tile direction
                        this->elapsedTicks = 0;

                    } else {
                        this->elapsedTicks = 2;
                        changeState( this->agentState, 1314 ); // mark area as explored

                    } // else

                } else if ( isThereVisibleTilesInsideTargetArea( ) ) { // can reach target area

                    changeState( this->agentState, 1313 ); // go to the visible tile inside the target area

                } else { // can't reach target area

                    std::stringstream parser;
                    spatial::math::Vector3 areaCenter =
                        getVisibilityMap( )->getAreaCenter( this->exploringArea, NUMBER_OF_AREAS );
                    parser << areaCenter.x << " " << areaCenter.z;
                    setProperty( "customPosition", parser.str( ) );

                    changeState( this->agentState, 1315 ); // look at area direction
                    this->elapsedTicks = 0;
                } // else

            } else { // i was'n exploring an area
                changeState( this->agentState, 1302 ); // choose an area to explore

            } // else
        } else if ( !isAgentNearBase( ) ) { // all areas were cleared
            if ( myTeamCode == 0 ) {
                setProperty( "customObject", redBaseId );

            } else {
                setProperty( "customObject", blueBaseId );

            } // else
            changeState( this->agentState, 1310 ); // go back to base to compute points

        } else {
            // game over
            resetGame( );
            this->agent->setMode( PLAYING );
        } // else

    }
    break;

    case 1319: { // follow owner
        updateStartPositionInCustomPath( );

        if ( getPropertyValue( "collectTreasures" ) == "yes" && treasureWasFound( ) ) {
            changeState( this->agentState, 1305 ); // go to after it
            logger().debug("ScavengerHuntAgentModeHandler - I'm following, collecting treasures and a treasure was found" );

        } else if ( getPropertyValue( "spying" ).length( ) > 0 ) {
            logger().debug("ScavengerHuntAgentModeHandler - I'm spying[%s]",  getPropertyValue( "spying" ).c_str( ) );
            if ( isAgentNextTo( getPropertyValue( "spying" ) ) ) {
                logger().debug("ScavengerHuntAgentModeHandler - I'm spying[%s] and next to target, so keep spying",  getPropertyValue( "spying" ).c_str( ) );
                changeState( this->agentState, 1321 ); // keepSpying
            } // if

        } else if ( getPropertyValue( "following" ).length( ) == 0 ) {
            logger().debug("ScavengerHuntAgentModeHandler - I'm following nothing" );
            if ( isAgentNextTo( getPropertyValue( "customObject" ) ) && this->elapsedTicks > 10 ) {
                logger().debug("ScavengerHuntAgentModeHandler - I'm following nothing and next to the target[%s]", getPropertyValue( "customObject" ).c_str( ) );
                changeState( this->agentState, 1317 ); // go
            } // if
            ++this->elapsedTicks;

        } else if ( isAgentNextTo( getPropertyValue( "customObject" ) ) ) {
            changeState( this->agentState, 1320 ); // stay

        } // else if
    }
    break;
    
    case 1313:
    case 1307: {
        updateStartPositionInCustomPath( );
    } break;
        

    case 1320: { // stay next target after following a target
        if ( getPropertyValue( "waiting" ) == "yes" ) {
            // ignore
        } else if ( !isAgentNextTo( getPropertyValue( "customObject" ) ) ) {
            changeState( this->agentState, 1319 ); // go after target
            setFollowingPosition( );
        } // if
    }
    break;


    case 1321: { // spying an agent
        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );
        try {
            const spatial::EntityPtr& target = spaceMap.getEntity( getPropertyValue( "spying" ) );
            if ( agentEntity->distanceTo( *target ) > 4000 ) {
                setFollowingPosition( );
                changeState( this->agentState, 1319 ); // go near target
            } // if
        } catch ( opencog::NotFoundException& ex ) { // look at target
            // search for the target
            this->elapsedTicks = 0;
            std::stringstream targetPosition;
            spatial::math::Vector3 position = agentEntity->getPosition( );


            spatial::math::Quaternion rotation( spatial::math::Vector3::Z_UNIT, M_PI / 2.0 );
            position += rotation.rotate( agentEntity->getDirection( ) ) * 1000;

            logger().debug("ScavengerHuntAgentModeHandler - Trying to look at a 90 degrees right. yaw[%f] dirBefore[%s] dirAfter[%s] agentPosition[%s] finalPosition[%s]", agentEntity->getOrientation( ).getRoll( ), agentEntity->getDirection( ).toString( ).c_str( ), agentEntity->getDirection( ) .toString( ).c_str( ), agentEntity->getPosition( ).toString( ).c_str( ), position.toString( ).c_str( ) );

            targetPosition << position.x << " " << position.y;
            setProperty( "customPosition", targetPosition.str( ) );
            changeState( this->agentState, 1322 );
        } // catch
    }
    break;
    case 1322: {
        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );
        try {
            spaceMap.getEntity( getPropertyValue( "spying" ) );
            changeState( this->agentState, 1321 ); // target found, so keep spying
        } catch ( opencog::NotFoundException& ex ) {
            if ( elapsedTicks > 3 ) {
                this->elapsedTicks = 0;

                std::stringstream targetPosition;
                spatial::math::Vector3 position = agentEntity->getPosition( );

                spatial::math::Quaternion rotation( spatial::math::Vector3::Z_UNIT, M_PI / 2.0 );
                position += rotation.rotate( agentEntity->getDirection( ) ) * 1000;

                logger().debug("ScavengerHuntAgentModeHandler - Trying to look at a 90 degrees right. yaw[%f] dirBefore[%s] dirAfter[%s] agentPosition[%s] finalPosition[%s]", agentEntity->getOrientation( ).getRoll( ), agentEntity->getDirection( ).toString( ).c_str( ), agentEntity->getDirection( ).toString( ).c_str( ), agentEntity->getPosition( ).toString( ).c_str( ), position.toString( ).c_str( ) );

                targetPosition << position.x << " " << position.y;
                setProperty( "customPosition", targetPosition.str( ) );
            } // if
            ++this->elapsedTicks;
        } // catch
    }
    break;
    case 1323: {
        try {
            const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();

            const spatial::EntityPtr& target = spaceMap.getEntity( getPropertyValue( "spying" ) );
            const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );

            if ( agentEntity->distanceTo( *target ) > 4000 ) {
                setFollowingPosition( );
                changeState( this->agentState, 1319 ); // go near target
            } // if

            // look at target

        } catch ( opencog::NotFoundException& ex ) {
            changeState( this->agentState, 1322 ); // search target
            this->elapsedTicks = 0;
        } // catch
    }
    break;

    case 1303: { // wait for answers during a short period
        logger().debug("ScavengerHuntAgentModeHandler - Receiving group_commands. ElapsedTicks[%d]", this->elapsedTicks );
        if ( this->elapsedTicks > 3 ) {
            this->elapsedTicks = 1;
            changeState( this->agentState, 1317 );
        } // if
        ++this->elapsedTicks;
    }
    break;

    case 1309: { // look at a hidden tile (from a specific area) direction
        if ( this->elapsedTicks > 2 ) { // wait to agent look to tile direction
            std::stringstream tilePosition( getPropertyValue( "customPosition" ) );
            spatial::math::Vector3 position;
            tilePosition >> position.x >> position.z;
            if ( getVisibilityMap( )->getTile( position )->isVisible( ) ) {
                this->elapsedTicks = 0;
                changeState( this->agentState, 1317 ); // restart area exploration
            } else {
                try {
                    const spatial::VisibilityMap::TilePtr& tile = getVisibilityMap( )->getNearestVisibleTile( getVisibilityMap( )->getTile( position )->getCenter( ), this->exploringArea, NUMBER_OF_AREAS );
                    setProperty( "customPath", computeWalkAroundWaypoints( tile->getCenter( ) ) );
                    changeState( this->agentState, 1313 ); // go nearest to target visible area tile
                } catch ( opencog::NotFoundException& ex ) {
                    std::stringstream exploredArea;
                    exploredArea << this->exploringArea;
                    setProperty( "customObject", exploredArea.str( ) );
                    changeState( this->agentState, 1315 );
                } // catch
            } // else
        } // if
        ++this->elapsedTicks;
    }
    break;

    case 1315: {
        if ( this->elapsedTicks > 3 ) {
            try {
                spatial::math::Vector3 areaCenter = getVisibilityMap( )->getAreaCenter( this->exploringArea, NUMBER_OF_AREAS );
                const spatial::VisibilityMap::TilePtr& tile = getVisibilityMap( )->getNearestVisibleTileToPosition( areaCenter );
                setProperty( "customPath", computeWalkAroundWaypoints( tile->getCenter( ) ) );
                changeState( this->agentState, 1313 ); // go to the nearest visible tile to target area
            } catch ( opencog::NotFoundException& ex ) {
                logger().debug(
                             "ScavengerHuntAgentModeHandler - There is no visible tile inside the current target area[%s]", ex.getMessage( ) );
                static int fileCounter = 0;
                std::stringstream fileName;
                fileName << "visibilityMap_" << fileCounter << ".bin";
                ++fileCounter;
                spatial::VisibilityMap::saveToFile( fileName.str( ), *getVisibilityMap( ) );
                changeState( this->agentState, 1317 ); // try to find another solution
            } // catch
        } // if
        ++this->elapsedTicks;
    }
    break;

    default: // do nothing
        break;
    } // switch

}

bool ScavengerHuntAgentModeHandler::isThereObjectToInspectInsideTargetArea( void )
{
    const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
    std::vector<std::string> entities;
    spaceMap.findAllEntities(back_inserter(entities));

    unsigned int i;
    for ( i = 0; i < entities.size( ); ++i ) {
        if ( entities[i] == this->redBaseId || entities[i] == this->blueBaseId ) {
            // skip bases
            continue;
        } // if


        HandleSeq objHandle;
        this->agent->getAtomSpace().getHandleSet(back_inserter(objHandle), OBJECT_NODE, entities[i] , true);

        if (objHandle.size() != 1) {
            logger().error("ScavengerHuntAgentModeHandler::isThereObjectToInspectInsideTargetArea - %d handles were found to entity[%s]", objHandle.size( ), entities[i].c_str( ) );
            continue;
        } // if

        if ( AtomSpaceUtil::isPredicateTrue( this->agent->getAtomSpace(), "is_edible", objHandle[0] ) ||
                AtomSpaceUtil::isPredicateTrue( this->agent->getAtomSpace(), "is_drinkable", objHandle[0] ) ) {
            logger().debug("ScavengerHuntAgentModeHandler::isThereObjectToInspectInsideTargetArea - Entity[%s] is edible or drinkable. ignoring...", entities[i].c_str( ) );
            continue;
        } // if

        try {
            const spatial::EntityPtr& obstacle = spaceMap.getEntity( entities[i] );
            if ( !obstacle->getBooleanProperty( spatial::Entity::OBSTACLE ) ) {
                logger().debug("ScavengerHuntAgentModeHandler::isThereObjectToInspectInsideTargetArea - Entity[%s] is not an obstacle. ignoring...", entities[i].c_str( ) );
                continue;
            } // if

            if ( !getVisibilityMap( )->isInsideArea( *obstacle, this->exploringArea, NUMBER_OF_AREAS ) ) {
                logger().debug("ScavengerHuntAgentModeHandler::isThereObjectToInspectInsideTargetArea - Entity[%s] is not inside area[%d]. ignoring...", entities[i].c_str( ), this->exploringArea );
                continue;
            } // if
        } catch ( opencog::NotFoundException& ex ) {
            logger().error("ScavengerHuntAgentModeHandler::isThereObjectToInspectInsideTargetArea - Entity[%s] was not found inside LocalSpaceMap2D",  entities[i].c_str( ) );
            continue;
        } // if

        std::vector<std::string>::iterator it =
            std::find( this->exploredObjects.begin( ), this->exploredObjects.end( ), entities[i] );

        if ( it == this->exploredObjects.end( ) ) {
            logger().debug("ScavengerHuntAgentModeHandler::isThereObjectToInspectInsideTargetArea - Found entity[%s] inside area[%d]", entities[i].c_str( ), this->exploringArea );
            setProperty( "nextObjectToExplore", entities[i] );
            setProperty( "customPath", computeWalkAroundWaypoints( entities[i] ) );
            setProperty( "walkSpeed", "1.5" );

            // if this entity belongs to a superentity, add all the entities that compose that superentity
            // into the exlored objects list
            if ( spaceMap.belongsToSuperEntity( entities[i] ) ) {
                std::vector<long> entitiesIds =
                    spaceMap.getSuperEntityWhichContains( entities[i] )->getSubEntitiesIds( );
                unsigned int j;
                for ( j = 0; j < entitiesIds.size( ); ++j ) {
                    const spatial::EntityPtr& entity = spaceMap.getEntity( entitiesIds[j] );
                    std::vector<std::string>::iterator itEntity =
                        std::find( this->exploredObjects.begin( ), this->exploredObjects.end( ), entity->getName( ) );
                    if ( itEntity == this->exploredObjects.end( ) ) {
                        this->exploredObjects.push_back( entity->getName( ) );
                    } // if
                } // for
            } else {
                this->exploredObjects.push_back( entities[i] );
            } // else

            return true;
        } // if
    } // for

    return false;
}

bool ScavengerHuntAgentModeHandler::isThereAreaToInspect( void )
{
    return getVisibilityMap( )->hasHiddenTile( );
}

bool ScavengerHuntAgentModeHandler::isThereVisibleTilesInsideTargetArea( void )
{
    try {
        const spatial::VisibilityMap::TilePtr& tile =
            getVisibilityMap( )->getNextVisibleTile( this->exploringArea, NUMBER_OF_AREAS );

        setProperty( "customPath", computeWalkAroundWaypoints( tile->getCenter( ) ) );
        return true;
    } catch ( ... ) {
        return false;
    } // catch
}

bool ScavengerHuntAgentModeHandler::isThereHiddenTilesInsideTargetArea( void )
{
    try {
        const spatial::VisibilityMap::TilePtr& tile =
            getVisibilityMap( )->getNextHiddenTile( this->exploringArea, NUMBER_OF_AREAS );
        std::stringstream parser;
        parser << tile->getCenter( ).x << " " << tile->getCenter( ).z;
        setProperty( "customPosition", parser.str( ) );
        return true;
    } catch ( ... ) {
        return false;
    } // catch
}


bool ScavengerHuntAgentModeHandler::imInsideTargetArea( void )
{
    if ( this->exploringArea == NUMBER_OF_AREAS ) {
        logger().debug("ScavengerHuntAgentModeHandler::imInsideTargetArea - Exploring area was not definded" );
        return false;
    } // if

    try {
        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );
        bool imInside = getVisibilityMap( )->isInsideArea( *agentEntity, this->exploringArea, NUMBER_OF_AREAS );
        logger().debug("ScavengerHuntAgentModeHandler::imInsideTargetArea - Exploring area[%d] Current Agent position[%s] Inside?[%s]", this->exploringArea, agentEntity->getPosition( ).toString( ).c_str( ), (imInside ? "yes" : "no") );
        return imInside;
    } catch ( opencog::NotFoundException& ex ) {

        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );
        logger().error("ScavengerHuntAgentModeHandler::imInsideTargetArea - AgentPosition[%s] AreaNumber[%d] %s", agentEntity->getPosition( ).toString( ).c_str( ), this->exploringArea, ex.getMessage( ) );
        return false;
    } // catch
}

bool ScavengerHuntAgentModeHandler::isExecutingSchema( void )
{

    if ( this->needToCollectProcedureId ) {
        this->runningProcedureId = this->agent->getRuleEngine( )->getExecutingSchemaID( );
    } // if

    if ( this->runningProcedureId == 0 ) {
        logger().debug("ScavengerHuntAgentModeHandler::isExecutingGoto - Running procedure id was not defined" );
        return false;
    } // if

    if ( this->agent->getRuleEngine( )->isSchemaExecFinished( this->runningProcedureId ) ) {
        logger().debug("ScavengerHuntAgentModeHandler::isExecutingSchema - Schema was finished[%d]", this->runningProcedureId );
        this->runningProcedureId = 0;
        this->needToCollectProcedureId = false;
        return false;
    } // if
    logger().debug("ScavengerHuntAgentModeHandler::isExecutionSchema - Schema was not finished[%d]", this->runningProcedureId );
    return true;
}

bool ScavengerHuntAgentModeHandler::treasureWasFound( void )
{
    try {

        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();

        std::list<std::string> entities;
        spaceMap.getAllObjects( back_inserter( entities ) );

        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );

        const spatial::EntityPtr& redBaseEntity = spaceMap.getEntity( redBaseId );
        const spatial::EntityPtr& blueBaseEntity = spaceMap.getEntity( blueBaseId );

        std::vector<std::string> treasures;
        std::vector<double> distanceToAgent;

        std::list<std::string>::iterator it;
        for ( it = entities.begin( ); it != entities.end( ); ++it ) {

            const spatial::EntityPtr& entity = spaceMap.getEntity( *it );
            Handle objectHandle = this->agent->getAtomSpace( ).getHandle( ACCESSORY_NODE, entity->getName( ) );
            if ( objectHandle == Handle::UNDEFINED ) {
                logger().debug("ScavengerHuntAgentModeHandler - %s is not an accessory", (*it).c_str());
                continue;
            } // if

            if ( !AtomSpaceUtil::isPredicateTrue( this->agent->getAtomSpace( ), "is_pickupable", objectHandle ) ) {
                logger().debug("ScavengerHuntAgentModeHandler - %s is not pickupable", (*it).c_str());
                continue;
            } // if

            if ( redBaseEntity->distanceTo( *entity ) <= 800 || blueBaseEntity->distanceTo( *entity ) <= 800 ) {
                logger().debug("ScavengerHuntAgentModeHandler - %s is near a team base", (*it).c_str());
                continue;
            } // if

            if ( AtomSpaceUtil::isObjectBeingHolded( this->agent->getAtomSpace( ), *it ) ) {
                logger().debug("ScavengerHuntAgentModeHandler - Some agent is holding object[%s]", (*it).c_str());
                continue;
            } // if

            logger().debug("ScavengerHuntAgentModeHandler - %s is a treasure", (*it).c_str());
            treasures.push_back( *it );
            distanceToAgent.push_back( agentEntity->distanceTo( *entity ) );

        } // for


        double minorDistance = std::numeric_limits<double>::max( );
        std::string chosenTreasure;
        unsigned int i;
        // select the nearest treasure to the agent
        for ( i = 0; i < treasures.size( ); ++i ) {
            if ( distanceToAgent[i] < minorDistance ) {
                chosenTreasure = treasures[i];
                minorDistance = distanceToAgent[i];
            } // if
        } // if

        if ( chosenTreasure.length( ) > 0 ) {
            logger().debug("ScavengerHuntAgentModeHandler - %s was selected as a treasure", chosenTreasure.c_str());
            setProperty( "customObject", chosenTreasure );
            setProperty( "treasure", chosenTreasure );
            return true;
        } // if

        return false;

    } catch ( opencog::NotFoundException& ex ) {
        logger().error("ScavengerHuntAgentModeHandler - %s", ex.getMessage( ) );
        return false;
    } // else

}

bool ScavengerHuntAgentModeHandler::isAgentNearTreasure( void )
{
    if ( getPropertyValue( "treasure" ).length( ) == 0 ) {
        return false;
    } // if

    Handle agentHandle = AtomSpaceUtil::getAgentHandle( this->agent->getAtomSpace( ), this->agent->getPetId( ) );
    Handle treasureHandle = this->agent->getAtomSpace( ).getHandle( ACCESSORY_NODE, getPropertyValue( "treasure" ) );

    if ( agentHandle == Handle::UNDEFINED || treasureHandle == Handle::UNDEFINED ) {
        logger().error("ScavengerHuntAgentModeHandler::isAgentNearTreasure - Treasure handle was not found" );
        return false;
    } // if

    const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();

    try {
        const spatial::EntityPtr& treasureEntity = spaceMap.getEntity( getPropertyValue( "treasure" ) );
        const spatial::EntityPtr& redBaseEntity = spaceMap.getEntity( redBaseId );
        const spatial::EntityPtr& blueBaseEntity = spaceMap.getEntity( blueBaseId );

        if ( redBaseEntity->distanceTo( *treasureEntity ) <= 800 || blueBaseEntity->distanceTo( *treasureEntity ) <= 800 ) {
            logger().debug("ScavengerHuntAgentModeHandler::isAgentNearTreasure - Treasure is near a team base" );
            return false;
        } // if

        bool isNearTreasure = AtomSpaceUtil::isPredicateTrue( this->agent->getAtomSpace( ), "near", agentHandle, treasureHandle );
        logger().debug("ScavengerHuntAgentModeHandler::isAgentNearTreasure - Agent %s near treasure", (isNearTreasure ? "is" : "is not" ) );
        return isNearTreasure;
    } catch ( opencog::NotFoundException& ex ) {
        setProperty( "treasure", "" );
        logger().debug("SavengerHuntAgentModeHandler::isAgentNearTreasure - %s", ex.getMessage( ) );
        return false;
    } // catch
}

bool ScavengerHuntAgentModeHandler::isAgentNearOwner( void )
{
    Handle agentHandle = AtomSpaceUtil::getAgentHandle( this->agent->getAtomSpace( ), this->agent->getPetId( ) );
    Handle ownerHandle = AtomSpaceUtil::getAgentHandle( this->agent->getAtomSpace( ), this->agent->getOwnerId( ) );

    return AtomSpaceUtil::isPredicateTrue( this->agent->getAtomSpace( ), "near", agentHandle, ownerHandle );
}


bool ScavengerHuntAgentModeHandler::isAgentHoldingTreasure( void )
{
    Handle agentHandle = AtomSpaceUtil::getAgentHandle( this->agent->getAtomSpace( ), this->agent->getPetId( ) );
    return AtomSpaceUtil::isPredicateTrue( this->agent->getAtomSpace(), "isHoldingSomething", agentHandle );
}

bool ScavengerHuntAgentModeHandler::isAgentNearBase( void )
{
    try {
        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );

        if ( this->myTeamCode == 0 ) {
            const spatial::EntityPtr& redBaseEntity = spaceMap.getEntity( redBaseId );
            return ( redBaseEntity->distanceTo( *agentEntity ) <= 800 );
        } else {
            const spatial::EntityPtr& blueBaseEntity = spaceMap.getEntity( blueBaseId );
            return ( blueBaseEntity->distanceTo( *agentEntity ) <= 800 );
        } // else
    } catch ( opencog::NotFoundException& ex ) {
        logger().error("ScavengerHuntAgentModeHandler::isAgentNearBase - %s", ex.getMessage( ) );
        return false;
    } // catch
}

std::string ScavengerHuntAgentModeHandler::computeWalkAroundWaypoints( const std::string& entityId )
{

    std::stringstream output;
    try {
        const SpaceServer::SpaceMap& sm = this->agent->getAtomSpace().getSpaceServer( ).getLatestMap( );

        const spatial::EntityPtr& occluderEntity = sm.getEntity( entityId );
        const spatial::EntityPtr& agentEntity = sm.getEntity( this->agent->getPetId( ) );

        output << agentEntity->getPosition( ).x << " " << agentEntity->getPosition( ).y << ";";

        std::vector<spatial::Point> wayPoints;

        spatial::Point startCorner;
        unsigned int startIndex = 0;
        float minDistance = std::numeric_limits<float>::max( );

        //std::vector<spatial::math::LineSegment> edges;
        std::list<spatial::math::Vector3> corners;
        unsigned int i;
        if ( sm.belongsToSuperEntity( entityId ) ) {
            corners = sm.getSuperEntityWhichContains( entityId )->getCorners( );
            //const std::list<spatial::math::LineSegment>& superEntityEdges =
            //    sm.getSuperEntityWhichContains( entityId )->getEdges( );
            /*
            std::list<spatial::math::LineSegment>::const_iterator it;
            for( it = superEntityEdges.begin( ); it != superEntityEdges.end( ); ++it ) {
                edges.push_back( *it );
            } // for
            */
        } else {
            const std::vector<spatial::math::LineSegment>& entityEdges =
                occluderEntity->getBoundingBox( ).getAllEdges( );
            for ( i = 0; i < 4; ++i ) {
                //edges.push_back( entityEdges[i] );
                corners.push_back( entityEdges[i].pointA );
            } // for
        } // else

        std::list<spatial::math::Vector3>::iterator it;
        i = 0;
        for ( it = corners.begin( ); it != corners.end( ); ++it ) {

            //for( i = 0; i < corners.size( ); ++i ) {

            //spatial::Point corner( edges[i].pointA.x, edges[i].pointA.y );
            spatial::Point corner( it->x, it->y );
            //spatial::math::Vector3 direction = edges[i].pointA - occluderEntity->getPosition( );

            corner = sm.getNearestFreePoint( corner );//targetPosition = sm.findFree( corner, spatial::Point( direction.x, direction.y ) );

            wayPoints.push_back( corner );
            float candidateDistance = ( agentEntity->getPosition( ) - spatial::math::Vector3( corner.first, corner.second ) ).length( );
            if ( candidateDistance < minDistance ) {
                startIndex = i;
                minDistance = candidateDistance;
            } // if
            ++i;
        } // for

        unsigned int cursor = startIndex;
        for ( i = 0; i < wayPoints.size( ); ++i ) {
            output << wayPoints[cursor].first << " " <<  wayPoints[cursor].second;
            if ( i + 1 < wayPoints.size( ) ) {
                output << ";";
            } // if
            cursor = (cursor + 1) % wayPoints.size( );
        } // if

    } catch ( opencog::NotFoundException& ex ) {
        logger().error(
                     "ScavengerHuntAgentModeHandler - There is no object %s inside map.", entityId.c_str( ) );
    } // catch
    return output.str( );
}

std::string ScavengerHuntAgentModeHandler::computeWalkAroundWaypoints( const spatial::math::Vector3& targetPosition )
{
    std::stringstream output;
    try {
        const SpaceServer::SpaceMap& sm = this->agent->getAtomSpace().getSpaceServer( ).getLatestMap( );

        const spatial::EntityPtr& agentEntity = sm.getEntity( this->agent->getPetId( ) );

        output << agentEntity->getPosition( ).x << " " << agentEntity->getPosition( ).y << ";";
        output << targetPosition.x << " " << targetPosition.z;

    } catch ( opencog::NotFoundException& ex ) {
        logger().error("ScavengerHuntAgentModeHandler - Agent was not found inside map" );
    } // catch
    return output.str( );
}

void ScavengerHuntAgentModeHandler::addPlayer( const std::string& playerName, unsigned int teamCode, float randomNumber )
{
    logger().debug("ScavengerHuntAgentModeHandler - adding player[%s] teamCode[%d] randomNumber[%f]", playerName.c_str( ), teamCode, randomNumber );
    std::map<std::string, PlayerMetaData>::iterator it = this->players.find( playerName );
    if ( it == this->players.end( ) ) {
        logger().debug("ScavengerHuntAgentModeHandler - player needs to be added to a team" );

        this->players.insert( std::map<std::string, PlayerMetaData>::value_type(
                                  playerName, PlayerMetaData( playerName, teamCode, randomNumber )
                              ) );

        std::set<PlayerMetaData> redTeam;
        std::set<PlayerMetaData> blueTeam;

        for ( it = this->players.begin( ); it != this->players.end( ); ++it ) {
            if ( it->second.suggestedCode == 0 ) {
                redTeam.insert( it->second );
            } else {
                blueTeam.insert( it->second );
            } // else
        } // for

        logger().debug("ScavengerHuntAgentModeHandler - redTeam has[%d], blueTeam has[%d] players", redTeam.size( ), blueTeam.size( ) );

        int teamDiff = static_cast<int>(redTeam.size( )) - static_cast<int>(blueTeam.size( ) );

        while ( std::abs(teamDiff) > 1 ) {
            logger().debug("ScavengerHuntAgentModeHandler - teamDifference[%d]", teamDiff );
            if ( teamDiff < 0 ) {
                // blue to red
                logger().debug("ScavengerHuntAgentModeHandler - passing %s from blue to red", blueTeam.rbegin( )->toString( ).c_str( ) );
                redTeam.insert( *blueTeam.rbegin( ) );
                blueTeam.erase( *blueTeam.rbegin( ) );
            } else {
                // red to blue
                logger().debug("ScavengerHuntAgentModeHandler - passing %s from red to blue", redTeam.rbegin( )->toString( ).c_str( ) );
                blueTeam.insert( *redTeam.rbegin( ) );
                redTeam.erase( *redTeam.rbegin( ) );
            } // else
            teamDiff = static_cast<int>(redTeam.size( )) - static_cast<int>(blueTeam.size( ) );
        } // while

        logger().debug("ScavengerHuntAgentModeHandler - redTeam has[%d], blueTeam has[%d] players", redTeam.size( ), blueTeam.size( ) );

        this->teamPlayers[0].clear( );
        this->teamPlayers[1].clear( );

        std::set<PlayerMetaData>::iterator it;
        for ( it = redTeam.begin( ); it != redTeam.end( ); ++it ) {
            logger().debug("ScavengerHuntAgentModeHandler - Verifying red team %s ==? %s", it->name.c_str( ), this->agent->getPetId( ).c_str( ) );
            if ( it->name == this->agent->getPetId( ) ) {
                this->myTeamCode = 0;
            } // if
            this->teamPlayers[0].push_back( it->name );
        } // for

        for ( it = blueTeam.begin( ); it != blueTeam.end( ); ++it ) {
            logger().debug("ScavengerHuntAgentModeHandler - Verifying blue team %s ==? %s", it->name.c_str( ), this->agent->getPetId( ).c_str( ) );
            if ( it->name == this->agent->getPetId( ) ) {
                this->myTeamCode = 1;
            } // if
            this->teamPlayers[1].push_back( it->name );
        } // for

        // reset exploring areas of my team
        this->exploringAreas.resize(NUMBER_OF_AREAS);
        //std::fill( this->exploringAreas.begin(), this->exploringAreas.end(), std::vector<std::string>() );

        this->exploredAreas.resize(NUMBER_OF_AREAS);
        //std::fill( this->exploredAreas.begin(), this->exploredAreas.end(), false );

        logger().debug("ScavengerHuntAgentModeHandler - I'm from %s team. New player %s belongs to %s team", (this->myTeamCode == 0 ? "red" : "blue"), playerName.c_str( ), ( std::find( this->teamPlayers[0].begin( ), this->teamPlayers[0].end( ), playerName ) != this->teamPlayers[0].end( ) ? "red" : "blue" )  );
    } // if
}

void ScavengerHuntAgentModeHandler::selectArea( const std::string& playerName, unsigned int areaNumber, float randomNumber )
{

    logger().debug("ScavengerHuntAgentModeHandler - selecting an area to explore. player[%s] teamCode[%d] randomNumber[%f] : agent which will handle this command[%s]", playerName.c_str( ), areaNumber, randomNumber, this->agent->getPetId( ).c_str( ) );

    std::map<std::string, PlayerMetaData>::iterator it = this->players.find( playerName );
    if ( it == this->players.end( ) ) {
        logger().error("ScavengerHuntAgentModeHandler - An invalid player send me a select area message %s %d %f", playerName.c_str( ), areaNumber, randomNumber );
        return;
    } // if

    std::vector<std::string>& myTeam = this->teamPlayers[this->myTeamCode];
    std::vector<std::string>::iterator it2 = std::find( myTeam.begin( ), myTeam.end( ), playerName );

    if ( it2 == myTeam.end( ) ) {
        logger().debug("ScavengerHuntAgentModeHandler - player[%s] doesn't belong to my team", playerName.c_str( ) );
        return; // handle only my team requests
    } // if

    it->second.randomNumber = randomNumber;
    it->second.suggestedCode = areaNumber;

    if ( this->exploredAreas[areaNumber] ) {
        logger().debug("ScavengerHuntAgentModeHandler - chosen area[%d] was already explored", areaNumber );
        return; // ignore, maybe it will be necessary notify (again) the agent that that area was already explored
    } // if

    // cleanup old registry about the given player
    unsigned int i;
    for ( i = 0; i < this->exploringAreas.size( ); ++i ) {
        std::vector<std::string>::iterator itPlayer =
            std::find( this->exploringAreas[i].begin( ), this->exploringAreas[i].end( ), playerName );
        if ( itPlayer != this->exploringAreas[i].end( ) ) {
            this->exploringAreas[i].erase( itPlayer );
        } // if
    } // for

    std::string looser = "";
    logger().debug("ScavengerHuntAgentModeHandler - exploring areas size[%d] for area number[%d]", this->exploringAreas[areaNumber].size( ), areaNumber );
    if ( this->exploringAreas[areaNumber].size( ) == 0 ) {
        logger().debug("ScavengerHuntAgentModeHandler - there is no agent exploring area[%d], so player[%s] will be allocated to it", areaNumber, playerName.c_str( ) );
        this->exploringAreas[areaNumber].push_back( playerName );

        if ( playerName == this->agent->getPetId( ) ) {
            this->exploringArea = areaNumber;
        } // if

    } else if ( this->exploringAreas[areaNumber].size( ) == 1 ) {
        logger().debug("ScavengerHuntAgentModeHandler - there is just one agent[%s] exploring area[%d]", this->exploringAreas[areaNumber][0].c_str( ), areaNumber );

        std::map<std::string, PlayerMetaData>::iterator it3 = this->players.find( this->exploringAreas[areaNumber][0] );
        if ( it3 == this->players.end( ) ) {
            logger().error("ScavengerHuntAgentModeHandler - An invalid player was allocated to an area of my team: %s %d", this->exploringAreas[areaNumber][0].c_str( ), areaNumber );
            return;
        } // if

        if ( randomNumber > it3->second.randomNumber ) {
            looser = it3->second.name;
            this->exploringAreas[areaNumber][0] = playerName;

            if ( playerName == this->agent->getPetId( ) ) {
                this->exploringArea = areaNumber;
            } // if
            logger().debug("ScavengerHuntAgentModeHandler - player[%s] win the agent that was exploring the area[%d], so he will assume this area. looser[%s]", playerName.c_str( ), areaNumber, looser.c_str( ) );
        } else {
            looser = playerName;
            logger().debug("ScavengerHuntAgentModeHandler - player[%s] loose the agent that was exploring the area[%d], so he will try to find another area to explore", playerName.c_str( ), areaNumber );
        } // else

        if ( looser == this->agent->getPetId( ) ) {
            logger().debug("ScavengerHuntAgentModeHandler - the agent[%s] is the looser, so reset the current exploring area", looser.c_str( ) );
            this->exploringArea = NUMBER_OF_AREAS;
        } // if
    } // if

    if ( looser.length( ) > 0 ) {

        // cleanup old registry about the looser
        unsigned int i;
        for ( i = 0; i < this->exploringAreas.size( ); ++i ) {
            std::vector<std::string>::iterator itPlayer =
                std::find( this->exploringAreas[i].begin( ), this->exploringAreas[i].end( ), looser );
            if ( itPlayer != this->exploringAreas[i].end( ) ) {
                this->exploringAreas[i].erase( itPlayer );
            } // if
        } // for

        logger().debug("ScavengerHuntAgentModeHandler - There are more than one agent expliring the same area, so lets reallocate the looser player" );
        // there are more than one player exploring the same area.
        // so, lets reallocate the looser player
        unsigned int numberOfExplorers = std::numeric_limits<unsigned int>::max( );
        unsigned int lessCrowdedAreaIndex = NUMBER_OF_AREAS;

        for ( i = 0; i < this->exploringAreas.size( ); ++i ) {
            if ( !this->exploredAreas[i] && this->exploringAreas[i].size( ) < numberOfExplorers ) {
                lessCrowdedAreaIndex = i;
                numberOfExplorers = this->exploringAreas[i].size( );
            } // if
        } // for
        logger().debug("ScavengerHuntAgentModeHandler - less crowded area[%d] number of areas[%d]. if ", lessCrowdedAreaIndex, NUMBER_OF_AREAS );

        if ( lessCrowdedAreaIndex != NUMBER_OF_AREAS ) {
            // there is an area to explore
            this->exploringAreas[lessCrowdedAreaIndex].push_back( looser );
            if ( looser == this->agent->getPetId( ) ) {
                this->exploringArea = lessCrowdedAreaIndex;
            } // if
        } // if
    } // if

}

bool ScavengerHuntAgentModeHandler::isAgentNextTo( const std::string& targetId )
{
    try {
        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );
        const spatial::EntityPtr& targetEntity = spaceMap.getEntity( targetId );

        if ( agentEntity->distanceTo( *targetEntity ) < 4000 ) {
            return true;
        } // if
        return false;
    } catch ( opencog::NotFoundException& ex ) {
        logger().debug("ScavengerHuntAgentModeHandler - There is no entity identifyed by[%s]", targetId.c_str( ) );
        return false;
    } // catch
}

bool ScavengerHuntAgentModeHandler::isTeamMember( const std::string& agentId, unsigned int teamCode )
{
    std::map<unsigned int, std::vector<std::string> >::const_iterator members = this->teamPlayers.find( teamCode );
    if ( members == this->teamPlayers.end( ) ) {
        return false;
    } // if

    return ( std::find( members->second.begin( ), members->second.end( ), agentId ) != members->second.end( ) );
}

void ScavengerHuntAgentModeHandler::updateStartPositionInCustomPath( void )
{
    
    try {
        std::stringstream output;
        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );

        output << agentEntity->getPosition( ).x << " " << agentEntity->getPosition( ).y << ";";

        std::string customPath = getPropertyValue( "customPath" );
        std::vector<std::string> path;
        boost::algorithm::split( path, customPath, boost::algorithm::is_any_of(";") );

        output << path[1];

        setProperty( "customPath", output.str( ) );
    } catch ( opencog::NotFoundException& ex ) {
        logger().error("ScavengerHuntAgentModeHandler - There is no entity identifyed by[%s]", getPropertyValue( "customObject" ).c_str( ) );
    } // catch
}

void ScavengerHuntAgentModeHandler::setFollowingPosition( void )
{
    try {
        const SpaceServer::SpaceMap& spaceMap = this->agent->getAtomSpace().getSpaceServer().getLatestMap();
        const spatial::EntityPtr& targetEntity = spaceMap.getEntity( getPropertyValue( "customObject" ) );
        const spatial::EntityPtr& agentEntity = spaceMap.getEntity( this->agent->getPetId( ) );

        spatial::Point finalPosition( agentEntity->getPosition( ).x, agentEntity->getPosition( ).y );

        spatial::math::Vector3 direction = agentEntity->getPosition( ) - targetEntity->getPosition( );

        if ( direction.length( ) > 2000.0 ) {
            direction.normalise( );
            spatial::math::Vector3 position = (direction * 2000.0) + targetEntity->getPosition( );
            finalPosition = spaceMap.getNearestFreePoint( spatial::Point( position.x, position.y ) );
        } // if

        std::stringstream output;

        output << agentEntity->getPosition( ).x << " " << agentEntity->getPosition( ).y << ";";
        output << finalPosition.first << " " << finalPosition.second;

        setProperty( "customPath", output.str( ) );
    } catch ( opencog::NotFoundException& ex ) {
        logger().error("ScavengerHuntAgentModeHandler - There is no entity identifyed by[%s]", getPropertyValue( "customObject" ).c_str( ) );
    } // catch
}

void ScavengerHuntAgentModeHandler::resetGame( void )
{
    changeState( this->agentState, 0 );
    getVisibilityMap( )->resetTiles( );
    this->exploredObjects.clear( );
    this->elapsedTicks = 0;
    this->players.clear( );
    this->teamPlayers.clear( );
    this->exploringAreas.clear( );
    this->exploredAreas.clear( );
    this->exploringArea = NUMBER_OF_AREAS;
    this->myTeamCode = 0;
    this->previousAgentState = 0;
    this->playing = false;
}
