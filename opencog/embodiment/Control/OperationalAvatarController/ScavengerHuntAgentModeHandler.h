/*
 * opencog/embodiment/Control/OperationalAvatarController/ScavengerHuntAgentModeHandler.h
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

#ifndef SCAVENGER_HUNT_AGENT_MODE_HANDLER_H
#define SCAVENGER_HUNT_AGENT_MODE_HANDLER_H

#include <opencog/embodiment/Control/OperationalAvatarController/BaseAgentModeHandler.h>
#include <opencog/spatial/VisibilityMap.h>
#include <sstream>
#include <opencog/embodiment/Control/Procedure/ProcedureInterpreter.h>

namespace opencog { namespace oac {

class Pet;

/**
 * Handle commands used by scavenger Hunt mode
 */
class ScavengerHuntAgentModeHandler : public BaseAgentModeHandler
{
public:

    static const unsigned int NUMBER_OF_AREAS;
    static const std::string RED_TEAM_BASE;
    static const std::string BLUE_TEAM_BASE;

    class PlayerMetaData
    {
    public:
        inline PlayerMetaData( const std::string& name, unsigned int suggestedCode, float randomNumber ) :
                name( name ), suggestedCode(suggestedCode), randomNumber(randomNumber) { }

        inline ~PlayerMetaData( void ) { }

        inline bool operator==( const PlayerMetaData& o ) const {
            return this->name == o.name;
        }
        inline bool operator<( const PlayerMetaData& o ) const {
            return ( this->randomNumber > o.randomNumber ||
                     this->suggestedCode < this->suggestedCode ||
                     this->name < o.name );
        }

        inline PlayerMetaData& operator=( const PlayerMetaData& o ) {
            this->name = o.name;
            this->suggestedCode = o.suggestedCode;
            this->randomNumber = o.randomNumber;
            return *this;
        }

        std::string toString( void ) const {
            std::stringstream str;
            str << "name[" << name << "] suggestedCode[" << suggestedCode << "] randomNumber[" << randomNumber << "]";
            return str.str( );
        }

        std::string name;
        unsigned int suggestedCode;
        float randomNumber;
    }; // PlayerMetaData


    ScavengerHuntAgentModeHandler( Pet* agent );
    inline virtual ~ScavengerHuntAgentModeHandler( void ) { };

    void changeState( int from, int to );

    void handleCommand( const std::string& name, const std::vector<std::string>& arguments );

    inline const std::string& getModeName( void ) {
        return this->modeName;
    }

    void update( void );

protected:
    spatial::VisibilityMap* getVisibilityMap( void );


    bool isThereObjectToInspectInsideTargetArea( void );
    bool isThereAreaToInspect( void );
    bool treasureWasFound( void );
    bool isAgentNearTreasure( void );
    bool isAgentNearOwner( void );
    bool isAgentHoldingTreasure( void );
    bool isAgentNearBase( void );
    bool isThereVisibleTilesInsideTargetArea( void );
    bool isThereHiddenTilesInsideTargetArea( void );
    bool imInsideTargetArea( void );
    bool isExecutingSchema( void );
    bool isAgentNextTo( const std::string& targetId );
    bool isTeamMember( const std::string& agentId, unsigned int teamCode );

    /**
     * This method must be called each time update was called if the current state was using
     * customPath property. It is necessary because the agent position can become older
     * into the customPath string and when it will be executed strange behaviors can be noted.
     * So, this method will update the property with the current agent's 
     * position to avoid problems.
     */
    void updateStartPositionInCustomPath( void );

    // TODO Use gonear_obj instead after it was fixed
    void setFollowingPosition( void );
    void resetGame( void );

    void addPlayer( const std::string& playerName, unsigned int teamCode, float randomNumber );
    void selectArea( const std::string& playerName, unsigned int areaNumber, float randomNumber );



    std::string computeWalkAroundWaypoints( const std::string& entityId );
    std::string computeWalkAroundWaypoints( const spatial::math::Vector3& targetPosition );

    const std::string modeName;
    Pet* agent;
    spatial::VisibilityMap* visibilityMap;
    int agentState;
    int previousAgentState;

    std::vector<std::string> exploredObjects;
    unsigned int elapsedTicks;

    std::map<std::string, PlayerMetaData> players;
    std::map<unsigned int, std::vector<std::string> > teamPlayers;
    std::vector< std::vector<std::string> > exploringAreas;
    std::vector<bool> exploredAreas;
    unsigned int exploringArea;
    unsigned int myTeamCode;
    bool playing;

    std::string redBaseId;
    std::string blueBaseId;

    Procedure::RunningProcedureID runningProcedureId;
    bool needToCollectProcedureId;
};

} } // namespace opencog::oac

#endif // SCAVENGER_HUNT_AGENT_MODE_HANDLER_H
