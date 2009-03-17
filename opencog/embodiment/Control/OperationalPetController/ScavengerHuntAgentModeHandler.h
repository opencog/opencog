#ifndef SCAVENGER_HUNT_AGENT_MODE_HANDLER_H
#define SCAVENGER_HUNT_AGENT_MODE_HANDLER_H

#include "AgentModeHandler.h"
#include "VisibilityMap.h"
#include <sstream>
#include <ProcedureInterpreter.h>

namespace OperationalPetController {
    class Pet;
    /**
     * Handle commands used by scavenger Hunt mode
     */
    class ScavengerHuntAgentModeHandler : public Control::AgentModeHandler {
    public:

        static const unsigned int NUMBER_OF_AREAS;
        static const std::string RED_TEAM_BASE;
        static const std::string BLUE_TEAM_BASE;

        class PlayerMetaData {
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

        inline const std::string& getModeName( void ) { return this->modeName; }

        void update( void );

    protected:        
        Spatial::VisibilityMap* getVisibilityMap( void );

    
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

        // TODO Use gonear_obj instead after it was fixed
        void setFollowingPosition( void );
        void resetGame( void );

        void addPlayer( const std::string& playerName, unsigned int teamCode, float randomNumber );
        void selectArea( const std::string& playerName, unsigned int areaNumber, float randomNumber );



        std::string computeWalkAroundWaypoints( const std::string& entityId );
        std::string computeWalkAroundWaypoints( const Spatial::Math::Vector3& targetPosition );

        const std::string modeName;
        Pet* agent;
        Spatial::VisibilityMap* visibilityMap;
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

}; // OperationalPetController

#endif // SCAVENGER_HUNT_AGENT_MODE_HANDLER_H
