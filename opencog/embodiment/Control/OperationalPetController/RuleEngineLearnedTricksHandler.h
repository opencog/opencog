#ifndef RULEENGINELEARNEDTRICKSHANDLER_H
#define RULEENGINELEARNEDTRICKSHANDLER_H

#include "OPC.h"

namespace OperationalPetController {

  class RuleEngineLearnedTricksHandler {
  public:
    static const std::string LEARNED_TRICK_NODE_NAME;
    static const int REWARD_VALUE;
    static const int PUNISHMENT_VALUE;
    static const int NOT_SELECTED_VALUE;

    RuleEngineLearnedTricksHandler( OPC* opc );
    
    ~RuleEngineLearnedTricksHandler( );

    void addLearnedSchema( const std::string& schemaName );

    void selectLearnedTrick( std::string& schemaName, std::set<std::string>& arguments ) throw (LADSUtil::NotFoundException);
    
    void rewardSchema( std::string& schemaName );
    
    void punishSchema( std::string& schemaName );

    bool hasLearnedTricks( void );
    
    void update( void );
    
  private:
    
    void addToSTIValue( Handle link, short value );

    OPC* opc;
    AtomSpace* atomSpace;
    Handle learnedTrickNode;
    std::set<Handle> punishedTricks;
    std::string latestSelectedTrick;
    int numberOfLearnedTricks;
    LADSUtil::RandGen* randGen;
  };

}; // OperationalPetController

#endif // RULEENGINELEARNEDTRICKSHANDLER_H
