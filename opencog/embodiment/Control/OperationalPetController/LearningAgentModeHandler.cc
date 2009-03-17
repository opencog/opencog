#include "LearningAgentModeHandler.h"
#include <LADSUtil/Logger.h>
#include "Pet.h"

using namespace OperationalPetController;

LearningAgentModeHandler::LearningAgentModeHandler( Pet* agent ) : 
  modeName( "LEARNING_MODE" ), agent(agent) { 
}

void LearningAgentModeHandler::handleCommand( const std::string& name, const std::vector<std::string>& arguments ) {
}

void LearningAgentModeHandler::update( void ) {
}
