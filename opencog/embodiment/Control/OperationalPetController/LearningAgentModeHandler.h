#ifndef LEARNING_AGENT_MODE_HANDLER_H
#define LEARNING_AGENT_MODE_HANDLER_H

#include "AgentModeHandler.h"

namespace OperationalPetController {
  class Pet;
  /**
   * Handle commands used by Learning mode
   */
  class LearningAgentModeHandler : public Control::AgentModeHandler {
  public:
    LearningAgentModeHandler( Pet* agent );
    inline virtual ~LearningAgentModeHandler( void ) { };

    void handleCommand( const std::string& name, const std::vector<std::string>& arguments );

    inline const std::string& getModeName( void ) { return this->modeName; }

    void update( void );

  protected:    
    const std::string modeName;
    Pet* agent;
  };

}; // OperationalPetController

#endif // LEARNING_AGENT_MODE_HANDLER_H
