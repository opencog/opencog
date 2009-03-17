#ifndef DEFAULT_AGENT_MODE_HANDLER
#define DEFAULT_AGENT_MODE_HANDLER

#include "AgentModeHandler.h"
#include "VisibilityMap.h"

namespace OperationalPetController {
  class Pet;
  /**
   * If a given mode doesn't need a handler, use a default handler to be returned by PetInterface
   */
  class DefaultAgentModeHandler : public Control::AgentModeHandler {
  public:
    
    DefaultAgentModeHandler( Pet* agent );

    virtual inline ~DefaultAgentModeHandler( void ) { };
    
    void handleCommand( const std::string& name, const std::vector<std::string>& arguments );

    inline const std::string& getModeName( void ) { return this->modeName; }

    void update( void );

    Spatial::VisibilityMap* getVisibilityMap( void );

  protected:
    const std::string modeName;
    Pet* agent;
    Spatial::VisibilityMap* visibilityMap;
  };
}; // OperationalPetController

#endif // DEFAULT_AGENT_MODE_HANDLER
