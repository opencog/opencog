/**
 * HCTestAgent.h
 *
 * Author: Nil Geisweiller
 * Creation: Thu Sep 13 2007
 */

#ifndef HCTESTAGENT_H
#define HCTESTAGENT_H

#include <opencog/server/Agent.h>
#include "MessageSender.h"

namespace OperationalPetController {

  using namespace opencog;

  /**
   * That class is in charge of executing a scenario to test hillclimbing
   * hosted by LS
   */

  class HCTestAgent : public Agent {
    
    enum HCTestMode {
      HCT_IDLE,
      HCT_INIT,
      HCT_WAIT1,
      HCT_WAIT2,
      HCT_WAIT3,
      HCT_WAIT4
    };
    
  private:
    HCTestMode mode;
    unsigned long cycle;
    std::string schemaName;
    std::vector<std::string> schemaArguments;
    std::string avatarId;
    std::string ownerId;
    SpaceServer* spaceServer;
    MessageSender* sender;
    
  public:
    
    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalPetController::HCTestAgent");
        return _ci;
    }

    HCTestAgent();
    void init(std::string sn, std::vector<std::string> schemaArgs, std::string b, std::string a, SpaceServer* ss, MessageSender* s);
    ~HCTestAgent();
    
    void run(opencog::CogServer* ne);

    void setWait2() {
      mode = HCT_WAIT2;
    }
    
    void setWait4() {
      mode = HCT_WAIT4;
    }

  }; // class
}  // namespace

#endif
