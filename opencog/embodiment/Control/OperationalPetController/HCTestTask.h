/**
 * HCTestTask.h
 *
 * Author: Nil Geisweiller
 * Creation: Thu Sep 13 2007
 */

#ifndef HCTESTTASK_H
#define HCTESTTASK_H

#include <NetworkElement.h>
#include "MessageSender.h"

namespace OperationalPetController {

  /**
   * That class is in charge of executing a scenario to test hillclimbing
   * hosted by LS
   */

  class HCTestTask : public MessagingSystem::IdleTask {
    
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
    
    HCTestTask(std::string sn, std::vector<std::string> schemaArgs, std::string b, std::string a, SpaceServer* ss, MessageSender* s);
    ~HCTestTask();
    
    void run(MessagingSystem::NetworkElement* ne);

    void setWait2() {
      mode = HCT_WAIT2;
    }
    
    void setWait4() {
      mode = HCT_WAIT4;
    }

  }; // class
}  // namespace

#endif
