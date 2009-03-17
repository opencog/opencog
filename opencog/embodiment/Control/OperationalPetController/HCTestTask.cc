/**
 * HCTestTask.cc
 *
 * Author: Nil Geisweiller
 * Creation: Thu Sep 13 2007
 */

#include "HCTestTask.h"
#include "OPC.h"

#define IDLE_TIME 1
#define WAIT1_TIME 10 //time to wait for the first learning iteration
#define WAIT2_TIME 2
#define WAIT3_TIME 100 //time to wait for the second learning iteration
#define WAIT4_TIME 2
#define MAX_CYCLE 40000

using namespace OperationalPetController;

HCTestTask::HCTestTask(std::string sn, std::vector<std::string> schemaArgs, std::string a, std::string b, SpaceServer* ss, MessageSender* s)
  : mode(HCT_INIT), cycle(0), schemaName(sn), schemaArguments(schemaArgs), avatarId(a), ownerId(b), spaceServer(ss), sender(s) {
}
HCTestTask::~HCTestTask() {
}


void HCTestTask::run(MessagingSystem::NetworkElement* ne) {
  logger().log(opencog::Logger::FINE, "Executing HCTestTask.");    
  cycle++;
  if(cycle > MAX_CYCLE) { //timeout in case the test takes too long
    logger().log(opencog::Logger::ERROR, "Executing HCTestTask.");
    exit(1);
  }
  // send the whole atomSpace to the LS
  switch(mode) {
  case HCT_IDLE:
    std::cout << "OPC IDLE" << std::endl;
    sleep(IDLE_TIME);
    break;
  case HCT_INIT:
    std::cout << "OPC INIT" << std::endl;
    sender->sendExemplar(schemaName, schemaArguments, ownerId, avatarId, *spaceServer);
    mode = HCT_WAIT1;
    break;
  case HCT_WAIT1:
    std::cout << "OPC WAIT1" << std::endl;
    sleep(WAIT1_TIME);
    std::cout << "OPC WAIT1 DONE" << std::endl;
    sender->sendCommand(NetworkElement::parameters.get("TRY_SCHEMA_CMD"),
			schemaName);
    mode = HCT_IDLE;
    break;

  case HCT_WAIT2:
    std::cout << "OPC WAIT2" << std::endl;
    sleep(WAIT2_TIME);
    std::cout << "OPC WAIT2 DONE" << std::endl;
    sender->sendExemplar(schemaName, schemaArguments, ownerId, avatarId, *spaceServer);
    mode = HCT_WAIT3;
    break;

  case HCT_WAIT3:
    std::cout << "OPC WAIT3" << std::endl;
    sleep(WAIT3_TIME);
    std::cout << "OPC WAIT3 DONE" << std::endl;
    sender->sendCommand(NetworkElement::parameters.get("TRY_SCHEMA_CMD"),
			schemaName);
    mode = HCT_IDLE;
    break;

  case HCT_WAIT4:
    std::cout << "OPC WAIT4" << std::endl;
    sleep(WAIT4_TIME);
    std::cout << "OPC WAIT4 DONE" << std::endl;
    sender->sendCommand(NetworkElement::parameters.get("STOP_LEARNING_CMD"),
			schemaName);
    mode = HCT_IDLE;
    break;

  default:
    break;
  }
}
