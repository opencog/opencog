#include "CogServerSetup.h"
#include "SimpleNetworkServer.h"
#include "CommandRequestProcessor.h"

using namespace opencog;

CogServerSetup::~CogServerSetup() {
}

CogServerSetup::CogServerSetup() {
}

void CogServerSetup::setUp(CogServer *server) {
    server->setNetworkServer(new SimpleNetworkServer(server, 17001));
    server->plugInRequestProcessor("COMMAND_LINE", new CommandRequestProcessor());
}
