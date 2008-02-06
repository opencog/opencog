#include "CogServerSetup.h"
#include "SimpleNetworkServer.h"
#include "CommandRequestProcessor.h"

using namespace opencog;

CogServerSetup::~CogServerSetup() {
}

CogServerSetup::CogServerSetup() {
}

#define COG_SERVER_DEFAULT_PORT 17001

void CogServerSetup::setUp(CogServer *server)
{
    server->setNetworkServer(new SimpleNetworkServer(server, COG_SERVER_DEFAULT_PORT));
}
