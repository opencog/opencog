#include <Logger.h>
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
    Util::Logger* log;

    // Setup main logger
    log = new Util::Logger("CogServer.txt", Util::Logger::INFO, true);
    log->setPrintToStdoutFlag(true);
    Util::Logger::initMainLogger(log);
    log->log(Util::Logger::INFO,"Logger set up!");

    
    server->setNetworkServer(new SimpleNetworkServer(server, COG_SERVER_DEFAULT_PORT));

}
