#include "CogServer.h"
#include "QueryProcessor.h"

using namespace opencog;

int main(int argc, char *argv[])
{
    CogServer server;

    // Cheapo hack to get the query processory up and running.
    // XXX fix me with some more permanent, appropriate solution.
    QueryProcessor *qp = new QueryProcessor();
    server.plugInMindAgent(qp, 1);
    server.serverLoop();
}
