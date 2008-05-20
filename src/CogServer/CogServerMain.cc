#include "CogServer.h"
#include "QueryProcessor.h"
#include "WordSenseProcessor.h"
#include "Config.h"

using namespace opencog;

static void usage(char* progname) {
    cerr << "Usage: " << progname << " [-c <config-file>]\n\n";
}

int main(int argc, char *argv[]) {
    // check command line
    if ((argc == 1) || ((argc == 3) && (strcmp(argv[1], "-c") == 0))) {
        // load the configuration from file if a filename was supplied on the command line
        if (argc == 3) {
            try {
                config().load(argv[2]);
            } catch (RuntimeException &e) {
                cerr << e.getMessage() << std::endl;
                return 1;
            }
        }

        // create server instance
        CogServer server;

        // Cheapo hack to get the query processory up and running.
        // XXX fix me with some more permanent, appropriate solution.
        server.plugInMindAgent(new QueryProcessor(), 1);
        server.plugInMindAgent(new WordSenseProcessor(), 1);

        server.serverLoop();
    } else {
        usage(argv[0]);
        return 1;
    }
}
