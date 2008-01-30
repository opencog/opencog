#include "CogServer.h"

using namespace opencog;

int main(int argc, char *argv[]) {
    CogServer server;
    server.serverLoop();
}
