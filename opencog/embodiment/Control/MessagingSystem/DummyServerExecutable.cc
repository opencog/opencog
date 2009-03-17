#include <SystemParameters.h>
#include "DummyServer.h"
#include <LADSUtil/files.h>

using namespace MessagingSystem;

int main(int argc, char *argv[]) {

    Control::SystemParameters parameters;
    
    // if exists load file with configuration parameters 
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object. 
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }    

    DummyServer *server = new DummyServer(parameters, argv[1], argv[2], atoi(argv[3]));
    server->serverLoop();
    return 0;
}

