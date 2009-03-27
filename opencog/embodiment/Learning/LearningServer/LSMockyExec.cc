#include <SystemParameters.h>
#include <cstdlib>
#include "util/files.h"
#include "LSMocky.h"

using namespace LearningServer;

int main(int argc, char *argv[]) {
  
	Control::SystemParameters parameters;

    // if exists load file with configuration parameters 
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object. 
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    server(LSMocky::createInstance);
    LSMocky& ls = static_cast<LSMocky&>(server());
  	ls.init(parameters.get("LS_ID"), 
  			parameters.get("LS_IP"), 
  			std::atoi(parameters.get("LS_PORT").c_str()), 
  			parameters);
  	ls.serverLoop();
  	return 0;
}
