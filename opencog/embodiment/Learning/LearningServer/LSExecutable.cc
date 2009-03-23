#include <SystemParameters.h>
#include <exception>
#include <cstdlib>
#include "util/files.h"
#include "util/Logger.h"
#include "LS.h"

using namespace LearningServer;
using namespace opencog;

void ls_unexpected_handler(){
    throw;
}

int main(int argc, char *argv[]) {
  
    Control::SystemParameters parameters;

    // if exists load file with configuration parameters 
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object. 
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    // setting unexpected handler in case a different exception from the
    // especified ones is throw in the code
    std::set_unexpected(ls_unexpected_handler);

    LS * ls = new LS(parameters.get("LS_ID"),
		     parameters.get("LS_IP"),
		     std::atoi(parameters.get("LS_PORT").c_str()),
		     parameters);

    try  {
        ls->serverLoop();

    } catch(std::bad_alloc){
        logger().log(opencog::Logger::ERROR, "LSExec - LS raised a bad_alloc exception.");
        
    } catch(...) {
        logger().log(opencog::Logger::ERROR, 
                "LSExec - An exceptional situation occured. Check log for more information.");
    }

    delete ls;

    return (0);
}
