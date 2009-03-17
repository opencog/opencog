#include <SystemParameters.h>
#include <exception>

#include <LADSUtil/exceptions.h>
#include "Router.h"
#include <LADSUtil/files.h>

using namespace MessagingSystem;

void router_unexpected_handler(){
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
    std::set_unexpected(router_unexpected_handler);
       
    Router *router = new Router(parameters);
    
    try{
        router->run();
    } catch(std::bad_alloc){
        MAIN_LOGGER.log(LADSUtil::Logger::ERROR, "RouterExec - Router raised a bad_alloc exception.");
        router->persistState();
    } catch(LADSUtil::NetworkException& e){
        MAIN_LOGGER.log(LADSUtil::Logger::ERROR, "RouterExec - Router raised a Runtime exception.");
        router->persistState();
    } catch(...){
         MAIN_LOGGER.log(LADSUtil::Logger::ERROR, 
             "RouterExec - An exceptional situation occured. Check log for more information.");
        router->persistState();
    }

    delete router;
    return (0);
}
