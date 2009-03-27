#include <SystemParameters.h>
#include <exception>

#include "util/exceptions.h"
#include "Spawner.h"
#include "util/files.h"

using namespace MessagingSystem;
using namespace opencog;

void spawner_unexpected_handler(){
    throw;
}

int main(int argc, char *argv[]) {
    
    Spawner *spawner = NULL;
    Control::SystemParameters parameters;
    
    // if exists load file with configuration parameters 
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object. 
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    // setting unexpected handler in case a different exception from the
    // especified ones is throw in the code
    std::set_unexpected(spawner_unexpected_handler);

    system("./router &");
    sleep(5);
    system("./learningServer &");     
    //system("./pvpSimulator &"); // proxy

    try {
   
        server(Spawner::createInstance);
        spawner = &(static_cast<Spawner&>(server()));
        spawner->init(parameters, 
                     parameters.get("SPAWNER_ID"), 
                     parameters.get("SPAWNER_IP"), 
                     atoi(parameters.get("SPAWNER_PORT").c_str()));
        spawner->serverLoop();   
    
    } catch(opencog::InvalidParamException& ipe){
        logger().log(opencog::Logger::ERROR, 
                        "SpawnerExec - Error creating spawner object.");
    } catch(std::bad_alloc) {
        logger().log(opencog::Logger::ERROR, 
                        "SpawnerExec - Spawner raised a bad_alloc exception.");
    } catch(...) {
        logger().log(opencog::Logger::ERROR, 
        "Spawner executable - An exceptional situation occured. Check log for more information.");
    }
   
    // if spawner criated, delete it
    if(spawner){
        delete spawner; 
    }

    return (0);
}

