#include <SystemParameters.h>
#include <exception>
#include "util/exceptions.h"
#include "PVPSimulator.h"
#include "SimulationParameters.h"
#include "TickerTask.h"
#include "InterfaceListenerTask.h"
#include "MessageSenderTask.h"
#include "util/files.h"
#include <StringMessage.h>
#include <unistd.h>

int main(int argc, char *argv[]) {

    Control::SystemParameters parameters;
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    PetaverseProxySimulator::SimulationParameters simParameters;
    if(fileExists(simParameters.get("CONFIG_FILE").c_str())){
    	simParameters.loadFromFile(simParameters.get("CONFIG_FILE"));
    }
    //PetaverseProxySimulator::PVPSimulator simulator(parameters, simParameters, parameters.get("PROXY_ID"), "127.0.0.1", 8211);

    PetaverseProxySimulator::PVPSimulator simulator(parameters, simParameters, parameters.get("PROXY_ID"), "127.0.0.1", 16315);

    if (!simulator.connectToSimWorld()) {
        logger().log(opencog::Logger::ERROR, "Could not connect to the simulated World\n");
        exit(0);
    }

    simulator.plugInIdleTask(new PetaverseProxySimulator::TickerTask(simParameters), 1);
    simulator.plugInIdleTask(new PetaverseProxySimulator::InterfaceListenerTask(), 1);
    simulator.plugInIdleTask(new PetaverseProxySimulator::MessageSenderTask(), 1);
    simParameters.startSimulation();

    try {
        simulator.serverLoop();
    } catch(std::bad_alloc){
        logger().log(opencog::Logger::ERROR, "PVPSimExec - PVPSim raised a bad_alloc exception.");
        simulator.persistState();

    } catch(...) {
        logger().log(opencog::Logger::ERROR, 
        "PVPSimExec - An exceptional situation occured. Check log for information.");
        simulator.persistState();
    }

    return 0;
}

