/**
 * PBTester.cc
 * This program performs automated system tests on PB-side of Petaverse code.
 * It simulates the PVP Proxy by reading from a file a sequence of messages to be sent and 
 * received to/from Petaverse ROUTER. 
 *
 * Author: Welter Luigi
 */


#include "PBTester.h"
#include <exception>
#include <unistd.h>
#include "util/files.h"
#include "GoldStdReaderTask.h"

int main(int argc, char *argv[]) {

    // Open/read the data file passed as argument
    if (argc < 2) {
        printf("Wrong number of arguments:\nExpected: %s <Gold Standard Filename>\n", argv[0]);
    }
    const char* filename = argv[1]; 

    Control::SystemParameters parameters;
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    AutomatedSystemTest::TestParameters testParameters;
    if(fileExists(testParameters.get("CONFIG_FILE").c_str())){
    	testParameters.loadFromFile(testParameters.get("CONFIG_FILE"));
    }

//    AutomatedSystemTest::PBTester pbTester(parameters, testParameters, parameters.get("PROXY_ID"), parameters.get("PROXY_IP"), atoi(parameters.get("PROXY_PORT").c_str()));
    AutomatedSystemTest::PBTester pbTester(parameters, testParameters, parameters.get("PROXY_ID"), testParameters.get("PROXY_IP"), atoi(testParameters.get("PROXY_PORT").c_str()));

    pbTester.plugInIdleTask(new AutomatedSystemTest::GoldStdReaderTask(testParameters, filename), 1);

    try {
        pbTester.serverLoop();
    } catch(std::bad_alloc){
        MAIN_LOGGER.log(opencog::Logger::ERROR, "PBTesterExec - PBTester raised a bad_alloc exception.");
       
    } catch(...) {
        MAIN_LOGGER.log(opencog::Logger::ERROR, 
        "PBTesterExec - An exceptional situation occured. Check log for information.");
    }

    return 0;
}

