#include <SystemParameters.h>
#include <unistd.h>
#include <LADSUtil/files.h>
#include "MockOpcHCTest.h"

using namespace OperationalPetController;

int main(int argc, char *argv[]) {
  
  LADSUtil::cassert(TRACE_INFO, argc == 3);
  Control::SystemParameters parameters;
  
  // if exists load file with configuration parameters 
  // IMPORTANT: this file should be the same for all executables that create
  // a systemParameter object.    
  if(fileExists(parameters.get("CONFIG_FILE").c_str())){
  	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
  }
  
  
  //char petName[256];
  //int petID = atoi(argv[1]);
  //int portNumber = 5100 + petID;
  int portNumber = atoi(argv[2]);
  //sprintf(petName, "%d", petID);
  
  MockOpcHCTest *mOpcHcTest = 
    new MockOpcHCTest(argv[1], "127.0.0.1", portNumber, argv[1], parameters);
  mOpcHcTest->serverLoop();
  delete mOpcHcTest;

  return 0;
}
