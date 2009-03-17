#include "ComboShellServer.h"
#include "util/files.h"
#include <SystemParameters.h>

int main(int argc,char** argv) {
  using namespace MessagingSystem;

  // set up the system for talking to the router
  Control::SystemParameters parameters;
  // if exists load file with configuration parameters 
  // IMPORTANT: this file should be the same for all executables that create
  // a systemParameter object. 

  if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    parameters.loadFromFile(parameters.get("CONFIG_FILE"));
  }

  ComboShellServer css=ComboShellServer(parameters);
  css.serverLoop();
  return 0;
}
