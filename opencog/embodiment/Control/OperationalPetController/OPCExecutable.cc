/*
 * opencog/embodiment/Control/MessagingSystem/OPCExecutable.cc
 *
 * Copyright (C) 2007-2008 Carlos Lopes
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <SystemParameters.h>
#include <exception>
#include <unistd.h>

#include "util/exceptions.h"
#include "util/files.h"
#include "OPC.h"

using namespace OperationalPetController;

void opc_unexpected_handler(){
    throw;
}

int main(int argc, char *argv[]) {

    if(argc != 6){
        logger().log(opencog::Logger::ERROR, "OPCExec - Usage: \n\topc <agent-id> <owner-id> <agent-type> <agent-traits> <port>.");
        return (1);
    }

    Control::SystemParameters parameters;
    
    // if exists load file with configuration parameters 
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.    
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }
    
    // setting unexpected handler in case a different exception from the
    // especified ones is throw in the code
    std::set_unexpected(opc_unexpected_handler);
   
    //char petName[256];
    //int petID = atoi(argv[1]);
    //int portNumber = 5100 + petID;
    int portNumber = atoi(argv[5]);

    server(OPC::createInstance);
    OPC& opc = static_cast<OPC&>(server());
    opc.init(argv[1], "127.0.0.1", portNumber, 
             PerceptionActionInterface::PAIUtils::getInternalId(argv[1]), 
             PerceptionActionInterface::PAIUtils::getInternalId(argv[2]), 
		     argv[3], argv[4], parameters);
    try {
        opc.serverLoop();
    } catch(std::bad_alloc){
        logger().log(Logger::ERROR,
                     "OPCExec - OPC raised a bad_alloc exception.");
        opc.saveState();        
    } catch(StandardException se) {
        logger().log(Logger::ERROR,
                     "OPC executable - An exceptional situation occured"
                     " with the following message '%s'"
                     ". Check log for more information.",
                     se.getMessage());
        opc.saveState();
    } catch(...) {
        logger().log(Logger::ERROR, 
                     "OPC executable - An exceptional situation occured"
                     ". Check log for more information.");
        opc.saveState();
    }
    
    // TODO: how to delete opc now?
    //delete opc;
    return (0);
}
