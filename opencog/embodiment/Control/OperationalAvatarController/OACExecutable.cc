/*
 * opencog/embodiment/Control/OperationalAvatarController/OACExecutable.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <exception>
#include <unistd.h>

#include <opencog/util/exceptions.h>
#include <opencog/util/files.h>
#include "OAC.h"

using namespace OperationalAvatarController;
using namespace opencog;

void opc_unexpected_handler()
{
    throw;
}

int main(int argc, char *argv[])
{
    // take care of the args number.
    if (argc != 8) {
        logger().error("OACExec - Usage: \n\topc <agent-brain-id> <owner-id> <agent-type> <agent-traits> <NetworkElement port> <CogServer shell port> <agent-id>.");
        return (1);
    }

    try {
        config(Control::EmbodimentConfig::embodimentCreateInstance, true);
        
        // if exists load file with configuration parameters
        // IMPORTANT: this file should be the same for all executables that create
        // a systemParameter object.
        if (fileExists(config().get("CONFIG_FILE").c_str())) {
            config().load(config().get("CONFIG_FILE").c_str());
        }
        
        config().set("EXTERNAL_TICK_MODE", "true");
        config().set("SERVER_PORT", argv[6]);

        // This is a hack!!! To redirect message to OCAvatar in unity game.
        config().set("PROXY_ID", argv[7]);

        // setting unexpected handler in case a different exception from the
        // especified ones is throw in the code
        std::set_unexpected(opc_unexpected_handler);
        
        //char petName[256];
        //int petID = atoi(argv[1]);
        //int portNumber = 5100 + petID;
        int portNumber = atoi(argv[5]);
        
        server(OAC::createInstance);
        OAC& opc = static_cast<OAC&>(server());
        // Open database *before* loading modules, since the modules
        // might create atoms, and we can't have that happen until 
        // storage is open, as otherwise, there will be handle conflicts.
        opc.openDatabase();

        opc.init(argv[1], "127.0.0.1", portNumber, 
                PerceptionActionInterface::PAIUtils::getInternalId(argv[1]), 
                PerceptionActionInterface::PAIUtils::getInternalId(argv[2]), 
                argv[3], argv[4]);
        

        // Load modules specified in config
        opc.loadModules(); 
        const char* config_path[] = {"."};
        opc.loadSCMModules(config_path);

        // enable the network server and run the server's main loop
        opc.enableNetworkServer();

        // TODO: After making OAC a CogServerMain-compatible server, create a
        // shell command to list all configuration parameters (like MIN_STI
        // bellow). An perhaps a command to set a config parameter at runtime,
        // which may be very useful.
        logger().debug("MIN_STI = %s\n", config().get("MIN_STI").c_str());

        //main loop
        opc.serverLoop();

    } catch (std::bad_alloc) {
        logger().error(
                     "OACExecutable - OAC raised a bad_alloc exception.");
        static_cast<OAC&>(server()).saveState();
    } catch (StandardException se) {
        logger().error(
                     "OACExecutable - An exceptional situation occured"
                     " with the following message '%s'"
                     ". Check log for more information.",
                     se.getMessage());
        static_cast<OAC&>(server()).saveState();
    } catch (...) {
        logger().error(
                     "OACExecutable - An exceptional situation occured"
                     ". Check log for more information.");
        static_cast<OAC&>(server()).saveState();
    }

    return (0);
}
