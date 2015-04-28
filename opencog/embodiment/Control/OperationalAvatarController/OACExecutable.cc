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

using namespace opencog::oac;
using namespace opencog;
using namespace opencog::pai;

void oac_unexpected_handler()
{
    throw;
}

int main(int argc, char *argv[])
{
    try {
        config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);
        
        // if exists load file with configuration parameters
        // IMPORTANT: this file should be the same for all executables that create
        // a systemParameter object.
        if (fileExists(config().get("CONFIG_FILE").c_str())) {
            config().load(config().get("CONFIG_FILE").c_str());
        }

         // take care of the args number.
        if ( config().get_bool("ENABLE_UNITY_CONNECTOR") && argc != 9 ) {
            logger().error("OACExec - Usage: \n\toac <agent-brain-id> <owner-id> <agent-type> <agent-traits> <NetworkElement port> <CogServer shell port> <ZeroMQ publish port> <agent-id>");
            return (1);
        }
        else if ( !config().get_bool("ENABLE_UNITY_CONNECTOR") && argc != 8 ) {
            logger().error("OACExec - Usage: \n\toac <agent-brain-id> <owner-id> <agent-type> <agent-traits> <NetworkElement port> <CogServer shell port> <ZeroMQ publish port>");
            return (1);
        }
       
        config().set("EXTERNAL_TICK_MODE", "true");
        config().set("SERVER_PORT", argv[6]);
        config().set("ZMQ_PUBLISH_PORT", argv[7]);

        // This is a hack by Troy!!! To redirect message to OCAvatar in unity game.
        if( config().get_bool("ENABLE_UNITY_CONNECTOR") ) {
			config().set("PROXY_ID", argv[8]);
		}

        // setting unexpected handler in case a different exception from the
        // specified ones is thrown in the code
        std::set_unexpected(oac_unexpected_handler);
        
        //char petName[256];
        //int petID = atoi(argv[1]);
        //int portNumber = 5100 + petID;
        int portNumber = atoi(argv[5]);

        server(OAC::createInstance);
        OAC& oac = static_cast<OAC&>(server());
        // Open database *before* loading modules, since the modules
        // might create atoms, and we can't have that happen until 
        // storage is open, as otherwise, there will be handle conflicts.
        oac.openDatabase();

        // Load modules specified in config
        oac.loadModules(); 
        oac.loadSCMModules({"."});

        // Initialize OAC
        //
        // OAC::loadSCMModules should be called before calling OAC::init, 
        // because OAC::loadSCMModules will load 'rules_core.scm',  which should be loaded 
        // before loading Psi Rules ('xxx_rules.scm') and 
        // OAC::init is responsible for loading Psi Rules via OAC::addRulesToAtomSpace
        oac.init(
            argv[1],        // agent-brain-id, i.e., id of OAC
            "127.0.0.1",    // NetworkElement ip 
            portNumber,     // NetworkElement port number
            argv[7],        // ZeroMQ port used by subscribers to get messages
            PAIUtils::getInternalId(argv[1]), // pet id 
            PAIUtils::getInternalId(argv[2]), // owner id 
            argv[3], // agent type
            argv[4]  // agemt traits
        );

        // enable the network server and run the server's main loop
        oac.enableNetworkServer();

        // TODO: After making OAC a CogServerMain-compatible server, create a
        // shell command to list all configuration parameters (like MIN_STI
        // bellow). An perhaps a command to set a config parameter at runtime,
        // which may be very useful.
        logger().debug("MIN_STI = %s\n", config().get("MIN_STI").c_str());

        //main loop
        oac.serverLoop();

    } catch (std::bad_alloc) {
        logger().error("OACExecutable - OAC raised a bad_alloc exception.");
        static_cast<OAC&>(server()).saveState();
    } catch (StandardException se) {
        logger().error("OACExecutable - An exceptional situation occured"
                       " with the following message '%s'"
                       ". Check log for more information.",
                       se.getMessage());
        static_cast<OAC&>(server()).saveState();
    } /*catch (...) {
        logger().error(
                     "OACExecutable - An exceptional situation occured"
                     ". Check log for more information.");
        static_cast<OAC&>(server()).saveState();
    }*/

    return (0);
}
