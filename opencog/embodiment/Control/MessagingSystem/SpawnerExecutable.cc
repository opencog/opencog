/*
 * opencog/embodiment/Control/MessagingSystem/SpawnerExecutable.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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
#include <opencog/util/exceptions.h>
#include <opencog/util/files.h>
#include <opencog/util/macros.h>
#include "Spawner.h"

using namespace opencog::messaging;
using namespace opencog;
using namespace opencog::control;

void spawner_unexpected_handler()
{
    throw;
}

int main(int argc, char *argv[])
{

    Spawner *spawner = NULL;
    
    config(EmbodimentConfig::embodimentCreateInstance, true);

    // if exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.
    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }

    // setting unexpected handler in case a different exception from
    // the specified ones is thrown in the code
    std::set_unexpected(spawner_unexpected_handler);

	int i;
    i = system("./router &");
    sleep(5);
    // i = system("./learningServer &");
    //i = system("./pvpSimulator &"); // proxy
	OC_UNUSED( i );
	
    try {

        server(Spawner::createInstance);
        spawner = &(static_cast<Spawner&>(server()));
        spawner->init(config().get("SPAWNER_ID"),
                      config().get("SPAWNER_IP"),
                      config().get_int("SPAWNER_PORT"));
        spawner->serverLoop();

    } catch (opencog::InvalidParamException& ipe) {
        logger().error(
                     "SpawnerExec - Error creating spawner object.");
    } catch (std::bad_alloc) {
        logger().error(
                     "SpawnerExec - Spawner raised a bad_alloc exception.");
    } catch (...) {
        logger().error(
                     "Spawner executable - An exceptional situation occured. Check log for more information.");
    }

    // if spawner criated, delete it
    if (spawner) {
        delete spawner;
    }

    return (0);
}

