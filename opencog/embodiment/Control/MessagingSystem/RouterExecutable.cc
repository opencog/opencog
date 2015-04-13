/*
 * opencog/embodiment/Control/MessagingSystem/RouterExecutable.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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
#include "Router.h"

using namespace opencog::messaging;
using namespace opencog;

void router_unexpected_handler()
{
    throw;
}

int main(int argc, char *argv[])
{

    config(opencog::control::EmbodimentConfig::embodimentCreateInstance, true);

    // If exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.
    if (fileExists(config().get("CONFIG_FILE").c_str())) {
        config().load(config().get("CONFIG_FILE").c_str());
    }

    // Setting unexpected handler in case a different exception from the
    // specified ones is thrown in the code
    std::set_unexpected(router_unexpected_handler);

    Router *router = new Router();

    try {
        router->run();
    } catch (std::bad_alloc) {
        opencog::logger().error("RouterExec - Router raised a bad_alloc exception.");
        router->persistState();
    } catch (opencog::NetworkException& e) {
        opencog::logger().error("RouterExec - Router raised a Runtime exception.");
        router->persistState();
    } catch (...) {
        opencog::logger().error(
                              "RouterExec - An exceptional situation occured. Check log for more information.");
        router->persistState();
    }

    delete router;
    return (0);
}
