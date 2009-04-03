/*
 * opencog/embodiment/Control/MessagingSystem/RouterExecutable.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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

#include "util/exceptions.h"
#include "Router.h"
#include "util/files.h"

using namespace MessagingSystem;

void router_unexpected_handler()
{
    throw;
}

int main(int argc, char *argv[])
{

    Control::SystemParameters parameters;

    // if exists load file with configuration parameters
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object.
    if (fileExists(parameters.get("CONFIG_FILE").c_str())) {
        parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    // setting unexpected handler in case a different exception from the
    // especified ones is throw in the code
    std::set_unexpected(router_unexpected_handler);

    Router *router = new Router(parameters);

    try {
        router->run();
    } catch (std::bad_alloc) {
        opencog::logger().log(opencog::Logger::ERROR, "RouterExec - Router raised a bad_alloc exception.");
        router->persistState();
    } catch (opencog::NetworkException& e) {
        opencog::logger().log(opencog::Logger::ERROR, "RouterExec - Router raised a Runtime exception.");
        router->persistState();
    } catch (...) {
        opencog::logger().log(opencog::Logger::ERROR,
                              "RouterExec - An exceptional situation occured. Check log for more information.");
        router->persistState();
    }

    delete router;
    return (0);
}
