/*
 * examples/extending/DerivedMain.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include "DerivedCogServer.h"

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

using namespace opencog;

int main(int argc, char *argv[])
{
    // setup global logger
    logger().setLevel(Logger::DEBUG);
    logger().setPrintToStdoutFlag(config().get_bool("LOG_TO_STDOUT"));

    // the *first* call to the opencog::server() global function passes a
    // CogServerFactory function pointer that returns an instance of the
    // derived server. This will not work if we cannot guarantee that this
    // will be the *very first* opencog::server() call.
    server(DerivedCogServer::derivedCreateInstance);
    (static_cast<DerivedCogServer&>(server())).serverLoop();
    return 0;
}
