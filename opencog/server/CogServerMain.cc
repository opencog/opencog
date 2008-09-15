/*
 * opencog/server/CogServerMain.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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

#include <opencog/query/QueryProcessor.h>
#include <opencog/nlp/wsd/WordSenseProcessor.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

using namespace opencog;

static void usage(char* progname)
{
    cerr << "Usage: " << progname << " [-c <config-file>]\n\n";
}

int main(int argc, char *argv[])
{
    // check command line
    if ((argc == 1) || ((argc == 3) && (strcmp(argv[1], "-c") == 0))) {
        // load the configuration from file if a filename was supplied on the command line
        if (argc == 3) {
            try {
                config().load(argv[2]);
            } catch (RuntimeException &e) {
                cerr << e.getMessage() << std::endl;
                return 1;
            }
        }

        // setup global logger
        logger().setFilename(config()["LOG_FILE"]);
        logger().setLevel(Logger::getLevelFromString(config()["LOG_LEVEL"]));
        logger().setPrintToStdoutFlag(config().get_bool("LOG_TO_STDOUT"));

        // cheapo hack to get the query processory up and running.
        // It would be more correct to have this loaded by a script
        // executed from the command shell, rather than hard-coded
        // into the C++ source.
        CogServer& cogserver = static_cast<CogServer&>(server());
        cogserver.plugInInputHandler(new QueryProcessor());
        cogserver.plugInInputHandler(new WordSenseProcessor());

        // enable the network server and run the server's main loop
        cogserver.enableNetworkServer();
        cogserver.serverLoop();
    } else {
        usage(argv[0]);
        return 1;
    }
}
