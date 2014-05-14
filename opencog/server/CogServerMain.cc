/*
 * opencog/server/CogServerMain.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <getopt.h>
#include <langinfo.h>
#include <locale.h>
#include <signal.h>
#include <string.h>

#include <string>
#include <thread>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/foreach.hpp>

#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/files.h>
#include <opencog/util/misc.h>

using namespace opencog;
using namespace std;

static const char* DEFAULT_CONFIG_FILENAME = "opencog.conf";
static const char* DEFAULT_CONFIG_PATHS[] = 
{
    // Search order for the config file:
    "lib",       // First, we look in the build directory (cmake puts it here)
    "../lib",    // Next, we look at the source directory
    CONFDIR,     // Next, the install directory
#ifndef WIN32
    "/etc",      // Finally, in the standard ssytem directory.
#endif // !WIN32
    NULL
};

static void usage(const char* progname)
{
    std::cerr << "Usage: " << progname << " [[-c <config-file>]..] [[-DOPTION=\"VALUE\"]..]\n\n";
    std::cerr << "Each config file is loaded sequentially, with the values in \n"
        << " later files overwriting earlier. Then each singular option overrides \n" 
        << " options in config files. " << std::endl;
}

// Catch and report sigsegv
void sighand(int sig)
{
    logger().setPrintToStdoutFlag(true);
    logger().error() << "Caught signal " << sig << " (" << strsignal(sig)
        << ") on thread " << std::this_thread::get_id();
    logger().flush();
    sleep(3);
    exit(1);
}

int main(int argc, char *argv[])
{
    // Get the locale from the environment... 
    // Perhaps we should someday get it from the config file ???
    setlocale(LC_ALL, "");

    // Check to make sure the current locale is UTF8; if its not,
    // then force-set this to the english utf8 locale 
    const char * codeset = nl_langinfo(CODESET);
    if (!strstr(codeset, "UTF") && !strstr(codeset, "utf"))
    {
       fprintf(stderr,
           "%s: Warning: locale %s was not UTF-8; force-setting to en_US.UTF-8\n",
            argv[0], codeset);
       setlocale(LC_CTYPE, "en_US.UTF-8");
    }

    static const char *optString = "c:D:h";
    int c = 0;
    vector<string> configFiles;
    vector< pair<string,string> > configPairs;
    string progname = argv[0];

    // parse command line
    while (1) {
        c = getopt (argc, argv, optString);
        /* Detect end of options */
        if (c == -1) {
            break;
        } else if (c == 'c') {
            configFiles.push_back(optarg);
        } else if (c == 'D') {
            // override all previous options, e.g.
            // -DLOG_TO_STDOUT=TRUE
            string text = optarg; 
            string value, optionName;
            vector<std::string> strs;
            boost::split(strs, text, boost::is_any_of("=:"));
            optionName = strs[0];
            if (strs.size() > 2) {
                // merge end tokens if more than one separator found
                for (uint i = 1; i < strs.size(); i++)
                    value += strs[i];
            } else if (strs.size() == 1) {
                std::cerr << "No value given for option " << strs[0] << endl;
            } else {
                value = strs[1];
            }
            configPairs.push_back( pair<string,string>(optionName, value) );
        } else {
            // unknown option (or help)
            usage(progname.c_str());
            if (c == 'h')
                exit(0);
            else
                exit(1);
        }

    }

    if (configFiles.size() == 0) {
        // search for configuration file on default locations
        for (int i = 0; DEFAULT_CONFIG_PATHS[i] != NULL; ++i) {
            boost::filesystem::path configPath(DEFAULT_CONFIG_PATHS[i]);
            configPath /= DEFAULT_CONFIG_FILENAME;
            if (boost::filesystem::exists(configPath)) {
                cerr << "Using default config at " << configPath.string() << endl;
                configFiles.push_back(configPath.string());
            }
        }
    }
    config().reset();
    if (configFiles.size() == 0) {
        cerr << "No config files could be found!" << endl;
        exit(-1);
    }
    // Each config file sequentially overwrites the next
    BOOST_FOREACH (string configFile, configFiles) {
        try {
            config().load(configFile.c_str(), false);
            break;
        } catch (RuntimeException &e) {
            std::cerr << e.getMessage() << std::endl;
            exit(1);
        }
    }
    // Each specific option
    pair<string,string> optionPair;
    BOOST_FOREACH (optionPair, configPairs) {
        //cerr << optionPair.first << " = " << optionPair.second << endl;
        config().set(optionPair.first, optionPair.second);
    }

    // setup global logger
    logger().setFilename(config()["LOG_FILE"]);
    logger().setLevel(Logger::getLevelFromString(config()["LOG_LEVEL"]));
    logger().setBackTraceLevel(Logger::getLevelFromString(config()["BACK_TRACE_LOG_LEVEL"]));
    logger().setPrintToStdoutFlag(config().get_bool("LOG_TO_STDOUT"));
    //logger().setLevel(Logger::DEBUG);

    // Start catching signals
    signal(SIGSEGV, sighand);
    signal(SIGBUS, sighand);
    signal(SIGFPE, sighand);
    signal(SIGILL, sighand);
    signal(SIGABRT, sighand);
    signal(SIGTRAP, sighand);
    signal(SIGQUIT, sighand);
    
    CogServer& cogserve = cogserver();

    // Open database *before* loading modules, since the modules
    // might create atoms, and we can't have that happen until 
    // storage is open, as otherwise, there will be handle conflicts.
    cogserve.openDatabase(); 

    // Load modules specified in config
    cogserve.loadModules(DEFAULT_MODULE_PATHS); 
    cogserve.loadSCMModules(DEFAULT_MODULE_PATHS);

    // enable the network server and run the server's main loop
    cogserve.enableNetworkServer();
    cogserve.serverLoop();
    exit(0);
}
