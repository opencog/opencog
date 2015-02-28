/*
 * opencog/cython/PythonModule.cc
 *
 * Copyright (C) 2013 by OpenCog Foundation
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
#include "PyIncludeWrapper.h"

#include <boost/filesystem/operations.hpp>

#include <opencog/util/Config.h>
#include <opencog/util/misc.h>
#include <opencog/atomspace/AtomSpace.h>


#include "PyMindAgent.h"
#include "PyRequest.h"
#include "PythonEval.h"
#include "PythonModule.h"

#include "opencog/agent_finder_types.h"
#include "opencog/agent_finder_api.h"

using std::vector;
using std::string;

using namespace opencog;

//#define DPRINTF printf
#define DPRINTF(...)

DECLARE_MODULE(PythonModule);

// Factories

Agent* PythonAgentFactory::create(CogServer& cs) const
{
    logger().info() << "Creating python agent " << _pySrcModuleName << "." << _pyClassName;
    PyMindAgent* pma = new PyMindAgent(cs, _pySrcModuleName, _pyClassName);
    return pma;
}

Request* PythonRequestFactory::create(CogServer& cs) const
{
    logger().info() << "Creating python request " << _pySrcModuleName << "." << _pyClassName;
    PyRequest* pma = new PyRequest(cs, _pySrcModuleName, _pyClassName, _cci);
    return pma;
}

// ------

PythonModule::PythonModule(CogServer& cs) : Module(cs)
{
}

static bool already_loaded = false;

PythonModule::~PythonModule()
{
    logger().info("[PythonModule] destructor");
    unregisterAgentsAndRequests();
    do_load_py_unregister();

    for (PythonAgentFactory* af : _agentFactories) delete af;
    for (PythonRequestFactory* rf : _requestFactories) delete rf;

    already_loaded = false;
}

bool PythonModule::unregisterAgentsAndRequests()
{
    // Requires GIL
    for (std::string s : _agentNames) {
        DPRINTF("Deleting all instances of %s\n", s.c_str());
        _cogserver.unregisterAgent(s);
    }
    for (std::string s : _requestNames) {
        DPRINTF("Unregistering requests of id %s\n", s.c_str());
        _cogserver.unregisterRequest(s);
    }

    return true;
}

void PythonModule::init()
{
    // Avoid hard crash if already loaded.
    if (already_loaded) return;
    already_loaded = true;

    logger().info("[PythonModule] Initialising Python CogServer module.");

    // Make sure that Python has been properly initialized.
    if (not Py_IsInitialized() || not PyEval_ThreadsInitialized()) {
            throw opencog::RuntimeException(TRACE_INFO,
                    "Python not initialized, missing global_python_init()");
    }

    // NOTE: Even though the Cython docs do not say that you need to call this
    // more than once, you need to call the import functions in each separate
    // shared library that accesses Cython defined api. If you don't then you
    // get a crash when you call an api function. 
    import_opencog__agent_finder();

    if (config().has("PYTHON_PRELOAD")) preloadModules();
    do_load_py_register();

}

bool PythonModule::preloadModules()
{
    // requires GIL
    std::vector<std::string> pythonmodules;
    tokenize(config()["PYTHON_PRELOAD"], std::back_inserter(pythonmodules), ", ");
    for (std::vector<std::string>::const_iterator it = pythonmodules.begin();
         it != pythonmodules.end(); ++it) {
        std::list<std::string> args;
        args.push_back(*it);
        logger().info("[PythonModule] Preloading python module " + *it);
        std::string result = do_load_py(NULL,args);
        logger().info("[PythonModule] " + result);
    }
    return true;
}

/// do_load_py -- load python code, given a file name. (Implements the loadpy command)
///
/// It is expected that the file contains a python module. The module
/// should implement either a mind-agent, or it should contain a 'request'
/// (shell command, written in python). Mind agents must inherit from
/// the python class opencog.cogserver.MindAgent, while requests/commnds
/// must inherit from opencog.cogserver.Request
//
std::string PythonModule::do_load_py(Request *dummy, std::list<std::string> args)
{
    if (args.empty()) return "Please specify Python module to load.";
    std::string moduleName = args.front();

    std::ostringstream oss;
    if (moduleName.substr(moduleName.size()-3,3) == ".py") {
        oss << "Warning: Python module name should be "
            << "passed without .py extension" << std::endl;
        moduleName.replace(moduleName.size()-3,3,"");
    }

    requests_and_agents_t thingsInModule = load_req_agent_module(moduleName);
    if (thingsInModule.err_string.size() > 0) {
        return thingsInModule.err_string;
    }

    // If there are agents, load them.
    if (thingsInModule.agents.size() > 0) {
        bool first = true;
        oss << "Python MindAgents found: ";
        for (std::string s : thingsInModule.agents) {
            if (!first) {
                oss << ", ";
                first = false;
            }

            // Register agent with cogserver using dotted name:
            // module.AgentName
            std::string dottedName = moduleName + "." + s;

            // register the agent with a custom factory that knows how to
            // instantiate new Python MindAgents
            PythonAgentFactory* afact = new PythonAgentFactory(moduleName,s);
            _agentFactories.push_back(afact);
            _cogserver.registerAgent(dottedName, afact);

            // save a list of Python agents that we've added to the CogServer
            _agentNames.push_back(dottedName);
            oss << s;
        }
        oss << "." << std::endl;
    } else {
        oss << "No subclasses of opencog.cogserver.MindAgent found.\n";
    }

    // Now load the commands/requests, if any.
    if (thingsInModule.requests.size() > 0) {
        bool first = true;
        oss << "Python Requests found: ";
        for (size_t i=0; i< thingsInModule.requests.size(); i++) {
            std::string s = thingsInModule.requests[i];
            std::string short_desc = thingsInModule.req_summary[i];
            std::string long_desc = thingsInModule.req_description[i];
            bool is_shell = thingsInModule.req_is_shell[i];

            // CogServer commands in Python
            // Register request with cogserver using dotted name: module.RequestName
            std::string dottedName = moduleName + "." + s;
            // register the agent with a custom factory that knows how to
            PythonRequestFactory* fact =
                new PythonRequestFactory(moduleName, s, short_desc, long_desc, is_shell);
            _requestFactories.push_back(fact);
            _cogserver.registerRequest(dottedName, fact);
            // save a list of Python agents that we've added to the CogServer
            _requestNames.push_back(dottedName);

            if (!first) { oss << ", "; first = false; }
            oss << s;
        }
        oss << ".";
    } else {
        oss << "No subclasses of opencog.cogserver.Request found.\n";
    }

    // Return info on what requests and mindagents were found
    // This gets printed out to the user at the shell prompt.
    return oss.str();
}

