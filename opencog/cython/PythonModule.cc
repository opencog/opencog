#include "PythonModule.h"

#include "agent_finder_api.h"

#include <boost/filesystem/operations.hpp>

#include <opencog/util/Config.h>
#include <opencog/util/misc.h>
#include <opencog/util/foreach.h>

using std::vector;
using std::string;

using namespace opencog;

//#define DPRINTF printf
#define DPRINTF(...)

DECLARE_MODULE(PythonModule);

// Factories

Agent* PythonAgentFactory::create() const {
    PyMindAgent* pma = new PyMindAgent(pySrcModuleName,pyClassName);
    return pma;
}

Request* PythonRequestFactory::create() const {
    PyRequest* pma = new PyRequest(pySrcModuleName,pyClassName);
    return pma;
}

// ------

PythonModule::PythonModule() : Module()
{
}

static bool already_loaded = false;

PythonModule::~PythonModule()
{
    logger().info("[PythonModule] destructor");
    unregisterAgentsAndRequests();
    do_load_py_unregister();
    Py_Finalize();
    already_loaded = false;
}

bool PythonModule::unregisterAgentsAndRequests()
{
    // Requires GIL
    foreach (std::string s, agentNames) {
        DPRINTF("Deleting all instances of %s\n", s.c_str());
        cogserver().unregisterAgent(s);
    }
    foreach (std::string s, requestNames) {
        DPRINTF("Unregistering requests of id %s\n", s.c_str());
        cogserver().unregisterRequest(s);
    }

    return true;
}

void PythonModule::init()
{
    // Avoid hard crash if already loaded.
    if (already_loaded) return;
    already_loaded = true;

    logger().info("[PythonModule] Initialising Python CogServer module.");

    PythonEval::instance();

    if (import_agent_finder() == -1) {
        PyErr_Print();
        throw RuntimeException(TRACE_INFO,"[PythonModule] Failed to load helper python module");
    }
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

std::string PythonModule::do_load_py(Request *dummy, std::list<std::string> args)
{
    //AtomSpace *space = CogServer::getAtomSpace();
    if (args.size() == 0) return "Please specify Python module to load.";
    requests_and_agents_t thingsInModule;
    std::string moduleName = args.front();
    std::ostringstream oss;
    if (moduleName.substr(moduleName.size()-3,3) == ".py") {
        oss << "Warning: Python module name should be "
            << "passed without .py extension" << std::endl;
        moduleName.replace(moduleName.size()-3,3,"");
    }
    thingsInModule = load_module(moduleName);
    if (thingsInModule.err_string.size() > 0) {
        return thingsInModule.err_string;
    }
    if (thingsInModule.agents.size() > 0) {
        bool first = true;
        oss << "Python MindAgents found: ";
        DPRINTF("Python MindAgents found: ");
        foreach(std::string s, thingsInModule.agents) {
            if (!first) {
                oss << ", ";
                first = false;
            }

            // Register agent with cogserver using dotted name:
            // module.AgentName
            std::string dottedName = moduleName + "." + s;

            // register the agent with a custom factory that knows how to
            // instantiate new Python MindAgents
            cogserver().registerAgent(dottedName, new PythonAgentFactory(moduleName,s));

            // save a list of Python agents that we've added to the CogServer
            agentNames.push_back(dottedName);
            oss << s;
            DPRINTF("%s ", s.c_str());
        }
        oss << "." << std::endl;
        DPRINTF("\n");
    } else {
        oss << "No subclasses of opencog.cogserver.MindAgent found.\n";
        DPRINTF("No subclasses of opencog.cogserver.MindAgent found.\n");
    }
    if (thingsInModule.requests.size() > 0) {
        bool first = true;
        oss << "Python Requests found: ";
        DPRINTF("Python Requests found: ");
        foreach(std::string s, thingsInModule.requests) {
            if (!first) {
                oss << ", ";
                first = false;
            }
            // CogServer requests in Python
            // Register request with cogserver using dotted name: module.RequestName
            std::string dottedName = moduleName + "." + s;
            // register the agent with a custom factory that knows how to
            cogserver().registerRequest(dottedName, new PythonRequestFactory(moduleName,s));
            // save a list of Python agents that we've added to the CogServer
            requestNames.push_back(dottedName);
            oss << s;
            DPRINTF("%s ", s.c_str());
        }
        oss << ".";
    } else {
        oss << "No subclasses of opencog.cogserver.Request found.\n";
        DPRINTF("No subclasses of opencog.cogserver.Request found.\n");
    }

    // return info on what requests and mindagents were found
    return oss.str();
}

