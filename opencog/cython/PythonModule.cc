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

static const char* DEFAULT_PYTHON_MODULE_PATHS[] =
{
    PROJECT_BINARY_DIR"/opencog/cython", // bindings
    PROJECT_SOURCE_DIR"/opencog/python", // opencog modules written in python
    PROJECT_SOURCE_DIR"/tests/cython",   // for testing
    DATADIR"/python",                    // install directory
    #ifndef WIN32
    "/usr/local/share/opencog/python",
    "/usr/share/opencog/python",
    #endif // !WIN32
    NULL
};

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

    // Start up Python (this init method skips registering signal handlers)
    if(!Py_IsInitialized())
        Py_InitializeEx(0);
    if(!PyEval_ThreadsInitialized())
        PyEval_InitThreads();

    // Add our module directories to the Python interprator's path
    const char** config_paths = DEFAULT_PYTHON_MODULE_PATHS;

    PyObject* sysPath = PySys_GetObject((char*)"path");

    // Default paths for python modules
    for (int i = 0; config_paths[i] != NULL; ++i) {
        boost::filesystem::path modulePath(config_paths[i]);
        if (boost::filesystem::exists(modulePath))
            PyList_Append(sysPath, PyString_FromString(modulePath.string().c_str()));
        //            applier.addSysPath(modulePath.string());

    }

    // Add custom paths for python modules from the config file if available
    if (config().has("PYTHON_EXTENSION_DIRS")) {
        std::vector<std::string> pythonpaths;
        // For debugging current path
        tokenize(config()["PYTHON_EXTENSION_DIRS"], std::back_inserter(pythonpaths), ", ");
        for (std::vector<std::string>::const_iterator it = pythonpaths.begin();
             it != pythonpaths.end(); ++it) {
            boost::filesystem::path modulePath(*it);
            if (boost::filesystem::exists(modulePath)) {
                PyList_Append(sysPath, PyString_FromString(modulePath.string().c_str()));
                //                applier.addSysPath(modulePath.string());
            } else {
                logger().error("PythonEval::%s Could not find custom python extension directory: %s ",
                               __FUNCTION__,
                               (*it).c_str()
                               );
            }
        }
    }


    if (import_agent_finder() == -1) {
        PyErr_Print();
        throw RuntimeException(TRACE_INFO,"[PythonModule] Failed to load helper python module");
    }
    // For debugging the python path:
    logger().debug("Python sys.path is: " + get_path_as_string());


    PythonEval& applier = PythonEval::instance();

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

