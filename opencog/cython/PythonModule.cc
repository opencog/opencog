#include "PythonModule.h"
#include <signal.h>

#include "agent_finder_api.h"

using std::vector;
using std::string;

using namespace opencog;

DECLARE_MODULE(PythonModule);

PythonModule::PythonModule() : Module()
{
    // We need to back up the SIGINT handler otherwise Python
    // steals Ctrl-C and we can't easily kill the CogServer when it runs in the
    // foreground
    __sighandler_t prev;
    prev = signal(SIGINT, SIG_DFL);

    logger().info("[PythonModule] constructor");
    // Start up Python
    Py_Initialize();
    PyEval_InitThreads();

    // Now that Python is initialised, restore the SIGINT handler
    prev = signal(SIGINT, prev);

    // Initialise the agent_finder module which helps with the Python side of
    // things
    if (import_agent_finder() == -1) {
        PyErr_Print();
        logger().error("[PythonModule] Failed to load helper python module");
    }
    do_load_py_register();
}

PythonModule::~PythonModule()
{
    logger().info("[PythonModule] destructor");
    do_load_py_unregister();
    Py_Finalize();
}

void PythonModule::init()
{
    logger().info("[PythonModule] init");
}

std::string PythonModule::do_load_py(Request *dummy, std::list<std::string> args)
{
    //AtomSpace *space = CogServer::getAtomSpace();
    if (args.size() == 0) return "Please specify Python module to load.";
    requests_and_agents_t thingsInModule;
    thingsInModule = load_module(args.front());
    foreach(std::string s, thingsInModule.agents) {
        std::cout << "I found agent with name " << s << std::endl;
    }
    // TODO save the py_module somewhere
    // ...
    // TODO register the agents
    //CogServer& cogserver = static_cast<CogServer&>(server());
    // How do we initialise a mind agent with a factory that will
    // load with given Python class
    //cogserver.registerAgent(PyMindAgent::info().id, &forgettingFactory);
    // TODO return info on what requests and mindagents were found
    return "Loaded python module";
}

