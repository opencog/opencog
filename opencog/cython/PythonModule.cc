#include "PythonModule.h"

#include "agent_finder_api.h"

using std::vector;
using std::string;

using namespace opencog;

DECLARE_MODULE(PythonModule);

PythonModule::PythonModule() : Module()
{
    logger().info("[PythonModule] constructor");
    // Start up Python (this init method skips registering signal handlers)
    Py_InitializeEx(0);
    PyEval_InitThreads();
    PyRun_SimpleString("import sys; print sys.path\n");
    //PyRun_SimpleString("import sys; sys.path.insert(0,'')\n");

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
    std::ostringstream oss;
    if (thingsInModule.agents.size() > 0) {
        bool first = true;
        oss << "Python MindAgents found: ";
        foreach(std::string s, thingsInModule.agents) {
            if (!first) {
                oss << ", ";
                first = false;
            }
            oss << s;
        }
        oss << ".";
    } else {
        oss << "No subclasses of opencog.MindAgent found.";
    }
    // TODO save the py_module somewhere
    // ...
    // TODO register the agents
    //CogServer& cogserver = static_cast<CogServer&>(server());
    // How do we initialise a mind agent with a factory that will
    // load with given Python class
    //cogserver.registerAgent(PyMindAgent::info().id, &forgettingFactory);
    // TODO return info on what requests and mindagents were found
    return oss.str();

}

