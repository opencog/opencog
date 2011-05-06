#include "PythonModule.h"

#include "agent_finder_api.h"

using namespace opencog;

DECLARE_MODULE(PythonModule);

PythonModule::PythonModule() : Module()
{
    logger().info("[PythonModule] constructor");
    do_load_py_register();
}

PythonModule::~PythonModule()
{
    logger().info("[PythonModule] destructor");
    do_load_py_unregister();
}

void PythonModule::init()
{
    logger().info("[PythonModule] init");
}

std::string PythonModule::do_load_py(Request *dummy, std::list<std::string> args)
{
    //AtomSpace *space = CogServer::getAtomSpace();
    if (args.size() == 0) return "Please specify Python module to load.";
    PyObject* py_module = load_module(args.front());
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

