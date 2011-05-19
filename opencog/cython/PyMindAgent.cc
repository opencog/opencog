
#include "PyMindAgent.h"

#include <opencog/server/CogServer.h>

#include "agent_finder_types.h"
#include "agent_finder_api.h"

using namespace opencog;

PyMindAgent::PyMindAgent(const std::string& _moduleName, const std::string& _className)
{
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure(); 
    import_agent_finder();
    moduleName = _moduleName;
    className = _className;
    // call out to our helper module written in cython
    pyagent = instantiate_agent(moduleName, className, this);
    PyGILState_Release(gstate); 
    if (pyagent == Py_None)
        throw RuntimeException(TRACE_INFO, "Error creating Python MindAgent");
}

const ClassInfo& PyMindAgent::classinfo() const
{ 
    static const ClassInfo _ci("opencog::PyMindAgent(" + moduleName+"."+className+")");
    return _ci;
}

PyMindAgent::~PyMindAgent()
{
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure(); 
    // decrement python object reference counter
    Py_DECREF(pyagent);
    PyGILState_Release(gstate); 
}

void PyMindAgent::run(CogServer* server)
{
    string result = run_agent(pyagent,server->getAtomSpace());
    // errors only with result is not empty... && duplicate errors are not reported.
    if (result.size() > 0 && result != last_result) {
        // Any string returned is a traceback
        logger().error("[%s::run] Python Exception:\n%s",
                this->classinfo().id.c_str(),result.c_str());
    }
    last_result = result;
}

