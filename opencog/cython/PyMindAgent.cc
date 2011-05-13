
#include "PyMindAgent.h"

#include <opencog/server/CogServer.h>

#include "agent_finder_types.h"
#include "agent_finder_api.h"

using namespace opencog;

PyMindAgent::PyMindAgent(const std::string& _moduleName, const std::string& _className)
{
    // TODO move the class instantiation code here
    import_agent_finder();
    moduleName = _moduleName;
    className = _className;
    // call out to our helper module written in cython
    pyagent = instantiate_agent(moduleName, className, this);
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
    // decrement python object reference counter
    Py_DECREF(pyagent);
    // TODO: check for exceptions and print error
    // PyErr_Print();
}

void PyMindAgent::run(CogServer* server)
{
    // clear error
    run_agent(pyagent,server->getAtomSpace());
    // TODO: check for exceptions and print error
    // PyErr_Print();
}

