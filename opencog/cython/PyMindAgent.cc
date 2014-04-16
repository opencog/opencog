
#include "PyMindAgent.h"

#include <opencog/server/CogServer.h>

#include "opencog/agent_finder_types.h"
#include "opencog/agent_finder_api.h"

using namespace opencog;

PyMindAgent::PyMindAgent(CogServer& cs,
                         const std::string& _moduleName,
                         const std::string& _className) :
    Agent(cs)
{
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure(); 
    import_opencog__agent_finder();
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

static bool in_fini = false;
#if __GNUC__
static __attribute__ ((destructor (65535))) void pyagent_fini(void)
{
    in_fini = true;
}
#endif

PyMindAgent::~PyMindAgent()
{
    // Do nothing if we are in finalizer ... because at this point,
    // python is probably already dead, and doing the below will just
    // segfault.
    if (in_fini) return;

    // Still fails XXX don't know how to fix this...
    // Maybe we can ask python if its been finalized?
    return;

    // decrement python object reference counter
    PyGILState_STATE gstate = PyGILState_Ensure(); 
    Py_DECREF(pyagent);
    PyGILState_Release(gstate); 
}

void PyMindAgent::run()
{
    std::string result = run_agent(pyagent, &_cogserver.getAtomSpace());
    
    // run_agent only returns a string if it is propagating an exception
    if (result.size() > 0) {
        throw RuntimeException(result.c_str(),
                               "PyMindAgent triggered runtime exception.");
    }
}

