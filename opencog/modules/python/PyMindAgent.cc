
#include <opencog/modules/python/PyMindAgent.h>

#include <opencog/server/CogServer.h>

#include <opencog/cython/opencog/agent_finder_types.h>
#include <opencog/cython/opencog/agent_finder_api.h>

using namespace opencog;

PyMindAgent::PyMindAgent(CogServer& cs,
                         const std::string& _moduleName,
                         const std::string& _className) :
    Agent(cs)
{
    moduleName = _moduleName;
    className = _className;

    // NOTE: You need to call the import functions in each separate
    // shared library that accesses Cython defined api. If you don't
    // then you get a crash when you call an api function.
    import_opencog__agent_finder();

    // Call out to our helper module written in cython. NOTE: Cython api calls
    // defined "with gil" can be called without grabbing the GIL manually.
    pyagent = instantiate_agent(moduleName, className, this);
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

    // Decrement python object reference counter
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

