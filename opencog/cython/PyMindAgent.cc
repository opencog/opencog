#include "PyMindAgent.h"

using namespace opencog;

PyMindAgent::PyMindAgent(const std::string& moduleName, const std::string& className, PyObject* o)
{
    pyagent=o;
}

PyMindAgent::~PyMindAgent()
{
    // decrement python object reference counter
    Py_DECREF(pyagent);
}

void PyMindAgent::run(CogServer* server)
{
    //PyObject_CallMethod(pyagent,"__run_wrap",Py_BuildValue("
    PyObject_CallMethod(pyagent,"run","s","blah");
    PyErr_Print();

}

