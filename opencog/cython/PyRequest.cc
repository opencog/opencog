#include "PyRequest.h"

#include <opencog/server/CogServer.h>

#include "agent_finder_types.h"
#include "agent_finder_api.h"

using namespace opencog;

PyRequest::PyRequest(const std::string& _moduleName, const std::string& _className)
{
    // XXX TODO FIXME -- obtain the request description from the python code.
    cci = new RequestClassInfo(
          _moduleName + _className,
          "Some request implemented in python. TODO provide short description.",
          "TODO provide a long description including parameter types."
    );

    PyGILState_STATE gstate = PyGILState_Ensure(); 

    import_agent_finder();
    moduleName = _moduleName;
    className = _className;
    // call out to our helper module written in cython
    pyrequest = instantiate_request(moduleName, className, this);

    PyGILState_Release(gstate); 

    if (pyrequest == Py_None)
        logger().error() << "Error creating python request "
                         << _moduleName << "." << _className;
    else
        logger().info() << "Created python request "
                        << _moduleName << "." << _className;
}

PyRequest::~PyRequest()
{
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure(); 
    // decrement python object reference counter
    Py_DECREF(pyrequest);
    PyGILState_Release(gstate); 
    delete cci;
}

bool PyRequest::execute()
{
    CogServer &s = cogserver();
    string result = run_request(pyrequest, _parameters, &s.getAtomSpace());
    // errors only with result is not empty... && duplicate errors are not reported.
    if (result.size() > 0 && result != last_result) {
        // Any string returned is a traceback
        logger().error("[%s::execute] Python Exception:\n%s",
                this->info().id.c_str(),result.c_str());
        last_result = result;
        return false;
    }
    last_result = result;
    return true;
}

