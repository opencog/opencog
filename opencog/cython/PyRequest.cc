#include "PyRequest.h"

#include <opencog/server/CogServer.h>

#include "agent_finder_types.h"
#include "agent_finder_api.h"

using namespace opencog;

PyRequest::PyRequest(CogServer& cs,
                     const std::string& moduleName, const std::string& className,
                     RequestClassInfo* cci) :
    Request(cs)
{
    _cci = cci;

    PyGILState_STATE gstate = PyGILState_Ensure(); 

    import_agent_finder();
    _moduleName = moduleName;
    _className = className;
    // call out to our helper module written in cython
    _pyrequest = instantiate_request(moduleName, className, this);

    PyGILState_Release(gstate); 

    if (_pyrequest == Py_None)
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
    Py_DECREF(_pyrequest);
    PyGILState_Release(gstate); 
}

bool PyRequest::execute()
{
    string result = run_request(_pyrequest, _parameters, &_cogserver.getAtomSpace());
    // errors only with result is not empty... && duplicate errors are not reported.
    if (result.size() > 0 && result != _last_result) {
        // Any string returned is a traceback
        logger().error("[%s::execute] Python Exception:\n%s",
                this->info().id.c_str(),result.c_str());
        _last_result = result;
        return false;
    }
    _last_result = result;
    return true;
}

