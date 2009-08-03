#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>

#include "server_module_wrap.h"
#include "Agent_wrap.h"
#include "Factory_wrap.h"
#include "BaseServer_wrap.h"
#include "CogServer_wrap.h"
#include "Request_wrap.h"

using namespace boost::python;

class server_scope_t {};

void init_server_module_py()
{
    // Setup scope.
    server_scope =
        class_<server_scope_t>("server");
    scope within(server_scope);

    init_BaseServer_py();
    init_CogServer_py();
    init_Agent_py();
    init_Factory_py();
    init_Request_py();
}
