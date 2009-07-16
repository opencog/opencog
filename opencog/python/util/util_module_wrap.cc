#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>

#include "util_module_wrap.h"
#include "Logger_wrap.h"

using namespace boost::python;

class util_scope_t {};

void init_util_module_py()
{
    // Setup scope.
    util_scope =
        class_<util_scope_t>("util");
    scope within(util_scope);

    init_Logger_py();
}
