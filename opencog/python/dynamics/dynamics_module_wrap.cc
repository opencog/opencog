#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>

#include "dynamics_module_wrap.h"
#include "attention_module_wrap.h"

using namespace boost::python;

class dynamics_scope_t {};

void init_dynamics_module_py()
{
    // Setup scope.
    dynamics_scope =
        class_<dynamics_scope_t>("dynamics");
    scope within(dynamics_scope);

    init_attention_module_py();
}
