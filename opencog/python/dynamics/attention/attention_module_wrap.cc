#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>

#include "attention_module_wrap.h"
#include "ForgettingAgent_wrap.h"

using namespace boost::python;

class attention_scope_t {};

void init_attention_module_py()
{
    // Setup scope.
    attention_scope =
        class_<attention_scope_t>("attention");
    scope within(attention_scope);

    init_ForgettingAgent_py();
}
