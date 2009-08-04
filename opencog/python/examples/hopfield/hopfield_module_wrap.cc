#include <boost/python/module.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/object.hpp>
#include <boost/python/class.hpp>

#include "hopfield_module_wrap.h"
#include "HopfieldServer_wrap.h"
#include "HopfieldOptions_wrap.h"

using namespace boost::python;

class hopfield_scope_t {};
object hopfield_scope;

void init_hopfield_module_py()
{
    // Setup scope.
    hopfield_scope = class_<hopfield_scope_t>("hopfield");
    scope within(hopfield_scope);

    init_HopfieldServer_py();
    init_HopfieldOptions_py();
}
