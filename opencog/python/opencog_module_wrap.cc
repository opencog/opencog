#include <boost/python/module.hpp>
#include <boost/python/scope.hpp>

#include "opencog_module_wrap.h"
#include "atomspace_module_wrap.h"
#include "server_module_wrap.h"
#include "dynamics_module_wrap.h"
#include "util_module_wrap.h"
#include "hopfield_module_wrap.h"

void init_opencog_py()
{
    init_atomspace_module_py();
    init_server_module_py();
    init_dynamics_module_py();
    init_util_module_py();
//    init_hopfield_module_py();
}

BOOST_PYTHON_MODULE(opencog)
{
    using namespace boost::python;

    // Setup the atomspace module info.
    scope().attr("__doc__") = "Wrapper for the OpenCog framework API";
    scope().attr("__name__") = "opencog";
#ifndef DEBUG
    scope().attr("__version__") = "some version (debug)";
#else
    scope().attr("__version__") = "some version (release)";
#endif
    scope().attr("__author__") = "David Kilgore <davidpkilgore@gmail.com>";
    scope().attr("__credits__") =
        "Based on the following libraries:\n"
        "Boost Python v2 (http://www.boost.org)";

    init_opencog_py();
}
