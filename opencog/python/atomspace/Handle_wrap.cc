#include "Handle_wrap.h"
#include "Handle.h"
#include <boost/python/class.hpp>
#include <boost/lexical_cast.hpp>

using namespace opencog;
using namespace boost::python;

void init_Handle_py()
{
    class_<Handle>("Handle", no_init)
        .def(init<>())
        .def(init<const Handle&>())
        .def_readonly("UNDEFINED", &Handle::UNDEFINED)
        .def("str", &Handle::str)
        .def("__str__", boost::lexical_cast<std::string, Handle const&>)
    ;
}
