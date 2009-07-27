#include "TLB_wrap.h"
#include "TLB.h"
#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/manage_new_object.hpp>

using namespace opencog;
using namespace boost::python;

void init_TLB_py()
{
    class_<TLB>("TLB", no_init)
        .def("getAtom", &TLB::getAtom,
            return_value_policy<manage_new_object>())
        .staticmethod("getAtom")
    ;
}
