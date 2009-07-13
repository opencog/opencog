#include "BaseServer_wrap.h"
#include <opencog/server/BaseServer.h>
#include <opencog/atomspace/AtomSpace.h>

#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
//#include <boost/python/return_by_value.hpp>
//#include <boost/python/reference_existing_object.hpp>
#include <boost/python/manage_new_object.hpp>

using namespace opencog;
using namespace boost::python;

void init_BaseServer_py()
{
    class_<BaseServer>("BaseServer")
        .def("getAtomSpace", &BaseServer::getAtomSpace,
            //return_value_policy<return_by_value>())
            //return_value_policy<reference_existing_object>())
            return_value_policy<manage_new_object>())
        .staticmethod("getAtomSpace")
    ;
}
