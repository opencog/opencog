#include "ClassServer_wrap.h"
#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/manage_new_object.hpp>
//#include <boost/python/overloads.hpp>
#include <opencog/atomspace/ClassServer.h>

using namespace opencog;
using namespace boost::python;

//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addLink_overloads, addLink, 1, 11)

void init_ClassServer_py()
{
    class_<ClassServer, boost::noncopyable>("ClassServer", no_init)
        .def("createInstance", &ClassServer::createInstance,
            return_value_policy<manage_new_object>())
        .staticmethod("createInstance")
        .def("getType", &ClassServer::getType)
    ;
}
