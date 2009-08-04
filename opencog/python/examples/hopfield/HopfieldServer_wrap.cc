#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/manage_new_object.hpp>
#include "HopfieldServer_wrap.h"
#include <examples/hopfield/HopfieldServer.h>
#include <opencog/server/CogServer.h>

using namespace boost::python;
using namespace opencog;

void init_HopfieldServer_py()
{
    class_<HopfieldServer, bases<CogServer> >("HopfieldServer")
        .def("derivedCreateInstance",
            &HopfieldServer::derivedCreateInstance,
            return_value_policy<manage_new_object>())
        .staticmethod("derivedCreateInstance")
        .def("init", &HopfieldServer::init)
        .def("encodePattern", &HopfieldServer::encodePattern)
        .def("totalEnergy", &HopfieldServer::totalEnergy)
        .def("retrievePattern", &HopfieldServer::retrievePattern)
    ;
}
