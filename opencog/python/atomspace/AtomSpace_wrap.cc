#include "AtomSpace_wrap.h"
#include "AtomSpace.h"
#include "SpaceServerContainer.h"
#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/reference_existing_object.hpp>
#include <boost/python/overloads.hpp>

using namespace opencog;
using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addNode_overloads, addNode, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addRealAtom_overloads, addRealAtom, 1, 2)
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addLink_overloads, addLink, 1, 11)

void init_AtomSpace_py()
{
    class_<AtomSpace, bases<SpaceServerContainer> >("AtomSpace")
        .def("storeAtom", &AtomSpace::storeAtom)
        .def("addNode", &AtomSpace::addNode, addNode_overloads())
        .def("addRealAtom", &AtomSpace::addRealAtom,
            addRealAtom_overloads())
        //.def("addLink", &AtomSpace::addLink, addLink_overloads())
        .def("getSpaceServer", &AtomSpace::getSpaceServer,
            return_value_policy<reference_existing_object>())
    ;
}
