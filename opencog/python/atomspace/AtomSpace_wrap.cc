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
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(removeAtom_overloads, removeAtom, 1, 2)
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addLink_overloads, addLink, 1, 11)

void init_AtomSpace_py()
{
    class_<std::vector<Handle> >("std_vector_Handle");

    class_<AtomSpaceWrap, bases<SpaceServerContainer>, boost::noncopyable >("AtomSpace")
        .def("storeAtom", &AtomSpace::storeAtom)
        .def("addNode", &AtomSpace::addNode, addNode_overloads())
        .def("addRealAtom", &AtomSpace::addRealAtom,
            addRealAtom_overloads())
        .def("removeAtom", &AtomSpace::removeAtom,
            removeAtom_overloads())
        //.def("addLink", &AtomSpace::addLink, addLink_overloads())
        //.def("addLink", &AtomSpaceWrap::addLinkp1)
        //.def("addLink", addLinkp1)
        .def("addLink", &AtomSpaceWrap::addLinkx1x1)
        .def("addLink", &AtomSpaceWrap::addLinkx1x2)
        /*.def("addLink", &AtomSpaceWrap::addLinkx2x1)
        .def("addLink", &AtomSpaceWrap::addLinkx2x2)*/
        /*.def("addLink", addLinkx1)
        .def("addLink", addLinkx2)*/
        .def("getSpaceServer", &AtomSpace::getSpaceServer,
            return_value_policy<reference_existing_object>())
    ;
}
