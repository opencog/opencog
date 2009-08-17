#include "AtomSpace_wrap.h"
#include "AtomSpace.h"
#include "SpaceServerContainer.h"
#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/reference_existing_object.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/overloads.hpp>

using namespace opencog;
using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addNode_overloads, addNode, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(removeAtom_overloads, removeAtom, 1, 2)
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(addLink_overloads, addLink, 1, 11)

void init_AtomSpace_py()
{
    class_<std::vector<Handle> >("std_vector_Handle");

    class_<AtomSpaceWrap, bases<SpaceServerContainer>, boost::noncopyable >("AtomSpace", no_init)
        .def(init<>())
        /*.def("registerBackingStore", &AtomSpace::registerBackingStore)
        .def("unregisterBackingStore", &AtomSpace::unregisterBackingStore)*/
        .def("storeAtom", &AtomSpace::storeAtom)
        .def("addNode", &AtomSpace::addNode, addNode_overloads())
        .def("removeAtom", &AtomSpace::removeAtom,
            removeAtom_overloads())
        .def("fetchAtom", &AtomSpace::fetchAtom)
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
        .def("getSTI",
            (AttentionValue::sti_t (AtomSpace::*)(AttentionValueHolder*) const)
            &AtomSpace::getSTI)
        .def("getSTI",
            (AttentionValue::sti_t (AtomSpace::*)(Handle) const)
            &AtomSpace::getSTI)
        .def("setSTI",
            (void (AtomSpace::*)(AttentionValueHolder*, AttentionValue::sti_t))
            &AtomSpace::setSTI)
        .def("setSTI",
            (void (AtomSpace::*)(Handle, AttentionValue::sti_t))
            &AtomSpace::setSTI)
        .def("getTV", &AtomSpace::getTV,
            return_value_policy<copy_const_reference>())
        .def("setTV", &AtomSpace::setTV)
        .def("getAV",
            (const AttentionValue& (AtomSpace::*)(AttentionValueHolder*) const)
            &AtomSpace::getAV,
            return_value_policy<copy_const_reference>())
        .def("getAV",
            (const AttentionValue& (AtomSpace::*)(Handle) const)
            &AtomSpace::getAV,
            return_value_policy<copy_const_reference>())
        .def("setAV",
            (void (AtomSpace::*)(AttentionValueHolder*, const AttentionValue&))
            &AtomSpace::setAV)
        .def("setAV",
            (void (AtomSpace::*)(Handle, const AttentionValue&))
            &AtomSpace::setAV)
        .def("setLTI",
            (void (AtomSpace::*)(AttentionValueHolder*, AttentionValue::lti_t))
            &AtomSpace::setLTI)
        .def("setLTI",
            (void (AtomSpace::*)(Handle, AttentionValue::lti_t))
            &AtomSpace::setLTI)
        .def("getLTI",
            (AttentionValue::lti_t (AtomSpace::*)(Handle) const)
            &AtomSpace::getLTI)
        .def("getLTI",
            (AttentionValue::lti_t (AtomSpace::*)(AttentionValueHolder*) const)
            &AtomSpace::getLTI)
        .def("getVLTI",
            (AttentionValue::vlti_t (AtomSpace::*)(Handle) const)
            &AtomSpace::getVLTI)
        .def("getVLTI",
            (AttentionValue::vlti_t (AtomSpace::*)(AttentionValueHolder*) const)
            &AtomSpace::getVLTI)
        .def("getName", (std::string (AtomSpace::*)(Type) const)
            &AtomSpace::getName)
        .def("getName", (const std::string& (AtomSpace::*)(Handle) const)
            &AtomSpace::getName,
            return_value_policy<copy_const_reference>())
        .def("getOutgoing", (Handle (AtomSpace::*)(Handle, int) const)
            &AtomSpace::getOutgoing)
        .def("getOutgoing", (const HandleSeq& (AtomSpace::*)(Handle) const)
            &AtomSpace::getOutgoing,
            return_value_policy<copy_const_reference>())
        .def("getHandle",
            (Handle (AtomSpace::*)(Type, const std::string&) const)
            &AtomSpace::getHandle)
        .def("getHandle",
            (Handle (AtomSpace::*)(Type, const HandleSeq&) const)
            &AtomSpace::getHandle)
        .def("getType", &AtomSpace::getType)
    ;
}
