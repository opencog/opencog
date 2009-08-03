#include "AtomTable_wrap.h"
#include <opencog/atomspace/AtomTable.h>
#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/manage_new_object.hpp>

using namespace opencog;
using namespace boost::python;

void init_AtomTable_py()
{
    class_<AtomTable, boost::noncopyable>("AtomTable", no_init)
        .def(init<optional<bool> >())
        .def("getSize", &AtomTable::getSize)
        .def("getHandle",
            (Handle (AtomTable::*)(const char*, Type) const)
            &AtomTable::getHandle)
        .def("getHandle",
            (Handle (AtomTable::*)(const Node*) const)
            &AtomTable::getHandle)
        .def("getHandle",
            (Handle (AtomTable::*)(Type, const HandleSeq &seq) const)
            &AtomTable::getHandle)
        .def("getHandle",
            (Handle (AtomTable::*)(const Link*) const)
            &AtomTable::getHandle)
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)(Type, bool) const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
    ;
}
