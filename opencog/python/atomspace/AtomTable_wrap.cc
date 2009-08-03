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
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)(Handle, Type, bool) const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)
                (const std::vector<Handle>&, Type*, bool*, Arity, Type, bool)
            const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)(const char*, Type, bool) const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)(const char*, Type, Type, bool) const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)
                (const char**, Type*, bool*, Arity, Type, bool)
            const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)
                (Type*, bool*, Arity, Type, bool)
            const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)
                (AttentionValue::sti_t, AttentionValue::sti_t)
            const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)(Type, bool, VersionHandle) const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)
                (Type, Type, bool, bool, VersionHandle, VersionHandle)
            const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)
                (Handle, Type, bool, VersionHandle)
            const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("getHandleSet",
            (HandleEntry* (AtomTable::*)
                (const std::vector<Handle>&, Type*, bool*, Arity, Type, bool,
                    VersionHandle)
            const)
            &AtomTable::getHandleSet,
            return_value_policy<manage_new_object>())
        .def("add", &AtomTable::add)
        .def("holds", &AtomTable::holds)
        .def("remove", &AtomTable::remove)
    ;
}
