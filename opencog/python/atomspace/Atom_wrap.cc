#include "Atom_wrap.h"
#include <boost/python.hpp>

using namespace opencog;
using namespace boost::python;

void init_Atom_py()
{
    class_<AtomWrap, bases<AttentionValue>, boost::noncopyable>("Atom", no_init)
#ifndef PUT_OUTGOING_SET_IN_LINKS
        .def(init<Type, const std::vector<Handle>&, const TruthValue&>())
#else
        .def(init<Type, const TruthValue&>())
        //.def(init<Type, optional<const TruthValue&> >())
#endif
        .def("toString", pure_virtual(&Atom::toString))
        .def("toShortString", pure_virtual(&Atom::toShortString))
        .def(self == self)
        .def(self != self)
        .def("hashCode", pure_virtual(&Atom::hashCode))
    ;
}

#ifndef PUT_OUTGOING_SET_IN_LINKS
AtomWrap::AtomWrap(Type t, const std::vector<Handle>& h, const TruthValue& tv):
    Atom(t, h, tv)
{}
#else
AtomWrap::AtomWrap(Type t, const TruthValue& tv):
    Atom(t, tv)
{}
#endif

// For the pure virtual functions.

std::string AtomWrap::toString(void) const
{
    return this->get_override("toString")();
}
std::string AtomWrap::toShortString(void) const
{
    return this->get_override("toShortString")();
}
bool AtomWrap::operator==(const Atom&) const
{
    return this->get_override("operator==")();
}
bool AtomWrap::operator!=(const Atom&) const
{
    return this->get_override("operator!=")();
}
size_t AtomWrap::hashCode(void) const
{
    return this->get_override("hashCode")();
}

// For the non-pure virtual functions.
