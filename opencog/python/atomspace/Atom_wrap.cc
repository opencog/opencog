#include "Atom_wrap.h"
#include <boost/python.hpp>

using namespace opencog;
using namespace boost::python;

void init_Atom_py()
{
    class_<AtomWrap, bases<AttentionValue>, boost::noncopyable>("Atom", no_init)
        .def(init<Type, const TruthValue&>())
        //.def(init<Type, optional<const TruthValue&> >())
        .def("toString", pure_virtual(&Atom::toString))
        .def("toShortString", pure_virtual(&Atom::toShortString))
        .def(self == self)
        .def(self != self)
        .def("hashCode", pure_virtual(&Atom::hashCode))
        .def("clone", pure_virtual(&Atom::clone),return_value_policy<manage_new_object>() )
    ;
}

AtomWrap::AtomWrap(Type t, const TruthValue& tv):
    Atom(t, tv)
{}

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
Atom* AtomWrap::clone(void) const
{
    return this->get_override("clone")();
}

// For the non-pure virtual functions.
