#include "Link_wrap.h"
#include <boost/python.hpp>

using namespace opencog;
using namespace boost::python;

void init_Link_py()
{
    class_<LinkWrap, bases<Atom>, boost::noncopyable>("Link", no_init)
        .def(init<Type, const std::vector<Handle>&,
            optional<const TruthValue&> >())
        .def(init<Type, Handle&, optional<const TruthValue&> >())
        .def(init<Type, Handle&, Handle&,
            optional<const TruthValue&> >())
        .def(init<Type, Handle&, Handle&, Handle&,
            optional<const TruthValue&> >())
        .def(init<Type, Handle&, Handle&, Handle&, Handle&,
            optional<const TruthValue&> >())
        .def(init<const Link&>())
        .def(self == self)
        .def(self != self)
        .def("hashCode", pure_virtual(&Atom::hashCode))
    ;
}

LinkWrap::LinkWrap(Type t, const std::vector<Handle>& oset,
    const TruthValue& tv = TruthValue::NULL_TV()):
    Link(t, oset, tv)
{}
LinkWrap::LinkWrap(Type t, Handle& h,
    const TruthValue& tv = TruthValue::NULL_TV()):
    Link(t, h, tv)
{}
LinkWrap::LinkWrap(Type t, Handle& ha, Handle& hb,
    const TruthValue& tv = TruthValue::NULL_TV()):
    Link(t, ha, hb, tv)
{}
LinkWrap::LinkWrap(Type t, Handle& ha, Handle& hb, Handle& hc,
    const TruthValue& tv = TruthValue::NULL_TV()):
    Link(t, ha, hb, hc, tv)
{}
LinkWrap::LinkWrap(Type t, Handle& ha, Handle& hb, Handle& hc, Handle& hd,
    const TruthValue& tv = TruthValue::NULL_TV()):
    Link(t, ha, hb, hc, hd, tv)
{}
LinkWrap::LinkWrap(const Link &l):
    Link(l)
{}

// For the pure virtual functions.

// For the non-pure virtual functions.

bool LinkWrap::operator==(const Atom& a) const
{
    if (override o = this->get_override("operator=="))
        return operator==(a);

    return Link::operator==(a);
}
bool LinkWrap::default_operator_equal_equal(const Atom& a) const
{
    return this->Link::operator==(a);
}

bool LinkWrap::operator!=(const Atom& a) const
{
    if (override o = this->get_override("operator!="))
        return operator!=(a);

    return Link::operator!=(a);
}
bool LinkWrap::default_operator_not_equal(const Atom& a) const
{
    return this->Link::operator!=(a);
}

size_t LinkWrap::hashCode(void) const
{
    if (override o = this->get_override("hashCode"))
        return hashCode();

    return Link::hashCode();
}
size_t LinkWrap::default_hashCode(void) const
{
    return this->Link::hashCode();
}
