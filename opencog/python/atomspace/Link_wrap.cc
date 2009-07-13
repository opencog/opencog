#include "Link_wrap.h"
#include <boost/python.hpp>

using namespace opencog;
using namespace boost::python;

void init_Link_py()
{
    class_<LinkWrap, bases<Atom>, boost::noncopyable>("Link", no_init)
        .def(self == self)
        .def(self != self)
        .def("hashCode", pure_virtual(&Atom::hashCode))
    ;
}

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
