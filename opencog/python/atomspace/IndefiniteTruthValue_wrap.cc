#include "IndefiniteTruthValue_wrap.h"
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

using namespace boost::python;
using namespace opencog;

void init_IndefiniteTruthValue_py()
{
    class_<IndefiniteTruthValueWrap, bases<TruthValue>, boost::noncopyable>("IndefiniteTruthValue")
        .def(self == self)
    ;
}

// Non-pure virtual functions.

bool IndefiniteTruthValueWrap::operator==(const TruthValue& rhs) const
{
    if (override o = this->get_override("operator=="))
        return operator==(rhs);

    return IndefiniteTruthValue::operator==(rhs);
}
bool IndefiniteTruthValueWrap::default_operator_equal_equal(const TruthValue& rhs) const
{
    return this->IndefiniteTruthValue::operator==(rhs);
}
