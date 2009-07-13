#include "CountTruthValue_wrap.h"
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

using namespace boost::python;
using namespace opencog;

void init_CountTruthValue_py()
{
    class_<CountTruthValueWrap, bases<TruthValue>, boost::noncopyable>("CountTruthValue", 
    no_init)
        .def(self == self)
        .def("merge", &CountTruthValue::merge,
            return_value_policy<manage_new_object>())
    ;
}

// Non-pure virtual functions.

bool CountTruthValueWrap::operator==(const TruthValue& rhs) const
{
    if (override o = this->get_override("operator=="))
        return operator==(rhs);

    return CountTruthValue::operator==(rhs);
}
bool CountTruthValueWrap::default_operator_equal_equal(const TruthValue& rhs) const
{
    return this->CountTruthValue::operator==(rhs);
}

TruthValue* CountTruthValueWrap::merge(const TruthValue& tv) const
{
    if (override o = this->get_override("merge"))
        return merge(tv);

    return CountTruthValue::merge(tv);
}
TruthValue* CountTruthValueWrap::default_merge(const TruthValue& tv) const
{
    return this->CountTruthValue::merge(tv);
}
