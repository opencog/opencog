#include "SimpleTruthValue_wrap.h"
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

using namespace boost::python;
using namespace opencog;

void init_SimpleTruthValue_py()
{
    class_<SimpleTruthValueWrap, bases<TruthValue>, boost::noncopyable>("SimpleTruthValue", no_init)
        .def(init<strength_t, count_t>())
        .def(init<const TruthValue&>())
        .def(init<SimpleTruthValue const&>())
        .def(self == self)
    ;
}

SimpleTruthValueWrap::SimpleTruthValueWrap(strength_t mean, count_t count):
    SimpleTruthValue(mean, count)
{}
SimpleTruthValueWrap::SimpleTruthValueWrap(const TruthValue& tv):
    SimpleTruthValue(tv)
{}
SimpleTruthValueWrap::SimpleTruthValueWrap(SimpleTruthValue const& stv):
    SimpleTruthValue(stv)
{}

// Non-pure virtual functions.

bool SimpleTruthValueWrap::operator==(const TruthValue& rhs) const
{
    if (override o = this->get_override("operator=="))
        return operator==(rhs);

    return SimpleTruthValue::operator==(rhs);
}
bool SimpleTruthValueWrap::default_operator_equal_equal(const TruthValue& rhs) const
{
    return this->SimpleTruthValue::operator==(rhs);
}

//TruthValue* (*factoryx1)(const char*) = &TruthValue::factory;
//TruthValue* (*factoryx2)(TruthValueType, const char*) = &TruthValue::factory;
/*factoryx1 = TruthValue::factory;
factoryx2 = TruthValue::factory;*/
