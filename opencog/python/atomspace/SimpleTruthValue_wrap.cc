#include "SimpleTruthValue_wrap.h"
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

using namespace boost::python;
using namespace opencog;

void init_SimpleTruthValue_py()
{
    class_<SimpleTruthValueWrap, bases<TruthValue>, boost::noncopyable>("SimpleTruthValue", 
    no_init)
        .def(self == self)
    ;
}

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
