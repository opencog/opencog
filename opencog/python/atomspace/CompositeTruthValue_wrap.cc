#include "CompositeTruthValue_wrap.h"
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

using namespace boost::python;
using namespace opencog;

void init_CompositeTruthValue_py()
{
    class_<CompositeTruthValueWrap, bases<TruthValue>, boost::noncopyable>("CompositeTruthValue", no_init)
        .def(init<const TruthValue&, VersionHandle>())
        .def(init<CompositeTruthValue const&>())
        .def(self == self)
    ;
}

CompositeTruthValueWrap::CompositeTruthValueWrap(const TruthValue& t,
    VersionHandle vh):
    CompositeTruthValue(t, vh)
{}
CompositeTruthValueWrap::CompositeTruthValueWrap(CompositeTruthValue const& 
ctv):
    CompositeTruthValue(ctv)
{}

// Non-pure virtual functions.

bool CompositeTruthValueWrap::operator==(const TruthValue& rhs) const
{
    if (override o = this->get_override("operator=="))
        return operator==(rhs);

    return CompositeTruthValue::operator==(rhs);
}
bool CompositeTruthValueWrap::default_operator_equal_equal(const TruthValue& rhs) const
{
    return this->CompositeTruthValue::operator==(rhs);
}

//TruthValue* (*factoryx1)(const char*) = &TruthValue::factory;
//TruthValue* (*factoryx2)(TruthValueType, const char*) = &TruthValue::factory;
/*factoryx1 = TruthValue::factory;
factoryx2 = TruthValue::factory;*/
