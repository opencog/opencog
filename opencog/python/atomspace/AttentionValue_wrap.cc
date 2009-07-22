#include "AttentionValue_wrap.h"
#include <boost/python/def.hpp>
#include <boost/python/class.hpp>
#include <boost/python/manage_new_object.hpp>

using namespace opencog;
using namespace boost::python;

void init_AttentionValue_py()
{
    class_<AttentionValueWrap, boost::noncopyable>("AttentionValue", no_init)
        .def(init<>())
        .def(init< optional<AttentionValueWrap::sti_t,
                    AttentionValueWrap::lti_t, AttentionValueWrap::vlti_t>
                >())
        .def("getSTI", &AttentionValue::getSTI,
            &AttentionValueWrap::default_getSTI)
        .def("getScaledSTI", &AttentionValue::getScaledSTI,
            &AttentionValueWrap::default_getScaledSTI)
        .def("getLTI", &AttentionValue::getLTI,
            &AttentionValueWrap::default_getLTI)
        .def("getVLTI", &AttentionValue::getVLTI,
            &AttentionValueWrap::default_getVLTI)
        .def("toString", &AttentionValue::toString,
            &AttentionValueWrap::default_toString)
        .def("clone", &AttentionValue::clone,
            &AttentionValueWrap::default_clone,
            return_value_policy<manage_new_object>())
        /*.def("operator==`", &AttentionValue::clone,
            &AttentionValueWrap::default_clone)*/
        .def("decaySTI", &AttentionValue::decaySTI)
    ;

    class_<AttentionValueHolder>("AttentionValueHolder")
        .def("getAttentionValue",
            &AttentionValueHolder::getAttentionValue,
            return_internal_reference<>())
    ;
}

AttentionValueWrap::AttentionValueWrap():
    AttentionValue()
{}
AttentionValueWrap::AttentionValueWrap(sti_t STI):
    AttentionValue(STI)
{}
AttentionValueWrap::AttentionValueWrap(sti_t STI, lti_t LTI):
    AttentionValue(STI, LTI)
{}
AttentionValueWrap::AttentionValueWrap(sti_t STI, lti_t LTI, vlti_t VLTI):
    AttentionValue(STI, LTI, VLTI)
{}

// Non-pure virtual functions.

AttentionValue::sti_t AttentionValueWrap::getSTI() const
{
    if (override getSTI = this->get_override("getSTI"))
        return getSTI();

    return AttentionValue::getSTI();
}
AttentionValue::sti_t AttentionValueWrap::default_getSTI() const
{
    return this->AttentionValue::getSTI();
}

float AttentionValueWrap::getScaledSTI() const
{
    if (override o = this->get_override("getScaledSTI"))
        return getScaledSTI();

    return AttentionValue::getScaledSTI();
}
float AttentionValueWrap::default_getScaledSTI() const
{
    return this->AttentionValue::getScaledSTI();
}

AttentionValue::lti_t AttentionValueWrap::getLTI() const
{
    if (override o = this->get_override("getLTI"))
        return getLTI();

    return AttentionValue::getLTI();
}
AttentionValue::lti_t AttentionValueWrap::default_getLTI() const
{
    return this->AttentionValue::getLTI();
}

AttentionValue::vlti_t AttentionValueWrap::getVLTI() const
{
    if (override o = this->get_override("getVLTI"))
        return getVLTI();

    return AttentionValue::getVLTI();
}
AttentionValue::vlti_t AttentionValueWrap::default_getVLTI() const
{
    return this->AttentionValue::getVLTI();
}

std::string AttentionValueWrap::toString() const
{
    if (override o = this->get_override("toString"))
        return toString();

    return AttentionValue::toString();
}
std::string AttentionValueWrap::default_toString() const
{
    return this->AttentionValue::toString();
}

AttentionValue* AttentionValueWrap::clone() const
{
    if (override o = this->get_override("clone"))
        return clone();

    return AttentionValue::clone();
}
AttentionValue* AttentionValueWrap::default_clone() const
{
    return this->AttentionValue::clone();
}

bool AttentionValueWrap::operator==(const AttentionValue& av) const
{
    if (override o = this->get_override("operator=="))
        return operator==(av);

    return AttentionValue::operator==(av);
}
bool AttentionValueWrap::default_operator_equal_equal(const AttentionValue& av) const
{
    return this->AttentionValue::operator==(av);
}
