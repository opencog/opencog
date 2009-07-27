#include "TruthValue_wrap.h"
#include <boost/python/enum.hpp>
#include <boost/python/manage_new_object.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/class.hpp>
#include <boost/python/pure_virtual.hpp>
#include <boost/python/operators.hpp>

using namespace opencog;
using namespace boost::python;

//struct TruthValueWrap::TruthValue_to_python
struct TruthValue_to_python
{
    static PyObject *convert(TruthValue const& tv)
    {
        return incref(object(tv).ptr());
    }
};

void init_TruthValue_py()
{
    enum_<TruthValueType>("TruthValueType")
        .value("SIMPLE_TRUTH_VALUE", SIMPLE_TRUTH_VALUE)
        .value("COUNT_TRUTH_VALUE", COUNT_TRUTH_VALUE)
        .value("INDEFINITE_TRUTH_VALUE", INDEFINITE_TRUTH_VALUE)
        .value("COMPOSITE_TRUTH_VALUE", COMPOSITE_TRUTH_VALUE)
        .value("NUMBER_OF_TRUTH_VALUE_TYPES", NUMBER_OF_TRUTH_VALUE_TYPES)
    ;

    class_<TruthValueWrap, boost::noncopyable>("TruthValue", no_init)
        .def("NULL_TV", &TruthValue::NULL_TV,
            return_value_policy<copy_const_reference>())
        .def("DEFAULT_TV", &TruthValue::DEFAULT_TV,
            return_value_policy<copy_const_reference>())
        .def("TRUE_TV", &TruthValue::TRUE_TV,
            return_value_policy<copy_const_reference>())
        .def("FALSE_TV", &TruthValue::FALSE_TV,
            return_value_policy<copy_const_reference>())
        .def("TRIVIAL_TV", &TruthValue::TRIVIAL_TV,
            return_value_policy<copy_const_reference>())
        /*.def("getTVTypeName", &TruthValue::getTVTypeName,
            return_value_policy<manage_new_object>())*/
        .def("getMean", pure_virtual(&TruthValue::getMean))
        .def("getCount", pure_virtual(&TruthValue::getCount))
        .def("getConfidence", pure_virtual(&TruthValue::getConfidence))
        .def("toFloat", pure_virtual(&TruthValue::toFloat))
        .def("toString", pure_virtual(&TruthValue::toString))
        .def("getType", pure_virtual(&TruthValue::getType))
        .def("clone", pure_virtual(&TruthValue::clone),
            return_value_policy<manage_new_object>())
        .def(self == self)
        .def("merge", &TruthValue::merge, &TruthValueWrap::default_merge,
            return_value_policy<manage_new_object>())
        .def("isNullTv", &TruthValue::isNullTv,
            &TruthValueWrap::default_isNullTv)
        .def("typeToStr", &TruthValue::typeToStr)
        .def("strToType", &TruthValue::strToType)
        .def("factory", factoryx1,
            return_value_policy<manage_new_object>())
        .def("factory", factoryx2,
            return_value_policy<manage_new_object>())
        /* Cannot make work because this function is protected
        .def("DeleteAndSetDefaultTVIfPertinent",
            &TruthValue::DeleteAndSetDefaultTVIfPertinent)*/
        .staticmethod("NULL_TV")
        .staticmethod("DEFAULT_TV")
        .staticmethod("TRUE_TV")
        .staticmethod("FALSE_TV")
        .staticmethod("TRIVIAL_TV")
        //.staticmethod("getTVTypeName")
        .staticmethod("typeToStr")
        .staticmethod("strToType")
        .staticmethod("factory")
        //.staticmethod("DeleteAndSetDefaultTVIfPertinent")
    ;

    //to_python_converter<TruthValue, &TruthValueWrap::TruthValue_to_python>();
    to_python_converter<TruthValue, TruthValue_to_python>();
}

// Pure virtual functions.

strength_t TruthValueWrap::getMean() const
{ return this->get_override("getMean")(); }
count_t TruthValueWrap::getCount() const
{ return this->get_override("getCount")(); }
confidence_t TruthValueWrap::getConfidence() const
{ return this->get_override("getConfidence")(); }
float TruthValueWrap::toFloat() const
{ return this->get_override("toFloat")(); }
std::string TruthValueWrap::toString() const
{ return this->get_override("toString")(); }
TruthValueType TruthValueWrap::getType() const
{ return this->get_override("getType")(); }
TruthValue* TruthValueWrap::clone() const
{ return this->get_override("clone")(); }
TruthValue& TruthValueWrap::operator=(const TruthValue& rhs)
{ return this->get_override("operator=")(rhs); }
bool TruthValueWrap::operator==(const TruthValue& rhs) const
{ return this->get_override("operator==")(rhs); }

// Non-pure virtual functions.

TruthValue* TruthValueWrap::merge(const TruthValue& other) const
{
    if (override o = this->get_override("merge"))
        return o(other);

    return TruthValue::merge(other);
}
TruthValue* TruthValueWrap::default_merge(const TruthValue& other) const
{
    return this->TruthValue::merge(other);
}

bool TruthValueWrap::isNullTv() const
{
    if (override o = this->get_override("isNullTv"))
        return o();

    return TruthValue::isNullTv();
}
bool TruthValueWrap::default_isNullTv() const
{
    return this->TruthValue::isNullTv();
}
