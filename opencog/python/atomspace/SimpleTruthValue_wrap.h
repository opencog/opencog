#ifndef _OPENCOG_SIMPLE_TRUTH_VALUE_WRAP_H
#define _OPENCOG_SIMPLE_TRUTH_VALUE_WRAP_H

#include "SimpleTruthValue.h"
#include <boost/python.hpp>
using namespace boost::python;
using namespace opencog;

/** Exposes the SimpleTruthValue class. */
void init_SimpleTruthValue_py();

/** A class wrapper of the SimpleTruthValue class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct SimpleTruthValueWrap : SimpleTruthValue, wrapper<SimpleTruthValue>
{
    SimpleTruthValueWrap(strength_t, count_t);
    SimpleTruthValueWrap(const TruthValue&);
    SimpleTruthValueWrap(SimpleTruthValue const&);

    // Non-pure virtual functions.

    bool operator==(const TruthValue& rhs) const;
    bool default_operator_equal_equal(const TruthValue& rhs) const;
};

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
//static TruthValue* (*factoryx1)(const char*) = &TruthValue::factory;

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
//static TruthValue* (*factoryx2)(TruthValueType, const char*) = &TruthValue::factory;

#endif // _OPENCOG_SIMPLE_TRUTH_VALUE_WRAP_H
