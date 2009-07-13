#ifndef _OPENCOG_INDEFINITE_TRUTH_VALUE_WRAP_H
#define _OPENCOG_INDEFINITE_TRUTH_VALUE_WRAP_H

#include "IndefiniteTruthValue.h"
#include <boost/python.hpp>
using namespace boost::python;
using namespace opencog;

/** Exposes the IndefiniteTruthValue class. */
void init_IndefiniteTruthValue_py();

/** A class wrapper of the IndefiniteTruthValue class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct IndefiniteTruthValueWrap : IndefiniteTruthValue, 
wrapper<IndefiniteTruthValue>
{
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

#endif // _OPENCOG_INDEFINITE_TRUTH_VALUE_WRAP_H
