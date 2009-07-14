#ifndef _OPENCOG_COMPOSITE_TRUTH_VALUE_WRAP_H
#define _OPENCOG_COMPOSITE_TRUTH_VALUE_WRAP_H

#include "CompositeTruthValue.h"
#include <boost/python.hpp>
using namespace boost::python;
using namespace opencog;

/** Exposes the CompositeTruthValue class. */
void init_CompositeTruthValue_py();

/** A class wrapper of the CompositeTruthValue class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct CompositeTruthValueWrap : CompositeTruthValue, 
wrapper<CompositeTruthValue>
{
    CompositeTruthValueWrap(const TruthValue&, VersionHandle);
    CompositeTruthValueWrap(CompositeTruthValue const&);

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

#endif // _OPENCOG_COMPOSITE_TRUTH_VALUE_WRAP_H
