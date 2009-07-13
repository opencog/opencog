#ifndef _OPENCOG_TRUTH_VALUE_WRAP_H
#define _OPENCOG_TRUTH_VALUE_WRAP_H

#include "TruthValue.h"
#include <boost/python/wrapper.hpp>
using namespace boost::python;
using namespace opencog;

/** Exposes the TruthValue class. */
void init_TruthValue_py();

/** A class wrapper of the TruthValue class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct TruthValueWrap : TruthValue, wrapper<TruthValue>
{
    // Pure virtual functions.

    strength_t getMean() const;
    count_t getCount() const;
    confidence_t getConfidence() const;

    float toFloat() const;
    std::string toString() const;
    TruthValueType getType() const;

    TruthValue* clone() const;

    TruthValue& operator=(const TruthValue&);
    bool operator==(const TruthValue&) const;

    // Non-pure virtual functions.

    TruthValue* merge(const TruthValue&) const;
    TruthValue* default_merge(const TruthValue&) const;
    bool isNullTv() const;
    bool default_isNullTv() const;
};

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
static TruthValue* (*factoryx1)(const char*) = &TruthValue::factory;

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
static TruthValue* (*factoryx2)(TruthValueType, const char*) = &TruthValue::factory;

#endif // _OPENCOG_TRUTH_VALUE_WRAP_H
