#ifndef _OPENCOG_COUNT_TRUTH_VALUE_WRAP_H
#define _OPENCOG_COUNT_TRUTH_VALUE_WRAP_H

#include "CountTruthValue.h"
#include <boost/python.hpp>
using namespace boost::python;
using namespace opencog;

/** Exposes the CountTruthValue class. */
void init_CountTruthValue_py();

/** A class wrapper of the CountTruthValue class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct CountTruthValueWrap : CountTruthValue, wrapper<CountTruthValue>
{
    CountTruthValueWrap(strength_t, confidence_t, count_t);
    CountTruthValueWrap(const TruthValue&);
    CountTruthValueWrap(CountTruthValue const&);

    // Non-pure virtual functions.

    bool operator==(const TruthValue& rhs) const;
    bool default_operator_equal_equal(const TruthValue& rhs) const;
    TruthValue* merge(const TruthValue&) const;
    TruthValue* default_merge(const TruthValue&) const;
};

#endif // _OPENCOG_COUNT_TRUTH_VALUE_WRAP_H
