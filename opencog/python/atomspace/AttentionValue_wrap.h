#ifndef _OPENCOG_ATTENTION_VALUE_WRAP_H
#define _OPENCOG_ATTENTION_VALUE_WRAP_H

#include "AttentionValue.h"
#include <boost/python/wrapper.hpp>
using namespace opencog;
using namespace boost::python;

/** Exposes the AttentionValue and AttentionValueHolder classes. */
void init_AttentionValue_py();

/** A class wrapper of the AttentionValue class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct AttentionValueWrap : AttentionValue, wrapper<AttentionValue>
{
    AttentionValueWrap();
    AttentionValueWrap(sti_t STI);
    AttentionValueWrap(sti_t STI, lti_t LTI);
    AttentionValueWrap(sti_t STI, lti_t LTI, vlti_t VLTI);
    // Non-pure virtual functions.

    sti_t getSTI() const;
    sti_t default_getSTI() const;
    float getScaledSTI() const;
    float default_getScaledSTI() const;
    lti_t getLTI() const;
    lti_t default_getLTI() const;
    vlti_t getVLTI() const;
    vlti_t default_getVLTI() const;
    std::string toString() const;
    std::string default_toString() const;
    AttentionValue* clone() const;
    AttentionValue* default_clone() const;
    bool operator==(const AttentionValue& av) const;
    bool default_operator_equal_equal(const AttentionValue& av) const;
};

#endif
