/*
 * opencog/atomspace/TruthValue.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Guilherme Lamacie
 *            Welter Silva <welter@vettalabs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_TRUTH_VALUE_H
#define _OPENCOG_TRUTH_VALUE_H

#include <memory>
#include <string>

#include <opencog/atomspace/types.h>
#include <opencog/util/exceptions.h>
#ifdef ZMQ_EXPERIMENT
#include "ProtocolBufferSerializer.h"
#endif
/** \addtogroup grp_atomspace
 *  @{
 */

class TruthValueUTest;

namespace opencog
{

// define strength_t and strength_seq
typedef float strength_t;
typedef std::vector<strength_t> strength_seq;
typedef strength_seq::iterator strength_seq_it;
typedef strength_seq::const_iterator strength_seq_const_it;

// define count_t and count_seq
typedef float count_t;
typedef std::vector<count_t> count_seq;
typedef strength_seq::iterator count_seq_it;
typedef strength_seq::const_iterator count_seq_const_it;

// define confidence_t and confidence_seq
typedef float confidence_t;
typedef std::vector<confidence_t> confidence_seq;
typedef strength_seq::iterator confidence_seq_it;
typedef strength_seq::const_iterator confidence_seq_const_it;

/** @todo This variable was moved from reasoning/StdAfx.h as it was. Find a better
 * implementation for it... (???)
 */
const strength_t MAX_TRUTH  = 1.0;
const confidence_t MAX_CONFIDENCE = 1.0;

//! TruthValue types
// NUMBER_OF_TRUTH_VALUE_TYPES must be the last one in this enum.
enum TruthValueType {
    SIMPLE_TRUTH_VALUE = 1,
    COUNT_TRUTH_VALUE,
    INDEFINITE_TRUTH_VALUE,
    COMPOSITE_TRUTH_VALUE,
    NUMBER_OF_TRUTH_VALUE_TYPES
};
#define MAX_TRUTH_VALUE_NAME_LEN 120

struct tv_summary_t {
    strength_t mean;
    confidence_t confidence;
    count_t count;
};

class TruthValue
{
    friend class CompositeTruthValue;
    friend class SavingLoading;
    friend class Atom;
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;
#endif

    // the TruthValueUTest class needs to access private members from the
    // TruthValue class, so we declare it as a friend class.
    friend class ::TruthValueUTest;

public:

    virtual ~TruthValue() {}

    // Special TVs

    /**
     * The shared reference to a special NullTruthValue object.
     * This is supposed to be used as a Flag only and so,
     * it cannot be used as a normal TV object, as for setting the TV
     * object of an Atom, for example.
     */
    static const TruthValue& NULL_TV();
    /**
     * The shared reference to a special default (Simple) TruthValue
     * object with both mean and count set to default values
     * (currently 0 and 0).  This is supposed to be used as a
     * temporary TV object (in Formulae and Rules internal
     * TV arrays, for instance).
     */
    static const TruthValue& DEFAULT_TV();
    /**
     * The shared reference to a special TRUE (Simple) TruthValue
     * object with MAX_TRUTH mean and MAX_TV_CONFIDENCE count.
     */
    static const TruthValue& TRUE_TV();
    /**
     * The shared reference to a special FALSE (Simple) TruthValue
     * object with 0 mean and MAX_TV_CONFIDENCE count.
     */
    static const TruthValue& FALSE_TV();
    /**
     * The shared reference to a special TRIVIAL (Simple) TruthValue
     * object with 0 count.
     */
    static const TruthValue& TRIVIAL_TV();

    /**
     * Gets the name of a TruthValue type
     */
    static const char* getTVTypeName(TruthValueType);

// PURE VIRTUAL METHODS:

    virtual strength_t getMean()  const = 0;
    virtual count_t getCount()  const = 0;
    virtual confidence_t getConfidence()  const = 0;

    virtual float toFloat() const  = 0;
    virtual std::string toString() const  = 0;
    virtual TruthValueType getType() const  = 0;

    /**
     * Deep clone of this object. Returns a new object. .
     */
    virtual TruthValue* clone() const  = 0;

    /**
     * Assignment operator. Must be implemented by each subclass to
     * allow correct assignment, according with the exact class of
     * the TruthValue objects in the left and right sides of the
     * operator.
     */
    virtual TruthValue& operator=(const TruthValue& rhs) = 0;

    /**
     * Equality. Used to determine if two truth values are the
     * same, or not. Primarily useful see if a TV is equal to
     * NULL_TV, TRUE_TV, FALSE_TV, etc.
     */
    virtual bool operator==(const TruthValue& rhs) const = 0;
    inline bool operator!=(const TruthValue& rhs) const 
         { return !(*this == rhs); }

// VIRTUAL METHODS:

    /**
     * Merge this TV object with the given TV object argument.
     * It always returns a new TV object with the result of the merge,
     * even if it is equals to one of the merged TV objects.
     *
     * Currently tv1.merge(tv2) works as follows:
     * If tv1 and tv2 are not CompositeTruthValue then
     * the resulting TV is, either tv1 or tv2, the one with the highest
     * confidence.
     * If tv1 is a CompositeTruthValue see CompositeTruthValue::merge.
     * If tv2 is a CompositeTruthValue but not tv1,
     * then tv2.CompositeTruthValue::merge(tv1) is called.
     */
    virtual TruthValue* merge(const TruthValue&) const;

    /**
     * Check if this TV is a null TV.
     */
    virtual bool isNullTv() const;

    /**
     * Check if this TV is equal to the default TV. operator!= only compares pointers
     */
    virtual bool isDefaultTV() const;

// STATIC METHODS:

    static const char* typeToStr(TruthValueType t)
    throw (InvalidParamException);
    static TruthValueType strToType(const char* str)
    throw (InvalidParamException);

    // Factories
    // former factory used by NMShell mkatom command
    static TruthValue* factory(const char*);
    static TruthValue* factory(TruthValueType, const char*)
    throw (InvalidParamException);

protected:

    /**
     * Special method that checks if the given TV is not the DefaultTV
     * object, but its equal to it and, if so, delete this object
     * and set it to the DefaultTV object.
     */
    static void DeleteAndSetDefaultTVIfPertinent(TruthValue** tv);

};

typedef std::shared_ptr<TruthValue> TruthValuePtr;

} // namespace opencog

// overload of operator<< to print TruthValue
namespace std {
    template<typename Out>
    Out& operator<<(Out& out, const opencog::TruthValue& tv) {
        return out << tv.toString();
    }
} // ~namespace std


/** @}*/
#endif // _OPENCOG_TRUTH_VALUE_H
