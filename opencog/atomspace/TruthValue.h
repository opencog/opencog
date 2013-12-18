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
#include <vector>

#include <opencog/util/exceptions.h>

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

//! TruthValue types
//! XXX TODO This should probably be removed.
//! The truth-value types are currently used in only two places;
//! The guile interpreter, and the SQL peristance layer.  Both of
//! these layers should almost surely use their own private system
//! for serializing/deserializing truth value types, instead of
//! using this.  This is unstable, and should be removed ...
// NUMBER_OF_TRUTH_VALUE_TYPES must be the last one in this enum.
enum TruthValueType
{
    NULL_TRUTH_VALUE = 0,
    SIMPLE_TRUTH_VALUE = 1,
    COUNT_TRUTH_VALUE,
    INDEFINITE_TRUTH_VALUE,
    COMPOSITE_TRUTH_VALUE,
    NUMBER_OF_TRUTH_VALUE_TYPES
};

class TruthValue;
typedef std::shared_ptr<TruthValue> TruthValuePtr;

class TruthValue
{
    friend class CompositeTruthValue;
    friend class SavingLoading;
    friend class Atom;

    // the TruthValueUTest class needs to access private members from the
    // TruthValue class, so we declare it as a friend class.
    friend class ::TruthValueUTest;

    // Disallow assignment -- truth values are immutable!
    TruthValue& operator=(const TruthValue& rhs) {
        throw RuntimeException(TRACE_INFO, "Cannot modify truth values!");
    }

public:
    virtual ~TruthValue() {}

    // Special TVs

    /**
     * The shared reference to a special NullTruthValue object.
     * This is supposed to be used as a Flag only and so,
     * it cannot be used as a normal TV object, as for setting the TV
     * object of an Atom, for example.
     */
    static TruthValuePtr NULL_TV();
    /**
     * The shared reference to a special default (Simple) TruthValue
     * object with both mean and count set to default values
     * (currently 0 and 0).  This is supposed to be used as a
     * temporary TV object (in Formulae and Rules internal
     * TV arrays, for instance).
     */
    static TruthValuePtr DEFAULT_TV();
    /**
     * The shared reference to a special TRUE (Simple) TruthValue
     * object with MAX_TRUTH mean and MAX_TV_CONFIDENCE count.
     */
    static TruthValuePtr TRUE_TV();
    /**
     * The shared reference to a special FALSE (Simple) TruthValue
     * object with 0 mean and MAX_TV_CONFIDENCE count.
     */
    static TruthValuePtr FALSE_TV();
    /**
     * The shared reference to a special TRIVIAL (Simple) TruthValue
     * object with 0 count.
     */
    static TruthValuePtr TRIVIAL_TV();

    /**
     * Gets the name of a TruthValue type
     */
    static const char* getTVTypeName(TruthValueType);

// PURE VIRTUAL METHODS:

    virtual strength_t getMean()  const = 0;
    virtual count_t getCount()  const = 0;
    virtual confidence_t getConfidence()  const = 0;

    virtual std::string toString() const  = 0;
    virtual TruthValueType getType() const  = 0;
    virtual TruthValuePtr clone() const  = 0;
    virtual TruthValue* rawclone() const  = 0;

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
    virtual TruthValuePtr merge(TruthValuePtr) const;

    /**
     * Check if this TV is a null TV.
     */
    virtual bool isNullTv() const;

    /**
     * Check if this TV is equal to the default TV. operator!= only compares pointers
     */
    virtual bool isDefaultTV() const;
};

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
