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

// Truth-value components
// For essentially all truth-value calculations, float is enough, so
// we save space here, and use float. For counting, a float is not
// enough -- it gets up to 16 million (24 bits) and then clamps. So
// we use a double for counting, which should provide 48 bits. Since
// SimpleTruthValue does not store count anyway, there is no storage
// penalty associated with this.
typedef float strength_t;
typedef float confidence_t;
typedef double count_t;

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
    NUMBER_OF_TRUTH_VALUE_TYPES
};

class TruthValue;
typedef std::shared_ptr<TruthValue> TruthValuePtr;

class TruthValue
    : public std::enable_shared_from_this<TruthValue>
{
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
     * This is supposed to be used only for book-keeping, and it must
     * not be used as a normal TV object. Calling methods on it will
     * throw exceptions.
     */
    static TruthValuePtr NULL_TV();
    /**
     * The shared reference to a special TRUE (Simple) TruthValue
     * object with MAX_TRUTH mean and MAX_TV_CONFIDENCE count. That is,
     * its true with absolute confidence.
     */
    static TruthValuePtr TRUE_TV();
    /**
     * The shared reference to a special default (Simple) TruthValue
     * object with MAX_TRUTH mean and 0 count.  That is, its true,
     * but with no confidence.
     */
    static TruthValuePtr DEFAULT_TV();
    /**
     * The shared reference to a special FALSE (Simple) TruthValue
     * object with 0 mean and MAX_TV_CONFIDENCE count. That is, its
     * false with absolute confidence.
     */
    static TruthValuePtr FALSE_TV();
    /**
     * The shared reference to a special TRIVIAL (Simple) TruthValue
     * object with 0 mean and 0 count. That is, its false, but with
     * no confidence.
     */
    static TruthValuePtr TRIVIAL_TV();

    /**
     * Gets the name of a TruthValue type
     */
    static const char* getTVTypeName(TruthValueType);

// PURE VIRTUAL METHODS:

    virtual strength_t getMean()  const = 0;
    virtual confidence_t getConfidence()  const = 0;
    virtual count_t getCount()  const = 0;

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
     * even if it is equal to one of the merged TV objects.
     */
    virtual TruthValuePtr merge(TruthValuePtr) const = 0;

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
        out << tv.toString();
        return out;
    }
} // ~namespace std


/** @}*/
#endif // _OPENCOG_TRUTH_VALUE_H
