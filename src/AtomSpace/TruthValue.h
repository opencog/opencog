/*
 * src/AtomSpace/TruthValue.h
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

#ifndef _TRUTH_VALUE_H_
#define _TRUTH_VALUE_H_

#include <string>
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <types.h>
#include "exceptions.h"

// TODO: These variables were moved from reasoning/StdAfx.h as they were. Find a better
// implementation for them...
const int DefaultU = 10000;
const float MAX_TRUTH = 1.0f;
const int MAX_TV_COUNT = DefaultU;

// TruthValue types:
// Warning: NUMBER_OF_TRUTH_VALUE_TYPES must be the last one in this enum.
enum TruthValueType {
    SIMPLE_TRUTH_VALUE = 0,
    INDEFINITE_TRUTH_VALUE,
    COMPOSITE_TRUTH_VALUE,
    NUMBER_OF_TRUTH_VALUE_TYPES
};

class TruthValue
{
    friend class CompositeTruthValue;
    friend class SavingLoading;
    friend class Atom;
    friend class NMSHServer;

    friend class TruthValueUTest;

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
     * object with MAX_TRUTH mean and MAX_TV_COUNT count.
     */
    static const TruthValue& TRUE_TV();
    /**
     * The shared reference to a special FALSE (Simple) TruthValue
     * object with 0 mean and MAX_TV_COUNT count.
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

    virtual float getMean()  const = 0;
    virtual float getCount()  const = 0;
    virtual float getConfidence()  const = 0;

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

// VIRTUAL METHODS:

    /**
     * Merge this TV object with the given TV object argument.
     * It always returns a new TV object with the result of the merge,
     * even if it is equals to one of the merged TV objects.
     */
    virtual TruthValue* merge(const TruthValue&) const;

    /**
     * Check if this TV is a null TV.
     */
    virtual bool isNullTv() const;

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

#endif
