/*
 * opencog/atomspace/SimpleTruthValue.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Guilherme Lamacie
 *            Murilo Queiroz <murilo@vettalabs.com>
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

#ifndef _OPENCOG_SIMPLE_TRUTH_VALUE_H_
#define _OPENCOG_SIMPLE_TRUTH_VALUE_H_

#include <opencog/atomspace/TruthValue.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class SimpleTruthValue;
typedef std::shared_ptr<SimpleTruthValue> SimpleTruthValuePtr;

//! a TruthValue that stores a mean and the number of observations (strength and confidence)
class SimpleTruthValue : public TruthValue
{
protected:

    /// Mean of the strength of the TV over all observations
    strength_t mean;

    /// Total number of observations used to estimate the mean 
    count_t count;

    void init(strength_t mean, count_t count);

public:

    SimpleTruthValue(strength_t mean, count_t count);
    SimpleTruthValue(const TruthValue&);
    SimpleTruthValue(SimpleTruthValue const&);

    virtual bool operator==(const TruthValue& rhs) const;

    /// Heuristic to compute the count given the confidence (according
    /// to the PLN book)
    /// count =  confidence * k / (1 - confidence)
    /// where k is the look-ahead
    static count_t confidenceToCount(confidence_t);

    /// Heuristic to compute the confidence given the count (according
    /// to the PLN book)
    /// confidence = count / (count + k)
    /// where k is the look-ahead
    static confidence_t countToConfidence(count_t);

    std::string toString() const;
    TruthValueType getType() const;

    strength_t getMean() const;
    count_t getCount() const;
    confidence_t getConfidence() const;

    static SimpleTruthValuePtr createSTV(strength_t mean, count_t count)
    {
        return std::make_shared<SimpleTruthValue>(mean, count);
    }
    static TruthValuePtr createTV(strength_t mean, count_t count)
    {
        return std::static_pointer_cast<TruthValue>(createSTV(mean, count));
    }

    TruthValuePtr clone() const
    {
        return std::make_shared<SimpleTruthValue>(*this);
    }
    TruthValue* rawclone() const
    {
        return new SimpleTruthValue(*this);
    }
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_SIMPLE_TRUTH_VALUE_H_
