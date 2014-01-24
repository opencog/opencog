/*
 * opencog/atomspace/CountTruthValue.h
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

#ifndef _OPENCOG_COUNT_TRUTH_VALUE_H_
#define _OPENCOG_COUNT_TRUTH_VALUE_H_

#include <opencog/atomspace/TruthValue.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class CountTruthValue;
typedef std::shared_ptr<CountTruthValue> CountTruthValuePtr;

//! a TruthValue that stores a mean, a confidence and the number of observations
class CountTruthValue : public TruthValue
{
protected:

    strength_t mean;
    confidence_t confidence;
    count_t count;

public:

    CountTruthValue(strength_t, confidence_t, count_t);
    CountTruthValue(const TruthValue&);
    CountTruthValue(CountTruthValue const&);

    virtual bool operator==(const TruthValue& rhs) const;

    std::string toString() const;
    TruthValueType getType() const;

    strength_t getMean() const;
    count_t getCount() const;
    confidence_t getConfidence() const;

    virtual TruthValuePtr merge(TruthValuePtr) const;

    static TruthValuePtr createTV(strength_t s, confidence_t f, count_t c)
    {
        return std::static_pointer_cast<TruthValue>(
            std::make_shared<CountTruthValue>(s, f, c));
    }

    TruthValuePtr clone() const
    {
        return std::make_shared<CountTruthValue>(*this);
    }
    TruthValue* rawclone() const
    {
        return new CountTruthValue(*this);
    }
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_COUNT_TRUTH_VALUE_H_
