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
#ifdef ZMQ_EXPERIMENT
#include "ProtocolBufferSerializer.h"
#endif

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

//! a TruthValue that stores a mean, a confidence and the number of observations
class CountTruthValue : public TruthValue
{
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;

private:
    CountTruthValue() {};
#endif

protected:

    strength_t mean;
    confidence_t confidence;
    count_t count;

    void init(strength_t, confidence_t, count_t);

public:

    CountTruthValue(strength_t, confidence_t, count_t);
    CountTruthValue(const TruthValue&);
    CountTruthValue(CountTruthValue const&);

    CountTruthValue* clone() const;
    CountTruthValue& operator=(const TruthValue& rhs)
        throw (RuntimeException);

    virtual bool operator==(const TruthValue& rhs) const;

    static CountTruthValue* fromString(const char*);

    float toFloat() const;
    std::string toString() const;
    TruthValueType getType() const;

    strength_t getMean() const;
    count_t getCount() const;
    confidence_t getConfidence() const;
    void setMean(strength_t);
    void setCount(count_t);
    void setConfidence(confidence_t);

    virtual TruthValue* merge(const TruthValue&) const;

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_COUNT_TRUTH_VALUE_H_
