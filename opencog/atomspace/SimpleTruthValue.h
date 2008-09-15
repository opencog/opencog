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
#include <opencog/atomspace/utils.h>

namespace opencog
{

class SimpleTruthValue : public TruthValue
{

protected:

    float mean;
    float count;

    void init(float mean, float count);

public:

    SimpleTruthValue(float mean, float count);
    SimpleTruthValue(const TruthValue&);
    SimpleTruthValue(SimpleTruthValue const&);

    SimpleTruthValue* clone() const;
    SimpleTruthValue& operator=(const TruthValue& rhs)
    throw (RuntimeException);

    virtual bool operator==(const TruthValue& rhs) const;

    static SimpleTruthValue* fromString(const char*);
    static float confidenceToCount(float);
    static float countToConfidence(float);

    float toFloat() const;
    std::string toString() const;
    TruthValueType getType() const;

    float getMean() const;
    float getCount() const;
    float getConfidence() const;
    void setMean(float);
    void setCount(float);
    void setConfidence(float);
};

} // namespace opencog

#endif // _OPENCOG_SIMPLE_TRUTH_VALUE_H_
