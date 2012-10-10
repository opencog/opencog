/*
 * opencog/atomspace/CountTruthValue.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Guilherme Lamacie
 *            Linas Vepstas <linasvepstas@gmail.com>
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
 *
 */

#include "CountTruthValue.h"

#include <math.h>

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

using namespace opencog;

CountTruthValue::CountTruthValue(strength_t m, confidence_t n, count_t c)
{
    mean = m;
    confidence = n;
    count = c;
}

CountTruthValue::CountTruthValue(const TruthValue& source)
{
    mean = source.getMean();
    confidence = source.getConfidence();
    count = source.getCount();
}
CountTruthValue::CountTruthValue(CountTruthValue const& source)
{
    mean = source.mean;
    confidence = source.confidence;
    count = source.count;
}

#ifdef ZMQ_EXPERIMENT
CountTruthValue::CountTruthValue(const ZMQSingleTruthValueMessage& singleTruthValue)
{
	mean=singleTruthValue.mean();
	confidence=singleTruthValue.confidence();
	count=singleTruthValue.count();
}
#endif

void CountTruthValue::setMean(strength_t m)
{
    mean = m;
}

void CountTruthValue::setCount(count_t c)
{
    count = c;
}

void CountTruthValue::setConfidence(confidence_t c)
{
    confidence = c;
}

strength_t CountTruthValue::getMean() const
{
    return mean;
}

count_t CountTruthValue::getCount() const
{
    return count;
}

confidence_t CountTruthValue::getConfidence() const
{
    return confidence;
}

float CountTruthValue::toFloat() const
{
    return static_cast<float>(getMean());
}

std::string CountTruthValue::toString() const
{
    char buf[1024];
    sprintf(buf, "[%f,%f,%f]",
            static_cast<float>(getMean()),
            static_cast<float>(getCount()),
            static_cast<float>(getConfidence()));
    return buf;
}

CountTruthValue* CountTruthValue::clone() const
{
    return new CountTruthValue(*this);
}

CountTruthValue& CountTruthValue::operator=(const TruthValue & rhs)
    throw (RuntimeException)
{
    const CountTruthValue* tv = dynamic_cast<const CountTruthValue*>(&rhs);
    if (tv) {
        if (tv != this) { // check if this is the same object first.
            mean = tv->mean;
            confidence = tv->confidence;
            count = tv->count;
        }
    } else {
#ifndef WIN32
        // The following line was causing a compilation error on MSVC...
        throw RuntimeException(TRACE_INFO,
              "Cannot assign a TV of type '%s' to one of type '%s'\n",
              typeid(rhs).name(), typeid(*this).name());
#else
        throw RuntimeException(TRACE_INFO,
              "CountTruthValue - Invalid assignment of a CountTV object.");
#endif

    }
    return *this;
}

bool CountTruthValue::operator==(const TruthValue& rhs) const
{
    const CountTruthValue *ctv = dynamic_cast<const CountTruthValue *>(&rhs);
    if (NULL == ctv) return false;
    if (mean != ctv->mean) return false;
    if (confidence != ctv->confidence) return false;
    if (count != ctv->count) return false;
    return true;
}

TruthValueType CountTruthValue::getType() const
{
    return COUNT_TRUTH_VALUE;
}

CountTruthValue* CountTruthValue::fromString(const char* tvStr)
{
    float tmean, tcount, tconf;
    sscanf(tvStr, "[%f,%f,%f]", &tmean, &tconf, &tcount);
    return new CountTruthValue(static_cast<strength_t>(tmean),
                               static_cast<confidence_t>(tconf),
                               static_cast<count_t>(tcount));
}

TruthValue* CountTruthValue::merge(const TruthValue& other) const
{
    const CountTruthValue *oc =
        dynamic_cast<const CountTruthValue *>(&other);

    // If other is a simple truth value, then perhaps we should 
    // merge it in, as if it were a count truth value with a count 
    // of 1?  In which case, we should add a merge routine to 
    // SimpleTruthValue to do likewise... Anyway, for now, just
    // ignore this possible complication to the semantics.
    if (NULL == oc) return TruthValue::merge(other);
    
    // If both this and other are counts, then accumulate to get the
    // total count, and average together the strengths, using the 
    // count as the relative weight.
    CountTruthValue *nc = clone();
    nc->count += oc->count;
    nc->mean = (this->mean * this->count +
                   oc->mean * oc->count) / nc->count;
    if (oc->confidence > nc->confidence)
    {
        nc->confidence = oc->confidence;
    }
    return nc;
}

#ifdef ZMQ_EXPERIMENT
void CountTruthValue::writeToZMQMessage(ZMQTruthValueMessage* truthValueMessage)
{
	ZMQSingleTruthValueMessage *singleTruthValue = truthValueMessage->add_singletruthvalue();
	singleTruthValue->set_truthvaluetype(ZMQTruthValueTypeCount);
	singleTruthValue->set_mean(mean);
	singleTruthValue->set_confidence(confidence);
	singleTruthValue->set_count(count);
}
#endif
