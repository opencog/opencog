/*
 * opencog/atomspace/ProbabilisticTruthValue.cc
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

#include "ProbabilisticTruthValue.h"

#include <math.h>

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

using namespace opencog;

ProbabilisticTruthValue::ProbabilisticTruthValue(strength_t m, confidence_t n, count_t c):TruthValue(PROBABILISTIC_TRUTH_VALUE)
{
    mean = m;
    confidence = n;
    count = c;
}

ProbabilisticTruthValue::ProbabilisticTruthValue(const TruthValue& source):TruthValue(PROBABILISTIC_TRUTH_VALUE)
{
    mean = source.getMean();
    confidence = source.getConfidence();
    count = source.getCount();
}
ProbabilisticTruthValue::ProbabilisticTruthValue(ProbabilisticTruthValue const& source):TruthValue(PROBABILISTIC_TRUTH_VALUE)
{
    mean = source.mean;
    confidence = source.confidence;
    count = source.count;
}

strength_t ProbabilisticTruthValue::getMean() const
{
    return mean;
}

count_t ProbabilisticTruthValue::getCount() const
{
    return count;
}

confidence_t ProbabilisticTruthValue::getConfidence() const
{
    return confidence;
}

std::string ProbabilisticTruthValue::toString() const
{
    char buf[1024];
    sprintf(buf, "(ctv %f %f %f)",
            static_cast<float>(getMean()),
            static_cast<float>(getCount()),
            static_cast<double>(getConfidence()));
    return buf;
}

bool ProbabilisticTruthValue::operator==(const TruthValue& rhs) const
{
    const ProbabilisticTruthValue *ctv = dynamic_cast<const ProbabilisticTruthValue *>(&rhs);
    if (NULL == ctv) return false;

#define FLOAT_ACCEPTABLE_ERROR 0.000001
    if (FLOAT_ACCEPTABLE_ERROR < fabs(mean - ctv->mean)) return false;
    if (FLOAT_ACCEPTABLE_ERROR < fabs(confidence - ctv->confidence)) return false;
#define DOUBLE_ACCEPTABLE_ERROR 1.0e-14
    if (DOUBLE_ACCEPTABLE_ERROR < fabs(1.0 - (ctv->count/count))) return false;

    return true;
}

// Note: this is NOT the merge formula used by PLN.  This is
// because the ProbabilisticTruthValue usally stores an integer count,
// and a log-probability or entropy, instead of a confidence.
TruthValuePtr ProbabilisticTruthValue::merge(TruthValuePtr other,MergeOption mo/*=DEFAULT*/) const
{
    ProbabilisticTruthValuePtr oc =
        std::dynamic_pointer_cast<ProbabilisticTruthValue>(other);

    // If other is a simple truth value, *and* its not the default TV,
    // then perhaps we should merge it in, as if it were a count truth
    // value with a count of 1?  In which case, we should add a merge
    // routine to SimpleTruthValue to do likewise... Anyway, for now,
    // just ignore this possible complication to the semantics.
    if (NULL == oc) return std::static_pointer_cast<TruthValue>(clone());
    
    // If both this and other are counts, then accumulate to get the
    // total count, and average together the strengths, using the 
    // count as the relative weight.
    ProbabilisticTruthValuePtr nc = std::static_pointer_cast<ProbabilisticTruthValue>(clone());
    nc->count += oc->count;
    nc->mean = (this->mean * this->count +
                   oc->mean * oc->count) / nc->count;

    // XXX This is not the correct way to handle confidence ... 
    // The confidence will typically hold the log probability,
    // where the probability is the normalized count.  Thus
    // the right thing to do is probably to add the probabilities!?
    // However, this is not correct when the confidence is actually
    // holding the mutual information ... which is additive ... 
    // Argh .. what to do?
    //    nc->confidence = oc->confidence;
    
    return std::static_pointer_cast<TruthValue>(nc);
}

