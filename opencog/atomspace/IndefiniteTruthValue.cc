/*
 * opencog/atomspace/IndefiniteTruthValue.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Fabricio Silva <fabricio@vettalabs.com>
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

#include <gsl/gsl_math.h>
#include <gsl/gsl_integration.h>

#include "IndefiniteTruthValue.h"

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

#define W() getU()-getL();

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

confidence_t IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL = 0.9;
count_t IndefiniteTruthValue::DEFAULT_K = 2.0;
strength_t IndefiniteTruthValue::diffError = 0.001;
strength_t IndefiniteTruthValue::s = 0.5;


// Formula defined in the integral of step one [(x-L1)^ks * (U1-x)^k(1-s)
static double integralFormula (double x, void * params)
{
    double L_, U_, k_, s_;
    double *in_params = static_cast<double*>(params);
    L_ = in_params[0];
    U_ = in_params[1];
    k_ = in_params[2];
    s_ = in_params[3];
    double f = (pow((x - L_), (k_ * s_))) * pow((U_ -x), (k_ * (1 - s_)));
    return f;
}

static strength_t DensityIntegral(strength_t lower, strength_t upper,
                                  strength_t L_, strength_t U_,
                                  count_t k_, strength_t s_)
{
    double params[4];
    size_t neval = 0;
    double result = 0.0, abserr = 0.0;
    gsl_function F;

    params[0] = static_cast<double>(L_);
    params[1] = static_cast<double>(U_);
    params[2] = static_cast<double>(k_);
    params[3] = static_cast<double>(s_);

    F.function = &integralFormula;
    F.params = &params;

    gsl_integration_qng (&F, lower, upper,
                         1e-1, 0.0, &result, &abserr, &neval);
    return (strength_t) result;
}

void IndefiniteTruthValue::init(strength_t l, strength_t u, confidence_t c)
{
    L = l;
    U = u;
    confidenceLevel = c;

    // the next 4 variables are initalized to -1 to indicate that they must
    // be calculated when accessed (using getDiff, etc)
    diff = -1.0;
    mean = -1.0;
    count = -1.0;
    confidence = -1.0;

    firstOrderDistribution.clear();
    symmetric = true;
}

void IndefiniteTruthValue::copy(const IndefiniteTruthValue& source)
{
    L = source.L;
    U = source.U;
    confidenceLevel = source.confidenceLevel;
    diff = source.diff;
    mean = source.mean;
    count = source.count;
    confidence = source.confidence;
    symmetric = source.symmetric;
}

IndefiniteTruthValue::IndefiniteTruthValue()
{
    init();
}

IndefiniteTruthValue::IndefiniteTruthValue(strength_t l, strength_t u,
                                           confidence_t c)
{
    init(l, u, c);
}

IndefiniteTruthValue::IndefiniteTruthValue(IndefiniteTruthValue const& source)
{
    copy(source);
}

bool IndefiniteTruthValue::operator==(const TruthValue& rhs) const
{
    const IndefiniteTruthValue* itv = dynamic_cast<const IndefiniteTruthValue*>(&rhs);
    if (NULL == itv) {
        return false;
    } else {
        return  (U == itv->U && L == itv->L 
                 && confidenceLevel == itv->confidenceLevel);
    }
}

strength_t IndefiniteTruthValue::getL() const
{
    return L;
}
strength_t IndefiniteTruthValue::getU() const
{
    return U;
}

strength_t IndefiniteTruthValue::getDiff()
{
    if (diff >= 0) return diff; // previously calculated
    else {
        if (U == L) { //Nil: I'm not sure returning 0 is the right thing to do
            diff = 0.0;
            return diff;
        } else {
            strength_t idiff = 0.01; //initial diff suggestion
            diff = findDiff(idiff);
            return diff;
        }
    }
}

strength_t IndefiniteTruthValue::findDiff(strength_t idiff)
{
    strength_t min = 0.0;
    strength_t max = 0.5; //diff cannot be larger than 1/2 cause symmetric case
    strength_t L1, U1;
    strength_t numerator, denominator, result;
    strength_t expected = (1 - confidenceLevel) / 2;
    bool lte, gte; //smaller than expected, greater than expected

    //loop until convergence
    do {
        U1 = U + idiff;
        L1 = L - idiff;

        numerator = DensityIntegral(U, U1, L1, U1, DEFAULT_K, s);
        denominator = DensityIntegral(L1, U1, L1, U1, DEFAULT_K, s);

        if (denominator > 0) result = numerator / denominator;
        else result = 0.0;

        lte = result < expected - diffError;
        gte = result > expected + diffError;

        if (lte) {
            min = idiff;
            idiff = (idiff + max) / 2;
        }
        if (gte) {
            max = idiff;
            idiff = (min + idiff) / 2;
        }
    } while (lte || gte);

    return idiff;
}

confidence_t IndefiniteTruthValue::getConfidenceLevel() const
{
    return confidenceLevel;
}

const std::vector<strength_t*>& IndefiniteTruthValue::getFirstOrderDistribution() const
{
    return firstOrderDistribution;
}

strength_t IndefiniteTruthValue::getU_() const
{
    strength_t u = U + diff;
// return (u > 1.0)?1.0f:u;
    return u;
}
strength_t IndefiniteTruthValue::getL_() const
{
    strength_t l = L - diff;
// return (l < 0.0)?0.0f:l;
    return l;
}

void IndefiniteTruthValue::setL(strength_t l)
{
    this->L = l;

    // the next 4 variables are set to -1 to indicate that they must
    // be recalculated
    diff = -1.0;
    mean = -1.0;
    count = -1.0;
    confidence = -1.0;
}

void IndefiniteTruthValue::setU(strength_t u)
{
    this->U = u;

    // the next 4 variables are set to -1 to indicate that they must
    // be recalculated
    diff = -1.0;
    mean = -1.0;
    count = -1.0;
    confidence = -1.0;
}

void IndefiniteTruthValue::setConfidenceLevel(confidence_t c)
{
    this->confidenceLevel = c;
}

void IndefiniteTruthValue::setDiff(strength_t diff)
{
    this->diff = diff;
}

void IndefiniteTruthValue::setFirstOrderDistribution(const std::vector<strength_t*>& v)
{
    this->firstOrderDistribution = v;
}

TruthValueType IndefiniteTruthValue::getType() const
{
    return INDEFINITE_TRUTH_VALUE;
}

void IndefiniteTruthValue::setMean(strength_t m)
{
    mean = m;
}

strength_t IndefiniteTruthValue::getMean() const
{
    if (mean < 0) { // mean must be updated
        mean = (L + U) / 2;
    }
    return mean;
}

count_t IndefiniteTruthValue::getCount() const
{
    if (count < 0) { // count must be updated
        strength_t W = W();
        // to avoid division by zero
        W = std::max(W, static_cast<strength_t>(0.0000001)); 
        count = (DEFAULT_K * (1 - W) / W);
    }
    return count;
}

float IndefiniteTruthValue::getConfidence() const
{
    if (confidence < 0) { // confidence must be updated
        count_t c = getCount();
        confidence = c / (c + DEFAULT_K);
    }
    return confidence;
}

bool IndefiniteTruthValue::isSymmetric() const
{
    return symmetric;
}

// Merge formula, as specified by PLN.
TruthValuePtr IndefiniteTruthValue::merge(TruthValuePtr other) const
{
    if (other->getConfidence() > getConfidence()) {
        return other->clone();
    }
    return clone();
}

std::string IndefiniteTruthValue::toString() const
{
    char buf[1024];
    sprintf(buf, "[%f,%f,%f,%f,%f,%d]",
            static_cast<float>(mean),
            static_cast<float>(L),
            static_cast<float>(U),
            static_cast<float>(confidenceLevel),
            static_cast<float>(diff),
            symmetric);
    return buf;
}
