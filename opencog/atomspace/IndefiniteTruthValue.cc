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

#include "IndefiniteTruthValue.h"

#include <opencog/util/platform.h>
#include <opencog/util/exceptions.h>

#define W() getU()-getL();

using namespace opencog;

float IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL = 0.9f;
float IndefiniteTruthValue::DEFAULT_K = 2.0f;
float IndefiniteTruthValue::diffError = 0.001f;
float IndefiniteTruthValue::s = 0.5f;

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

static float DensityIntegral(float lower, float upper,
                             float L_, float U_, float k_, float s_)
{
    double params[4];
    int status = 0; size_t neval = 0;
    double result = 0, abserr = 0 ;
    gsl_function F;

    params[0] = static_cast<double>(L_);
    params[1] = static_cast<double>(U_);
    params[2] = static_cast<double>(k_);
    params[3] = static_cast<double>(s_);

    F.function = &integralFormula;
    F.params = &params;

    status = gsl_integration_qng (&F, lower, upper,
                                  1e-1, 0.0, &result, &abserr, &neval);
    return (float) result;
}

void IndefiniteTruthValue::init(float l, float u, float c)
{
    L = l;
    U = u;
    confidenceLevel = c;

    // the next 4 variables are initalized to -1 to indicate that they must
    // be calculated when accessed (using getDiff, etc)
    diff = -1.0f;
    mean = -1.0f;
    count = -1.0f;
    confidence = -1.0f;

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

IndefiniteTruthValue::IndefiniteTruthValue(float l, float u, float c)
{
    init(l, u, c);
}

IndefiniteTruthValue::IndefiniteTruthValue(IndefiniteTruthValue const& source)
{
    copy(source);
}

IndefiniteTruthValue* IndefiniteTruthValue::clone() const
{
    return new IndefiniteTruthValue(*this);
}

IndefiniteTruthValue& IndefiniteTruthValue::operator=(const TruthValue & rhs)
    throw (RuntimeException)
{
    const IndefiniteTruthValue* tv =
        dynamic_cast<const IndefiniteTruthValue*>(&rhs);
    if (tv) {
        if (tv != this) { // check if this is the same object first.
            copy(*tv);
        }
    } else {
#if 0
        // The following line was causing a compilation error on MSVC...
        throw RuntimeException(TRACE_INFO,
                               "Cannot assign a TV of type '%s' to one of type '%s'\n",
                               typeid(rhs).name(), typeid(*this).name());
#else
        throw RuntimeException(TRACE_INFO,
                               "Invalid assignment of a IndefiniteTV object\n");
#endif
    }
    return *this;
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

float IndefiniteTruthValue::getL() const
{
    return L;
}
float IndefiniteTruthValue::getU() const
{
    return U;
}

float IndefiniteTruthValue::getDiff()
{
    if (diff >= 0) return diff; // previously calculated
    else {
        if (U == L) { //Nil: I'm not sure returning 0 is the right thing to do
            diff = 0.0f;
            return diff;
        } else {
            float idiff = 0.01; //initial diff suggestion
            diff = findDiff(idiff);
            return diff;
        }
    }
}

float IndefiniteTruthValue::findDiff(float idiff)
{
    float min = 0.0;
    float max = 0.5; //diff cannot be larger than 1/2 because symmetric case
    float L1, U1;
    float numerator, denominator, result;
    float expected = (1 - confidenceLevel) / 2;
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

float IndefiniteTruthValue::getConfidenceLevel() const
{
    return confidenceLevel;
}

const vector<float*>& IndefiniteTruthValue::getFirstOrderDistribution() const
{
    return firstOrderDistribution;
}

float IndefiniteTruthValue::getU_() const
{
    float u = U + diff;
// return (u > 1.0)?1.0f:u;
    return u;
}
float IndefiniteTruthValue::getL_() const
{
    float l = L - diff;
// return (l < 0.0)?0.0f:l;
    return l;
}

void IndefiniteTruthValue::setL(float l)
{
    this->L = l;

    // the next 4 variables are set to -1 to indicate that they must
    // be recalculated
    diff = -1.0f;
    mean = -1.0f;
    count = -1.0f;
    confidence = -1.0f;
}

void IndefiniteTruthValue::setU(float u)
{
    this->U = u;

    // the next 4 variables are set to -1 to indicate that they must
    // be recalculated
    diff = -1.0f;
    mean = -1.0f;
    count = -1.0f;
    confidence = -1.0f;
}

void IndefiniteTruthValue::setConfidenceLevel(float c)
{
    this->confidenceLevel = c;
}

void IndefiniteTruthValue::setDiff(float diff)
{
    this->diff = diff;
}

void IndefiniteTruthValue::setFirstOrderDistribution(const vector<float*>& v)
{
    this->firstOrderDistribution = v;
}

TruthValueType IndefiniteTruthValue::getType() const
{
    return INDEFINITE_TRUTH_VALUE;
}

void IndefiniteTruthValue::setMean(float m)
{
    mean = m;
}

float IndefiniteTruthValue::getMean() const
{
    if (mean < 0) { // must be updated
        mean = (L + U) / 2;
    }
    return mean;
}

float IndefiniteTruthValue::getCount() const
{
    if (count < 0) { // must be updated
        float W = W();
        W = max(W, 0.0000001f); // to avoid division by zero
        count = (DEFAULT_K * (1 - W) / W);
    }
    return count;
}

float IndefiniteTruthValue::getConfidence() const
{
    if (confidence < 0) { // must be updated
        float c = getCount();
        confidence = c / (c + DEFAULT_K);
    }
    return confidence;
}

bool IndefiniteTruthValue::isSymmetric() const
{
    return symmetric;
}

std::string IndefiniteTruthValue::toString() const
{
    char buf[1024];
    sprintf(buf, "[%f,%f,%f,%f,%f,%d]", mean, L, U, confidenceLevel, diff, symmetric);
    return buf;
}

IndefiniteTruthValue* IndefiniteTruthValue::fromString(const char* tvStr)
{
    float m, l, u, c, d;
    int s;
    sscanf(tvStr, "[%f,%f,%f,%f,%f,%d]", &m, &l, &u, &c, &d, &s);
    //printf("IndefiniteTruthValue::fromString(%s) => mean = %f, L = %f, U = %f, confLevel = %f, diff = %f, symmetric = %d\n", tvStr, m, l, u, c, d, s);
    IndefiniteTruthValue* result = new IndefiniteTruthValue(l, u, c);
    result->setDiff(d);
    result->symmetric = s != 0;
    result->setMean(m);
    return result;
}

float IndefiniteTruthValue::toFloat() const
{
    return getMean();
}
