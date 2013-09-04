/*
 * src/reasoning/indefinite/IndefinitePLNFormulas.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 *
 * Modified by Cesar Marcondes <cesar@cs.ucla.edu>
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

#include "FormulasIndefinite.h"
#include <assert.h>
#include <math.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <algorithm>
#include <opencog/util/macros.h>

using std::vector;

namespace opencog {
namespace pln {

gsl_rng* rng_ = gsl_rng_alloc (gsl_rng_mt19937);

/* In order to build other distributions
 * wrap around the argv and the result
 */
void* BetaDistribution(void *arg)
{
    float * argv; argv = (float*)arg;
    float a = argv[0];
    float b = argv[1];
    float *c = new float(gsl_ran_beta(rng_, a, b));
    return (void *) c;
}

static number_t scaleLU(number_t value, number_t L_, number_t U_)
{
    value = L_ + (U_ - L_) * value;
    value = (value > 1.0) ? 1.0 : (value < 0.0) ? 0.0 : value;
    return value;
}

static void truncate(number_t &value)
{
    value = (value <= 0.0) ? 0.000001 : (value >= 1.0) ? 0.999999 : value;
}

void setSaveDeductionLookupTable(bool b)
{
    SAVE_DEDUCTION_LOOKUP_TABLE = b;
}

void setUseDeductionLookupTable(bool b)
{
    USE_DEDUCTION_LOOKUP_TABLE = b;
}

void Sampler::generateSample(IndefiniteTruthValuePtr TVa,
                             IndefiniteTruthValuePtr TVb)
{

    number_t valuesA[100], valuesB[100];
    float alpha = IndefiniteTruthValue::DEFAULT_K * 0.5;
    float beta = alpha;
    float default_k = IndefiniteTruthValue::DEFAULT_K;
    float args[2];
    vector <float*> distributionA, distributionB;

    // Alocate distributionA and distributionB

    // check if first order distribution is ready
    distributionA = TVa->getFirstOrderDistribution();
    distributionB = TVb->getFirstOrderDistribution();
    if (!distributionA.empty() && !distributionB.empty()) return;

    // step 1 - calculate diff on both variables
    // verify if L_ and U_ were previously calculated

    TVa->getDiff();
    TVb->getDiff();

    // step 2 - generate n1 values for each TV
    // for two TV - no need to check consistency
    // The parameters alfa=beta=0.5 is
    // the default for beta distribution

    args[0] = alpha;
    args[1] = beta;

    int n1 = 100; n2 = 100;

    for (int i = 0; i < n1; i++) {
        valuesA[i] = *(number_t*) distribution(args);
        valuesB[i] = *(number_t*) distribution(args);

        valuesA[i] = scaleLU(valuesA[i], TVa->getL_(), TVa->getU_());
        valuesB[i] = scaleLU(valuesB[i], TVb->getL_(), TVb->getU_());

        // Alocate distributionA and distributionB
        distributionA.push_back(new float[100]);
        distributionB.push_back(new float[100]);
    }

    // step 3 - first Order Distribution

    for (int i = 0; i < n1; i++) {
        truncate(valuesA[i]);
        truncate(valuesB[i]);
        //fill distributionA
        args[0] = default_k * valuesA[i];
        args[1] = default_k * (1 - valuesA[i]);
        for (int j = 0; j < n2; j++) {
            distributionA[i][j] = *(number_t*) distribution(args);
            truncate(distributionA[i][j]);
        }
        //fill distributionB
        args[0] = default_k * valuesB[i];
        args[1] = default_k * (1 - valuesB[i]);
        for (int j = 0;j < n2;j++) {
            distributionB[i][j] = *(number_t*) distribution(args);
            truncate(distributionB[i][j]);
        }
    }

    TVa->setFirstOrderDistribution(distributionA);
    TVb->setFirstOrderDistribution(distributionB);
}

void Sampler::generateSample(IndefiniteTruthValuePtr TVa,
                             IndefiniteTruthValuePtr TVb,
                             IndefiniteTruthValuePtr TVc)
{

    number_t valuesA[100], valuesC[100], valuesAC[100];
    float alpha = IndefiniteTruthValue::DEFAULT_K * 0.5;
    float beta = alpha;
    float default_k = IndefiniteTruthValue::DEFAULT_K;
    float args[2];
    vector <float*> distributionA, distributionC, distributionAC;

    // check if first order distribution is ready
    distributionA = TVa->getFirstOrderDistribution();
    distributionC = TVb->getFirstOrderDistribution();
    distributionAC = TVc->getFirstOrderDistribution();
    if (!distributionA.empty() &&
            !distributionC.empty() && !distributionAC.empty()) return;

    // step 1 - same as generateSample(TVa, TVb);
    TVa->getDiff();
    TVb->getDiff();
    TVc->getDiff();

    // step 2 - generate n1 consistent values for each TV
    args[0] = alpha;
    args[1] = beta;
    int n1 = 100; n2 = 100;

    for (int i = 0; i < n1; i++) {
        do {
            valuesA[i] = *(number_t*) distribution(args);
            valuesC[i] = *(number_t*) distribution(args);
            valuesAC[i] = *(number_t*) distribution(args);

            valuesA[i] = scaleLU(valuesA[i], TVa->getL_(), TVa->getU_());
            valuesC[i] = scaleLU(valuesC[i], TVb->getL_(), TVb->getU_());
            valuesAC[i] = scaleLU(valuesAC[i], TVc->getL_(), TVc->getU_());
        } while (valuesAC[i] <
                 GSL_MAX((valuesA[i] + valuesC[i] - 1) / valuesA[i], 0)
                 ||
                 valuesAC[i] > GSL_MIN(valuesC[i] / valuesA[i], 1));

        // Alocate distributionA and distributionB
        distributionA.push_back(new float[100]);
        distributionC.push_back(new float[100]);
        distributionAC.push_back(new float[100]);
    }

    // step 3 - first Order Distribution
    for (int i = 0; i < n1; i++) {
        truncate(valuesA[i]);
        truncate(valuesC[i]);
        truncate(valuesAC[i]);

        for (int j = 0;j < n2;j++) {
            do {
                args[0] = default_k * valuesA[i];
                args[1] = default_k * (1 - valuesA[i]);
                distributionA[i][j] = *(number_t*) distribution(args);
                truncate(distributionA[i][j]);

                args[0] = default_k * valuesC[i];
                args[1] = default_k * (1 - valuesC[i]);
                distributionC[i][j] = *(number_t*) distribution(args);
                truncate(distributionC[i][j]);

                args[0] = default_k * valuesAC[i];
                args[1] = default_k * (1 - valuesAC[i]);
                distributionAC[i][j] = *(number_t*) distribution(args);
                truncate(distributionAC[i][j]);

            } while (distributionAC[i][j] <
                     GSL_MAX((distributionA[i][j] + distributionC[i][j] - 1) /
                             distributionA[i][j], 0) ||
                     distributionAC[i][j] > GSL_MIN(distributionC[i][j] /
                                                    distributionA[i][j], 1)
                    );
        }
    }

    TVa->setFirstOrderDistribution(distributionA);
    TVb->setFirstOrderDistribution(distributionC);
    TVc->setFirstOrderDistribution(distributionAC);
}

void Sampler::generateSample(IndefiniteTruthValuePtr TVa,
                             IndefiniteTruthValuePtr TVb,
                             IndefiniteTruthValuePtr TVc,
                             IndefiniteTruthValuePtr TVab,
                             IndefiniteTruthValuePtr TVcb_bc)
{

    number_t valuesA[100], valuesB[100],
    valuesC[100], valuesAB[100], valuesCB_BC[100];
    float alpha = IndefiniteTruthValue::DEFAULT_K * 0.5;
    float beta = alpha;
    float default_k = IndefiniteTruthValue::DEFAULT_K;
    float args[2];
    bool check = false;
    vector <float*> distributionA, distributionB,
    distributionC, distributionAB, distributionCB_BC;

    // check if first order distribution is ready
    distributionA = TVa->getFirstOrderDistribution();
    distributionB = TVb->getFirstOrderDistribution();
    distributionC = TVc->getFirstOrderDistribution();
    distributionAB = TVab->getFirstOrderDistribution();
    distributionCB_BC = TVcb_bc->getFirstOrderDistribution();

    if (!distributionA.empty() && !distributionB.empty() &&
        !distributionC.empty() && !distributionAB.empty() &&
        !distributionCB_BC.empty()) return;

    // step 1 - same as generateSample(TVa, TVb);
    TVa->getDiff();
    TVb->getDiff();
    TVc->getDiff();
    TVab->getDiff();
    TVcb_bc->getDiff();

    // step 2 - generate n1 consistent values for each TV
    args[0] = alpha;
    args[1] = beta;

    for (int i = 0; i < n1; i++) {
        do {
            valuesA[i] = *(number_t*) distribution(args);
            valuesB[i] = *(number_t*) distribution(args);
            valuesC[i] = *(number_t*) distribution(args);
            valuesAB[i] = *(number_t*) distribution(args);
            valuesCB_BC[i] = *(number_t*) distribution(args);

            valuesA[i] = scaleLU(valuesA[i], TVa->getL_(), TVa->getU_());
            valuesB[i] = scaleLU(valuesB[i], TVb->getL_(), TVb->getU_());
            valuesC[i] = scaleLU(valuesC[i], TVc->getL_(), TVc->getU_());
            valuesAB[i] = scaleLU(valuesAB[i], TVab->getL_(), TVab->getU_());
            valuesCB_BC[i] = scaleLU(valuesCB_BC[i],
                                     TVcb_bc->getL_(), TVcb_bc->getU_());

            if (!deduction)
                check = (valuesCB_BC[i] <
                         GSL_MAX((valuesC[i] + valuesB[i] - 1) / valuesC[i], 0)
                         || valuesCB_BC[i] >
                         GSL_MIN(valuesB[i] / valuesC[i], 1));
            else
                check = (valuesCB_BC[i] <
                         GSL_MAX((valuesB[i] + valuesC[i] - 1) / valuesB[i], 0)
                         || valuesCB_BC[i] >
                         GSL_MIN(valuesC[i] / valuesB[i], 1));

        } while (valuesAB[i] <
                 GSL_MAX((valuesA[i] + valuesB[i] - 1) / valuesA[i], 0) ||
                 valuesAB[i] > GSL_MIN(valuesB[i] / valuesA[i], 1) ||
                 check);

        // Alocate distributionA and distributionB
        distributionA.push_back(new float[100]);
        distributionB.push_back(new float[100]);
        distributionC.push_back(new float[100]);
        distributionAB.push_back(new float[100]);
        distributionCB_BC.push_back(new float[100]);
    }

    // step 3 - first Order Distribution

    for (int i = 0; i < n1; i++) {
        truncate(valuesA[i]);
        truncate(valuesB[i]);
        truncate(valuesC[i]);
        truncate(valuesAB[i]);
        truncate(valuesCB_BC[i]);

        for (int j = 0;j < n2;j++) {
            do {
                args[0] = default_k * valuesA[i];
                args[1] = default_k * (1 - valuesA[i]);
                distributionA[i][j] = *(number_t*) distribution(args);
                truncate(distributionA[i][j]);

                args[0] = default_k * valuesB[i];
                args[1] = default_k * (1 - valuesB[i]);
                distributionB[i][j] = *(number_t*) distribution(args);
                truncate(distributionB[i][j]);

                args[0] = default_k * valuesC[i];
                args[1] = default_k * (1 - valuesC[i]);
                distributionC[i][j] = *(number_t*) distribution(args);
                truncate(distributionC[i][j]);

                args[0] = default_k * valuesAB[i];
                args[1] = default_k * (1 - valuesAB[i]);
                distributionAB[i][j] = *(number_t*) distribution(args);
                truncate(distributionAB[i][j]);

                args[0] = default_k * valuesCB_BC[i];
                args[1] = default_k * (1 - valuesCB_BC[i]);
                distributionCB_BC[i][j] = *(number_t*) distribution(args);
                truncate(distributionCB_BC[i][j]);

                if (!deduction)
                    check = (distributionCB_BC[i][j] <
                             GSL_MAX((distributionC[i][j] + distributionB[i][j] - 1) /
                                     distributionC[i][j] , 0)
                             || distributionCB_BC[i][j] >
                             GSL_MIN(distributionB[i][j] / distributionC[i][j], 1));
                else
                    check = (distributionCB_BC[i][j] <
                             GSL_MAX((distributionB[i][j] + distributionC[i][j] - 1) /
                                     distributionB[i][j], 0)
                             || distributionCB_BC[i][j] >
                             GSL_MIN(distributionC[i][j] / distributionB[i][j], 1));

            } while (distributionAB[i][j] <
                     GSL_MAX((distributionA[i][j] + distributionB[i][j] - 1) /
                             distributionA[i][j], 0) ||
                     distributionAB[i][j] > GSL_MIN(distributionB[i][j] /
                                                    distributionA[i][j], 1) || check);
        }
    }

    TVa->setFirstOrderDistribution(distributionA);
    TVb->setFirstOrderDistribution(distributionB);
    TVc->setFirstOrderDistribution(distributionC);
    TVab->setFirstOrderDistribution(distributionAB);
    TVcb_bc->setFirstOrderDistribution(distributionCB_BC);
}

IndefiniteRule::IndefiniteRule (IndefiniteTruthValuePtr TVa,
                                IndefiniteTruthValuePtr TVb)
{
    tvset.push_back(TVa);
    tvset.push_back(TVb);
    s = new Sampler(BetaDistribution, 100, 100);
    s->generateSample(TVa, TVb);
}

IndefiniteRule::IndefiniteRule (IndefiniteTruthValuePtr TVa,
                                IndefiniteTruthValuePtr TVb,
                                IndefiniteTruthValuePtr TVc)
{
    tvset.push_back(TVa);
    tvset.push_back(TVb);
    tvset.push_back(TVc);
    s = new Sampler(BetaDistribution, 100, 100);
    s->generateSample(TVa, TVb, TVc);
}

IndefiniteRule::IndefiniteRule (IndefiniteTruthValuePtr TVa,
                                IndefiniteTruthValuePtr TVb,
                                IndefiniteTruthValuePtr TVc,
                                IndefiniteTruthValuePtr TVab,
                                IndefiniteTruthValuePtr TVbc,
                                bool deduction)
{
    tvset.push_back(TVa);
    tvset.push_back(TVb);
    tvset.push_back(TVc);
    tvset.push_back(TVab);
    tvset.push_back(TVbc);
    s = new Sampler(BetaDistribution, 100, 100);
    s->setDeduction(deduction);
    s->generateSample(TVa, TVb, TVc, TVab, TVbc);
}

// Common conclusion among IndefiniteRules
IndefiniteTruthValue* IndefiniteRule::conclusion(const pvector& distributions)
{

    int qBelow, qAbove, n1 = 100, n2 = 100;
    float mean = 0.0, strength = 0.0, percentile = 0.0;
    vector <float> means;

    IndefiniteTruthValue *result = new IndefiniteTruthValue();

    // Compute Mean of Means from All Distributions
    for (int i = 0;i < n1;i++) {
        mean = 0.0f;
        for (int j = 0;j < n2;j++) {
            mean += distributions[i][j];
        }
        means.push_back(mean / n2);
        strength += means[i];
    }

    strength = strength / n1;
    result->setMean(strength);

    // Find L and U based on Confidence Percentile
    percentile = (1 - IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL) / 2;
    qBelow = int(ceil(n1 * percentile));
    qAbove = qBelow;

    std::sort(means.begin(), means.end());
    result->setL(means[qBelow]);
    result->setU(means[n1-qAbove-1]);

    return result;
}

// Methods Implementations
ConjunctionRule::ConjunctionRule(IndefiniteTruthValuePtr TVa,
                                 IndefiniteTruthValuePtr TVb)
    : IndefiniteRule::IndefiniteRule(TVa, TVb) {}

IndefiniteTruthValue* ConjunctionRule::solve()
{
    int n1 = 100; int n2 = 100;
    vector <float *> distributionResult;

    const pvector& distributionA = tvset[0]->getFirstOrderDistribution();
    const pvector& distributionB = tvset[1]->getFirstOrderDistribution();

    for (int i = 0;i < n1;i++) {
        distributionResult.push_back(new float[100]);
        for (int j = 0;j < n2;j++) {
            distributionResult[i][j] = distributionA[i][j] * distributionB[i][j];
        }
    }

    return IndefiniteRule::conclusion(distributionResult);
}

ImplicationRule::ImplicationRule(IndefiniteTruthValuePtr TVa,
                                 IndefiniteTruthValuePtr TVab)
    : IndefiniteRule::IndefiniteRule(TVa, TVab) {}

IndefiniteTruthValue* ImplicationRule::solve()
{
    int n1 = 100, n2 = 100;
    vector <float *> Q, R;
    vector <float> meansQ, meansR, meansQR;
    float meanQ = 0.0, meanR = 0.0, lower = 0.0, upper = 0.0;
    vector<vector<float> > result;

    const pvector& distributionA = tvset[0]->getFirstOrderDistribution();
    const pvector& distributionB = tvset[1]->getFirstOrderDistribution();

    for (int i = 0;i < n1;i++) {

        meanQ = 0.0; meanR = 0.0;
        Q.push_back(new float[100]);
        R.push_back(new float[100]);

        for (int j = 0;j < n2;j++) {
            Q[i][j] = distributionA[i][j] * distributionB[i][j];
            R[i][j] = 1 - distributionA[i][j] + Q[i][j];
            meanQ += Q[i][j];
            meanR += R[i][j];
        }

        meansQ.push_back(meanQ / n2);
        meansR.push_back(meanR / n2);

        if (i == 0) {
            lower = meansQ[0];
            upper = meansQ[0];
        }

        meansQR.push_back((meansQ[i] + meansR[i]) / 2);

        if (meansQ[i] < lower)
            lower = meansQ[i];

        if (meansR[i] > upper)
            upper = meansR[i];
    }


    result.push_back(meansQ);
    result.push_back(meansR);
    result.push_back(meansQR);
    return ImplicationRule::q_r_conclusion(lower, upper, result);
}

IndefiniteTruthValue* ImplicationRule::q_r_conclusion(float l, float u,
        const vector<vector<float> >& d)
{
    int middle, count, n1 = 100;
    float lower = l, upper = u, middle_d, rate, strength = 0;
    float b = IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL;
    const vector<float>& meansQ = d[0];
    const vector<float>& meansR = d[1];
    const vector<float>& meansQR = d[2];

    middle_d = floor(b * n1);
    middle = int(middle_d);
    rate = 0.001f;

    while (true) {
        count = 0;
        lower = lower + rate;
        upper = upper - rate;

        for (int i = 0;i < n1;i++) {
            strength += meansQR[i];
            if (meansQ[i] >= lower && meansR[i] <= upper) count++;
        }

        if (count <= middle) {
            if (count < middle) {
                lower = (lower + (lower - rate)) / 2;
                upper = (upper + (upper + rate)) / 2;
            }

            IndefiniteTruthValue* result =
                new IndefiniteTruthValue(lower, upper);
            result->setMean(strength / n1);
            return result;
        }
    }
}

RevisionRule::RevisionRule(IndefiniteTruthValuePtr TVa,
                           IndefiniteTruthValuePtr TVb)
    : IndefiniteRule::IndefiniteRule(TVa, TVb) {}

IndefiniteTruthValue* RevisionRule::solve()
{
    int n1 = 100, n2 = 100;
    float w1, w2, n1_, n2_;
    vector <float *> distributionResult;
    const pvector& distributionA = tvset[0]->getFirstOrderDistribution();
    const pvector& distributionB = tvset[1]->getFirstOrderDistribution();
    IndefiniteTruthValue* result;

    w1 = tvset[0]->getU() - tvset[0]->getL();
    w2 = tvset[1]->getU() - tvset[1]->getL();

    n1_ = IndefiniteTruthValue::DEFAULT_K * (1 - w1) / w1;
    n2_ = IndefiniteTruthValue::DEFAULT_K * (1 - w2) / w2;

    w1 = n1_ / (n1_ + n2_);
    w2 = n2_ / (n1_ + n2_);

    for (int i = 0;i < n1;i++) {
        distributionResult.push_back(new float[100]);
        for (int j = 0;j < n2;j++)
            distributionResult[i][j] = (w1 * distributionA[i][j])
                                       + (w2 * distributionB[i][j]);
    }

    result = IndefiniteRule::conclusion(distributionResult);
    return result;
}

AbductionRule::AbductionRule(IndefiniteTruthValuePtr TVa,
                             IndefiniteTruthValuePtr TVb,
                             IndefiniteTruthValuePtr TVc,
                             IndefiniteTruthValuePtr TVab,
                             IndefiniteTruthValuePtr TVcb)
    : IndefiniteRule::IndefiniteRule(TVa, TVb, TVc, TVab, TVcb, false) {}

IndefiniteTruthValue* AbductionRule::solve()
{
    int n1 = 100, n2 = 100;
    vector <float *> distributionAC;
    const pvector& distributionA = tvset[0]->getFirstOrderDistribution();
    OC_UNUSED(distributionA);
    const pvector& distributionB = tvset[1]->getFirstOrderDistribution();
    const pvector& distributionC = tvset[2]->getFirstOrderDistribution();
    const pvector& distributionAB = tvset[3]->getFirstOrderDistribution();
    const pvector& distributionCB = tvset[4]->getFirstOrderDistribution();

    for (int i = 0;i < n1;i++) {
        distributionAC.push_back(new float[100]);
        for (int j = 0;j < n2;j++) {
            if (distributionB[i][j] <= 0.000001)
                distributionAC[i][j] = distributionC[i][j];
            if (distributionB[i][j] >= 0.999999)
                distributionAC[i][j] = 1.0;

            if (distributionB[i][j]<0.999999 && distributionB[i][j]>0.000001) {
                distributionAC[i][j] =
                    distributionAB[i][j] * distributionCB[i][j] *
                    distributionC[i][j] / distributionB[i][j]
                    +
                    (1 - distributionAB[i][j]) * (1 - distributionCB[i][j]) *
                    distributionC[i][j] / (1 - distributionB[i][j]);
            }
        }
    }

    return IndefiniteRule::conclusion(distributionAC);
}

BayesRule::BayesRule(IndefiniteTruthValuePtr TVa,
                     IndefiniteTruthValuePtr TVc,
                     IndefiniteTruthValuePtr TVac)
    : IndefiniteRule::IndefiniteRule(TVa, TVc, TVac) {}

IndefiniteTruthValue* BayesRule::solve()
{
    int n1 = 100, n2 = 100;
    vector <float *> distributionCA;
    const pvector& distributionA = tvset[0]->getFirstOrderDistribution();
    const pvector& distributionC = tvset[1]->getFirstOrderDistribution();
    const pvector& distributionAC = tvset[2]->getFirstOrderDistribution();

    for (int i = 0;i < n1;i++) {
        distributionCA.push_back(new float[100]);
        for (int j = 0;j < n2;j++) {
            if (distributionC[i][j] <= 0.000001)
                distributionCA[i][j] = 0.0;
            else
                distributionCA[i][j] = 
                    (distributionA[i][j] * distributionAC[i][j])
                    / distributionC[i][j];
        }
    }

    return IndefiniteRule::conclusion(distributionCA);
}

DeductionRule::DeductionRule(IndefiniteTruthValuePtr TVa,
                             IndefiniteTruthValuePtr TVb,
                             IndefiniteTruthValuePtr TVc,
                             IndefiniteTruthValuePtr TVab,
                             IndefiniteTruthValuePtr TVbc)
    : IndefiniteRule::IndefiniteRule(TVa, TVb, TVc, TVab, TVbc, true) {}

IndefiniteTruthValue* DeductionRule::solve()
{
    int n1 = 100, n2 = 100;
    vector <float *> distributionAC;

    const pvector& distributionA = tvset[0]->getFirstOrderDistribution();
    OC_UNUSED(distributionA);
    const pvector& distributionB = tvset[1]->getFirstOrderDistribution();
    const pvector& distributionC = tvset[2]->getFirstOrderDistribution();
    const pvector& distributionAB = tvset[3]->getFirstOrderDistribution();
    const pvector& distributionBC = tvset[4]->getFirstOrderDistribution();

    for (int i = 0;i < n1;i++) {
        distributionAC.push_back(new float[100]);
        for (int j = 0;j < n2;j++) {
            if (distributionC[i][j] >= 0.999999)
                distributionAC[i][j] = 
                    distributionAB[i][j] * distributionBC[i][j];
            else
                distributionAC[i][j] =
                    distributionAB[i][j] * distributionBC[i][j]
                    + (1 - distributionAB[i][j]) * (distributionC[i][j] 
                                                    - distributionB[i][j] *
                                                    distributionBC[i][j])
                    / (1 - distributionB[i][j]);
        }
    }
    return IndefiniteRule::conclusion(distributionAC);
}

TruthValue* IndefiniteSymmetricBayesFormula::simpleCompute(const TVSeq& TV, long U) const
{
    int N = TV.size();
    assert(N == 3);
    assert(TV[0]); assert(TV[1]); assert(TV[2]);
    IndefiniteTruthValuePtr TVa = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[0]);
    IndefiniteTruthValuePtr TVc = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[1]);
    IndefiniteTruthValuePtr TVac = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[2]);
    IndefiniteTruthValue* result;

    RuleGenerator<BayesRule, IndefiniteTruthValuePtr> myCreator;
    BayesRule *a = myCreator.CreateRule(TVa, TVc, TVac);
    result = a->solve();
    return result;
}

TruthValue* IndefiniteSymmetricImplicationBreakdownFormula::simpleCompute(const TVSeq& TV, long U) const
{
    int N = TV.size();
    assert(N == 2);
    assert(TV[0]); assert(TV[1]);
    IndefiniteTruthValuePtr linkAB = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[0]);
    IndefiniteTruthValuePtr TVA = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[1]);
    IndefiniteTruthValue* result;

    RuleGenerator<ImplicationRule, IndefiniteTruthValuePtr> myCreator;
    ImplicationRule *a = myCreator.CreateRule(TVA, linkAB);
    result = a->solve();
    return result;
}

TruthValue* IndefiniteSymmetricDeductionFormula::simpleCompute(const TVSeq& TV,
                                                               long U) const
{
    int N = TV.size();
    assert(N == 5);
    assert(TV[0]); assert(TV[1]);
    assert(TV[2]); assert(TV[3]);assert(TV[4]);
    IndefiniteTruthValuePtr TVa = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[0]);
    IndefiniteTruthValuePtr TVb = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[1]);
    IndefiniteTruthValuePtr TVc = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[2]);
    IndefiniteTruthValuePtr TVab = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[3]);
    IndefiniteTruthValuePtr TVbc = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[4]);
    IndefiniteTruthValue* result;

    RuleGenerator<DeductionRule, IndefiniteTruthValuePtr> myCreator;
    DeductionRule *a = myCreator.CreateRule(TVa, TVb, TVc, TVab, TVbc);
    result = a->solve();
    return result;
}

TruthValue* IndefiniteSymmetricRevisionFormula::simpleCompute(const TVSeq& TV,
                                                              long U) const
{
    int N = TV.size();
    assert(N == 2);
    assert(TV[0]); assert(TV[1]);
    IndefiniteTruthValuePtr TVa = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[0]);
    IndefiniteTruthValuePtr TVb = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[1]);
    IndefiniteTruthValue* result;

    RuleGenerator<RevisionRule, IndefiniteTruthValuePtr> myCreator;
    RevisionRule *a = myCreator.CreateRule(TVa, TVb);
    result = a->solve();
    return result;
}

TruthValue* IndefiniteSymmetricAndFormula::simpleCompute(const TVSeq& TV,
                                                         long U) const
{
    int N = TV.size();
    assert(N == 2);
    assert(TV[0]); assert(TV[1]);
    IndefiniteTruthValuePtr TVa = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[0]);
    IndefiniteTruthValuePtr TVb = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[1]);
    IndefiniteTruthValue* result;
    RuleGenerator<ConjunctionRule, IndefiniteTruthValuePtr> myCreator;
    ConjunctionRule *a = myCreator.CreateRule(TVa, TVb);
    result = a->solve();
    return result;
}

TruthValue* IndefiniteMem2InhFormula::simpleCompute(const TVSeq& TV,
                                                    long U) const
{
    int N = TV.size();
    assert(N == 1);
    assert(TV[0]);
    IndefiniteTruthValuePtr TVA = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[0]);
    float width = TVA->getU() - TVA->getL();
    float center = TVA->getL() + width / 2;
    float L_ = center - ((width * IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor) / 2);
    float U_ = center + ((width * IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor) / 2);
    truncate(L_);
    truncate(U_);
    IndefiniteTruthValue* result = new IndefiniteTruthValue(L_, U_);
    result->setMean(TVA->getMean());
    return result;
}

TruthValue* IndefiniteInh2MemFormula::simpleCompute(const TVSeq& TV,
                                                    long U) const
{
    int N = TV.size();
    assert(N == 1);
    assert(TV[0]);
    IndefiniteTruthValuePtr TVA = boost::dynamic_pointer_cast<IndefiniteTruthValue>(TV[0]);
    float width = TVA->getU() - TVA->getL();
    float center = TVA->getL() + width / 2;
    float L_ = center - ((width * 1.2f) / 2);
    float U_ = center + ((width * 1.2f) / 2);
    truncate(L_);
    truncate(U_);
    IndefiniteTruthValue* result = new IndefiniteTruthValue(L_, U_);
    result->setMean(TVA->getMean());
    return result;
}

}} // namespace opencog::pln
