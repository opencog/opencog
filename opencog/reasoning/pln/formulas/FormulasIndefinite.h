/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
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

/*
 * src/reasoning/indefinite/IndefinitePLNFormulas.h
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

#ifndef _INDEFINITEPLNFORMULAS_H
#define _INDEFINITEPLNFORMULAS_H

#include "Formula.h"
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <vector>

using namespace opencog;

// ***************************************************
// Syntactic sugar

typedef float number_t;
typedef std::vector<number_t> dvector;
typedef std::vector<number_t*> pvector;
typedef std::vector<std::vector<number_t> > dmatrix;

namespace opencog {
namespace pln {
    
const float
IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor = 1.5f;

static bool SAVE_DEDUCTION_LOOKUP_TABLE = true;
static bool USE_DEDUCTION_LOOKUP_TABLE = false;

void setSaveDeductionLookupTable(bool b);
void setUseDeductionLookupTable(bool b);

/* Wrappers that Implement the Formulas */
class IndefiniteSymmetricBayesFormula : public Formula<3>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

class IndefiniteSymmetricImplicationBreakdownFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

class IndefiniteSymmetricDeductionFormula : public Formula<5>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

class IndefiniteSymmetricRevisionFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

class IndefiniteSymmetricAndFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

class IndefiniteMem2InhFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

class IndefiniteInh2MemFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

// ***************************************************
// Decouple Sampling and Constraints From Core Indefinite Rules
class Sampler
{
public:
    Sampler(void *(*function)(void *), int n1_, int n2_) {
        distribution = function;
        n1 = n1_;
        n2 = n2_;
        deduction = false;
    };

    void generateSample(IndefiniteTruthValue* const& TVa,
                        IndefiniteTruthValue* const& TVb);

    void generateSample(IndefiniteTruthValue* const& TVa,
                        IndefiniteTruthValue* const& TVb,
                        IndefiniteTruthValue* const& TVc);

    void generateSample(IndefiniteTruthValue* const& TVa,
                        IndefiniteTruthValue* const& TVb,
                        IndefiniteTruthValue* const& TVc,
                        IndefiniteTruthValue* const& TVab,
                        IndefiniteTruthValue* const& TVbc);

    void setDimensions(int n1_, int n2_) {
        n1 = n1_;
        n2 = n2_;
    };

    void setDeduction(bool b) {
        deduction = b;
    }

private:
    bool deduction;
    int n1, n2;
    void *(*distribution)(void *arg);
};

// IndefiniteRule Super-class

class IndefiniteRule
{
public:
    IndefiniteRule () { };
    virtual ~IndefiniteRule () { };

    IndefiniteRule (IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVb);

    IndefiniteRule (IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVb,
                    IndefiniteTruthValue* const& TVc);

    IndefiniteRule (IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVb,
                    IndefiniteTruthValue* const& TVc,
                    IndefiniteTruthValue* const& TVab,
                    IndefiniteTruthValue* const& TVbc,
                    bool deduction);

    virtual IndefiniteTruthValue* solve() {
        return NULL;
    };
    IndefiniteTruthValue* conclusion(const pvector& distribution);

protected:
    std::vector<IndefiniteTruthValue *> tvset;
    Sampler *s;
};

// ***************************************************
// Sub-classes: ConjunctionRule, ImplicationRule, Revision
//     AdbductionRule, BayesRule, DeductionRule

class ConjunctionRule : public IndefiniteRule
{
public:
    ConjunctionRule(IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVb);
    IndefiniteTruthValue* solve();
};

class ImplicationRule : public IndefiniteRule
{
public:
    ImplicationRule(IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVb);
    IndefiniteTruthValue* solve();
    IndefiniteTruthValue* q_r_conclusion(float lower, float upper,
                                         const std::vector<std::vector<float> >& d);
};

class RevisionRule : public IndefiniteRule
{
public:
    RevisionRule   (IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVb);
    IndefiniteTruthValue* solve();
};

class BayesRule : public IndefiniteRule
{
public:
    BayesRule    (IndefiniteTruthValue* const& TVa,
                  IndefiniteTruthValue* const& TVc,
                  IndefiniteTruthValue* const& TVac);
    IndefiniteTruthValue* solve();
};

class AbductionRule : public IndefiniteRule
{
public:
    AbductionRule  (IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVb,
                    IndefiniteTruthValue* const& TVc,
                    IndefiniteTruthValue* const& TVab,
                    IndefiniteTruthValue* const& TVbc);
    IndefiniteTruthValue* solve();
};

class DeductionRule : public IndefiniteRule
{
public:
    DeductionRule  (IndefiniteTruthValue* const& TVa,
                    IndefiniteTruthValue* const& TVc,
                    IndefiniteTruthValue* const& TVb,
                    IndefiniteTruthValue* const& TVab,
                    IndefiniteTruthValue* const& TVbc);
    IndefiniteTruthValue* solve();
};

// ***************************************************
// Factory can create any IndefiniteRule sub-classing

template <typename R, typename T>
class RuleGenerator
{
public:
    R*  CreateRule(T* const& a,
                   T* const& b);

    R*  CreateRule(T* const& a,
                   T* const& b,
                   T* const& c);

    R*  CreateRule(T* const& a,
                   T* const& b,
                   T* const& c,
                   T* const& d,
                   T* const& e);
};

template <typename R, typename T>
R*
RuleGenerator<R, T>::CreateRule(T* const& a, T* const& b)
{
    return new R(a, b);
}

template <typename R, typename T>
R*
RuleGenerator<R, T>::CreateRule(T* const& a,
                                T* const& b,
                                T* const& c)
{
    return new R(a, b, c);
}

template <typename R, typename T>
R*
RuleGenerator<R, T>::CreateRule(T* const& a,
                                T* const& b,
                                T* const& c,
                                T* const& d,
                                T* const& e)
{
    return new R(a, b, c, d, e);
}

}} // namespace opencog::pln

#endif /*INDEFINITEPLNFORMULAS_H */
