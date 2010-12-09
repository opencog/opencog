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

#include "../PLN.h"
#include "FormulasIndefinite.h"
#include "Formulas.h"
#include "../PLNUtils.h"
#include <assert.h>
#include <memory>
#include <algorithm>

using namespace std;

const bool PLNdebug = true;

//namespace haxx
//{
//    int contractInclusionExclusionFactorial(int total);
//};

///fabricio: method that checks if all TV are of indefinite type (return false otherwise)
///In order to avoid "multiple definition" error and to not create a definition file only to
///definine this function, I declared it as inline
inline bool isAllIndefiniteTruthValueType(TruthValue** TV, int N)
{
    for (int i = 0; i < N; i++) {
        if (!(TV[i]->getType() == INDEFINITE_TRUTH_VALUE))
            return false;
    }
    return true;
}

namespace opencog {
namespace pln {

/*=============================================================================
  Macros of common body parts 
=============================================================================*/

#define PLNFormulaBodyFor_Link \
    assert(N == 1); \
    assert(TV[0]); \
    TruthValue* linkAB = TV[0]; \
    strength_t sAB = linkAB->getMean(); \
    count_t nAB = linkAB->getCount(); \
     
#define PLNFormulaBodyFor_Atom2 \
    assert(N == 2); \
    assert(TV[0]); \
    assert(TV[1]); \
    strength_t sA = TV[0]->getMean(); \
    count_t nA = TV[0]->getCount(); \
    strength_t sB = TV[1]->getMean(); \
    count_t nB = TV[1]->getCount(); \
     
#define PLNFormulaBodyFor_Link1Node2 \
    assert(N == 3); \
    assert(TV[0]); \
    assert(TV[1]); \
    assert(TV[2]); \
    TruthValue* linkAB = TV[0]; \
    TruthValue* atomA = TV[1]; \
    TruthValue* atomB = TV[2]; \
    strength_t sAB = linkAB->getMean(); \
    strength_t sA = atomA->getMean(); \
    strength_t sB = atomB->getMean(); \
    count_t nAB = linkAB->getCount(); \
    count_t nA = atomA->getCount(); \
    count_t nB = atomB->getCount();

#define PLNFormulaBodyFor_Link2Node2 \
    assert(N == 4); \
    assert(TV[0]); \
    assert(TV[1]); \
    assert(TV[2]); \
    assert(TV[3]); \
    TruthValue* linkAB = TV[0]; \
    TruthValue* linkBA = TV[1]; \
    TruthValue* atomA = TV[2]; \
    TruthValue* atomB = TV[3]; \
    strength_t sAB = linkAB->getMean(); \
    strength_t sBA = linkBA->getMean(); \
    strength_t sA = atomA->getMean(); \
    strength_t sB = atomB->getMean(); \
    count_t nAB = linkAB->getCount(); \
    count_t nBA = linkBA->getCount(); \
    count_t nA = atomA->getCount(); \
    count_t nB = atomB->getCount();

#define PLNFormulaBodyFor_Link2Node3 \
    assert(N == 5); \
    assert(TV[0]); \
    assert(TV[1]); \
    assert(TV[2]); \
    assert(TV[3]); \
    assert(TV[4]); \
    TruthValue* linkAB = TV[0]; \
    TruthValue* linkBC = TV[1]; \
    TruthValue* atomA = TV[2]; \
    TruthValue* atomB = TV[3]; \
    TruthValue* atomC = TV[4]; \
    strength_t sA = atomA->getMean(); \
    strength_t sB = atomB->getMean(); \
    strength_t sC = atomC->getMean(); \
    strength_t sAB = linkAB->getMean(); \
    strength_t sBC = linkBC->getMean(); \
    count_t nA = atomA->getCount(); \
    count_t nB = atomB->getCount(); \
    count_t nC = atomC->getCount(); \
    count_t nAB = linkAB->getCount(); \
    count_t nBC = linkBC->getCount();

#define DebugPLNBodyFor_Link if (PLNdebug) cprintf(-3, " A->B:%f\n",  sAB);

#define DebugPLNBodyFor_Link2 if (PLNdebug) cprintf(-3, "sA:%f sB:%f \n", sA, sB);

#define DebugPLNBodyFor_Link1Node2 if (PLNdebug) cprintf(-3, "sA:(%f,%f) sB:(%f,%f) A->B:(%f,%f)\n", sA, nA, sB, nB, sAB, nAB);

#define DebugPLNBodyFor_Link2Node2 if (PLNdebug) cprintf(-3, "sA:%f sB:%f   A->B:%f B->C:%f\n", sA, sB, sAB, sBA);

#define DebugPLNBodyFor_Link2Node3 if (PLNdebug) cprintf(-3, "sA:%f sB:%f sC:%f   A->B:%f B->C:%f\n", sA, sB, sC, sAB, sBC);



//===========================================================================//
TruthValue* IdentityFormula::simpleCompute(TruthValue** TV,
                                           int N, long U) const
{
    assert(N > 0);
    return TV[0]->clone();
}

/*=============================================================================
    simpleCompute() methods take parameters in the order:
    {link Tvalues}, { node Tvalues }
=============================================================================*/

//===========================================================================//
TruthValue* InversionFormula::simpleCompute(TruthValue** TV,
                                            int N, long U) const
{
    cprintf(-3, "Invert...\n");

    ///IndefiniteTV check
    if (isAllIndefiniteTruthValueType(TV, N)) {
        cprintf(-3, "IndefiniteSymmetricBayesFormula\n");
        return IndefiniteSymmetricBayesFormula().simpleCompute(TV, N, U);
    }

    PLNFormulaBodyFor_Link1Node2;
    DebugPLNBodyFor_Link1Node2;

    strength_t sBA = sAB * sA / std::max(sB, 0.00001f);
    count_t nBA = nAB * nB / std::max(nA, 0.00001f);

    return checkTruthValue( new SimpleTruthValue(sBA, nBA) );
}

//===========================================================================//
TruthValue* ImplicationBreakdownFormula::simpleCompute(TruthValue** TV,
                                                       int N, long U) const
{
    cprintf(-3, "InferenceMindAgent::ImplicationBreakdown\n");

    ///IndefiniteTV check
    if (isAllIndefiniteTruthValueType(TV, N)) {
        cprintf(-3, "IndefiniteSymmetricImplicationBreakdown\n");
        return IndefiniteSymmetricImplicationBreakdownFormula().simpleCompute(TV, N, U);
    }

    PLNFormulaBodyFor_Link1Node2;
    DebugPLNBodyFor_Link1Node2;

    count_t n2 = std::min(nAB, nA);

    // sB is used for P(B|NOT A)
    // so it must assume that B and NOT A are independent
    // @todo, we should make the formula look more general with sB
    // being possibly P(B|NOT A), and the caller of that formula
    // would the one deciding what is the assumption
    strength_t s2 = ((n2 + nB) > 0)
        ? ( (sAB * sA * n2 + sB * (1 - sA) * nB)
            / std::max((n2 + nB), 0.00001f) )
        : sB;

    if (n2 < nB) {
        s2 = sB;
        n2 = nB;
    }

    cprintf(-3, "ImBD: %f %f %f %f %f\n", s2, n2, (sAB * sA * n2 + sB*nB), sA, sB);

    return checkTruthValue(  new SimpleTruthValue(s2, n2) );
}

//=======================================================================================//
TruthValue* ImplicationConstructionFormula::simpleCompute(TruthValue** TV,
                                                          int N, long U) const
{
    cprintf(-3, "InferenceMindAgent::ImplicationConstruction\n");

    PLNFormulaBodyFor_Link1Node2;
    DebugPLNBodyFor_Link1Node2;

    strength_t s2 = ((sA > 0) ? (sAB / sA) : 0);


    return checkTruthValue(  new SimpleTruthValue(s2, nAB) );
}

//===========================================================================//
TruthValue* NotFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "InferenceMindAgent::NotEvaluation\n");

    PLNFormulaBodyFor_Atom2;
    DebugPLNBodyFor_Link2;

    /*    float s2 = (nA*sA + 1 - nB*sB) / (nA + nB); // s(LINK) - s(NOT)
        if (s2 < 0.0f) s2 = 0.0f;
        float n2 = (nA - nB);
        if (n2 < 0.0f) n2 = 0.0f;*/

    return checkTruthValue(  new SimpleTruthValue(1.0f - sB, nB) );
}

//===========================================================================//
TruthValue* DeductionSimpleFormula::simpleCompute(TruthValue** TV,
                                                  int N, long U) const
{
    cprintf(-3, "InferenceMindAgent::deduct(handleA, linkType, nodeType)\n");

    ///IndefiniteTV check
    if (isAllIndefiniteTruthValueType(TV, N)) {
        cprintf(-3, "IndefiniteSymmetricDeductionFormula\n");
        return IndefiniteSymmetricDeductionFormula().simpleCompute(TV, N, U);
    }

    TruthValue* linkTEMP = TV[0];

    PLNFormulaBodyFor_Link2Node3;
    DebugPLNBodyFor_Link2Node3;

    /// Temporary filtering fix to make sure that nAB >= nA
    nA = std::min(nA, nAB);

    strength_t secondDenominator = std::max( (1 - sB), TV_MIN);
    count_t thirdDenominator = std::max( nB, TV_MIN);

    strength_t w1 = DEDUCTION_TERM_WEIGHT;
    strength_t w2 = 2 - w1;
    strength_t sAC =
        w1 * sAB * sBC 
        + w2 * (1 - sAB) * (sC - sB * sBC) / secondDenominator;

    count_t nAC = IndependenceAssumptionDiscount * nA * nBC / thirdDenominator;

    return checkTruthValue(  new SimpleTruthValue(sAC, nAC) );
}

//===========================================================================//
strength_t DeductionGeometryFormula::g(strength_t sA, strength_t sB,
                                       strength_t sC, strength_t sAB) const
{
    strength_t p1 = std::abs(sA - sB) / std::max(sA + sB, TV_MIN);
    strength_t p2 =
        1
        - std::min(1 + sB - sAB * sA - sC, sA + sB - sAB * sA)
        + std::max(sB, 1 - sC);

    return p1 + (1 - p1)*p2;
}

//===========================================================================//
strength_t DeductionGeometryFormula::g2(strength_t sA, strength_t sB,
                                        strength_t sC, strength_t sAB) const
{
    return 1.0f;
}

//===========================================================================//
TruthValue* DeductionGeometryFormula::simpleCompute(TruthValue** TV,
                                                    int N, long U) const
{
    cprintf(-3, "Geom: deduct\n");

    PLNFormulaBodyFor_Link2Node3;
    DebugPLNBodyFor_Link2Node3;

    TruthValue* baTV[] = { linkAB, atomA, atomB };
    TruthValue* linkBA = InversionFormula::simpleCompute(baTV, 3);

    strength_t sBA = linkBA->getMean();

    strength_t sAC =
        sAB * sBC
        * (1 + g(sA, sB, sC, sAB)
           * std::max(0.0f, -1 + 1 / std::max(sBA + sBC, TV_MIN)))
        + (1 - sAB) * (1 - sBC)
        * (1 + g(sA, 1 - sB, sC, sAB)
           * std::max(0.0f, -1 + 1 / std::max(1 - sBA + 1 - sBC,
                                              TV_MIN)));
    strength_t nAC =
        IndependenceAssumptionGeometryDiscount * nA * nBC
        / std::max(nB, TV_MIN);

    return NULL;
    //    return checkTruthValue(  new SimpleTruthValue(sAC,nAC) );
}

//===========================================================================//
TruthValue* RevisionFormula::simpleCompute(TruthValue** TV,
                                           int N, long U) const
{
    cprintf(-3, "Revision...\n");

    ///IndefiniteTV check
    if (isAllIndefiniteTruthValueType(TV, N)) {
        cprintf(-3, "IndefiniteSymmetricRevisionFormula\n");
        return IndefiniteSymmetricRevisionFormula().simpleCompute(TV, N, U);
    }

    PLNFormulaBodyFor_Atom2;
    //    DebugPLNBodyFor_Atom2;

    strength_t wA = nA / std::max(nA + nB, TV_MIN);
    count_t wB = nB / std::max(nA + nB, TV_MIN);

    strength_t c_strength = REVISION_STRENGTH_DEPENDENCY;
    count_t c_count = REVISION_COUNT_DEPENDENCY;

    strength_t s3 = (wA * sA + wB * sB - c_strength * sA * sB);
    count_t n3 = std::max(nA, nB) + c_count * std::min(nA, nB);

    return checkTruthValue( new SimpleTruthValue(s3, n3) );
}

//===========================================================================//
TruthValue* Inh2SimFormula::simpleCompute(TruthValue** TV,
                                          int N, long U) const
{
    cprintf(-3, "Inh2Sim...\n");

    PLNFormulaBodyFor_Link2Node2;
    DebugPLNBodyFor_Link2Node2;

    strength_t sABsim =
        1 / ( ( 1 + sA / std::max(sB, TV_MIN)) / std::max(sAB - 1, TV_MIN));

    count_t nABsim = nAB + nBA - sAB * nA;

    return checkTruthValue( new SimpleTruthValue(sABsim, nABsim) );
}

//===========================================================================//
TruthValue* Sim2InhFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "Sim2Inh...\n");

    PLNFormulaBodyFor_Link1Node2;
    DebugPLNBodyFor_Link1Node2;

    strength_t sABinh =
        (1 + sB / std::max(sA, TV_MIN)) * sAB / (1 + std::max(sAB, TV_MIN));

    count_t nABinh =  (nAB + sAB * nA) * nA / std::max( nA + nB, TV_MIN );

    return checkTruthValue( new SimpleTruthValue(sABinh, nABinh) );
}

//===========================================================================//
TruthValue* ANDBreakdownFormula::simpleCompute(TruthValue** TV,
                                               int N, long U) const
{
    cprintf(-3, "ANDbreak...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sA = sAB;
    count_t nA = nAB;

    return checkTruthValue( new SimpleTruthValue(sA, nA) );
}

//===========================================================================//
TruthValue* ModusPonensFormula::simpleCompute(TruthValue** TV,
                                              int N, long U) const
{
    cprintf(-3, "Modus Ponens...\n");

    PLNFormulaBodyFor_Atom2;
    //    DebugPLNBodyFor_atom2;

    // Note that sB corresponds to sAB
    // DefaultNodeProbability is supposed to replace the unknown P(B|NOT A)
    strength_t sC = sA * sB + DefaultNodeProbability * (1 - sA);
    count_t nC = nA;

    return checkTruthValue( new SimpleTruthValue(sC, nC) );
}

//===========================================================================//
TruthValue* Inh2ImpFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "Inh2Imp...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Imp2InhFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "Imp2Inh...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    //    float sAB = sAB;
    //    float nAB = nAB;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Mem2InhFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "Mem2Inh...\n");

    ///IndefiniteTV check
    if (isAllIndefiniteTruthValueType(TV, N)) {
        cprintf(-3, "IndefiniteMem2InhFormula\n");
        return IndefiniteMem2InhFormula().simpleCompute(TV, N, U);
    }


    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sABinh = sAB;
    count_t nABinh = nAB * MembershipToExtensionalInheritanceCountDiscountFactor;

    return checkTruthValue( new SimpleTruthValue(sABinh, nABinh) );
}

//===========================================================================//
TruthValue* Mem2EvalFormula::simpleCompute(TruthValue** TV,
                                           int N, long U) const
{
    cprintf(-3, "Mem2Eval...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    //    float sABext = sAB;
    //float nABext = nAB;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Eval2InhFormula::simpleCompute(TruthValue** TV,
                                           int N, long U) const
{
    cprintf(-3, "Eval2Inh...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    //    float sABext = sAB;
    //float nABext = nAB;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Ext2IntFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "Ext2Int...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sABint = sAB;
    count_t nABint = nAB * ExtensionToIntensionCountDiscountFactor;

    return checkTruthValue( new SimpleTruthValue(sABint, nABint) );
}

//===========================================================================//
TruthValue* Int2ExtFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "Int2Ext...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sABint = sAB;
    count_t nABint = nAB * IntensionToExtensionCountDiscountFactor;

    return checkTruthValue( new SimpleTruthValue(sABint, nABint) );
}

//===========================================================================//
TruthValue* SymmetricANDFormula::simpleCompute(TruthValue** TV,
                                               int N, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Logical AND...\n");
    cprintf(-3, "SymmetricAND...\n");

    /* Indefinite Formula */
    if (isAllIndefiniteTruthValueType(TV, N)) {
        cprintf(-3, "IndefiniteSymmetricANDFormula\n");
        if (N == 2) {
            return IndefiniteSymmetricANDFormula().simpleCompute(TV, N, U);
        }

        IndefiniteTruthValue* _TV[2];
        IndefiniteTruthValue* result = (IndefiniteTruthValue*)TV[0];
        for (int i = 1; i < N; i++) {
            _TV[0] = result;
            _TV[1] = (IndefiniteTruthValue*)TV[i];
            result = (IndefiniteTruthValue*)IndefiniteSymmetricANDFormula().simpleCompute((TruthValue**)_TV, 2, U);
        }
        return result;
    }
    /* End of Indefinite Formulas */


    strength_t sTot = 1.0f;
    confidence_t conTot = 1.0f;

    for (int i = 0; i < N; i++) {
        cprintf(-3, "%f,%f & ", TV[i]->getMean(), TV[i]->getConfidence());
        sTot *= TV[i]->getMean();
        conTot *= TV[i]->getConfidence();
    }

    strength_t sAND = sTot;
    count_t nAND = SimpleTruthValue::confidenceToCount(conTot);
    //float KKK = IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL;
    //KKK * conTot / (1 - conTot); /// The standard count=>confidence formula!

//  cprintf(-3, " = %f\n", sAND);

//    assert(nTot <= 1.0f);

    TruthValue* retTV = new SimpleTruthValue(sAND, nAND);


    return checkTruthValue(retTV);
}

//===========================================================================//
TruthValue* AsymmetricANDFormula::simpleCompute(TruthValue** TV,
                                                int N, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Logical AND2...\n");

    PLNFormulaBodyFor_Atom2;
    //    DebugPLNBodyFor_Atom2;

    strength_t sAND = sA * sB;
    count_t nAND = nB;

    return checkTruthValue( new SimpleTruthValue(sAND, nAND) );
}

//===========================================================================//
TruthValue* OldANDFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "OldAND...\n");


    strength_t mean = TV[0]->getMean() * TV[1]->getMean();
    count_t count1 = TV[0]->getCount();
    count_t count2 = TV[1]->getCount();
    count_t count = count1 > count2 ? count2 : count1;

    TruthValue* retTV = new SimpleTruthValue(mean, count);

    return checkTruthValue(retTV);
}

//===========================================================================//
TruthValue* ORFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3,  "Logical OR with : ");
    for (int k = 0;k < N;k++)
        cprintf(-3,  "#%d:%s ", k, TV[k]->toString().c_str());
    cprintf(-3, "\n");

    auto_ptr<TruthValue> res1, res2;

    if (N > 2) {
//  TVType* next[2];

        int N1 = (int)(N / 2);
        int N2 = (int)(N / 2.0 + 0.501);

        cprintf(-3,  "Division: %d - %d\n", N1, N2);

        if (N1 == 1) { //Either (>1, >1) or (1, >1).
            TV[1] = simpleCompute(&TV[N1], N2, U);

            res1 = auto_ptr<TruthValue>(TV[1]);
        } else {
            TV[0] = simpleCompute(TV, N1, U);
            TV[1] = simpleCompute(&TV[N1], N2, U);

            res1 = auto_ptr<TruthValue>(TV[0]);
            res2 = auto_ptr<TruthValue>(TV[1]);
        }
    }

    strength_t sTot = 0.0f;
    count_t nTot = 0.0f;

    for (int i = 0; i < 2; i++)
        sTot += TV[i]->getMean();

    strength_t sA = TV[0]->getMean();
    strength_t sB = TV[1]->getMean();
    count_t nA = TV[0]->getCount();
    count_t nB = TV[1]->getCount();

    count_t A = sA * nB;
    count_t B = sB * nA;

    nTot = nA + nB - (A + B) / 2;

    cprintf(-3,  "nA=%.4f nB=%.4f nTot=%.4f\n", nA, nB, nTot);

    return checkTruthValue( new SimpleTruthValue(sTot, nTot) );
}

//===========================================================================//
TruthValue* ExcludingORFormula::simpleCompute(TruthValue** TV,
                                              int N, long U) const
{
//  LOG(3, "Logical OR...\n");
//  int level1N = haxx::contractInclusionExclusionFactorial(N); //2*(int)(sqrt(N)); //inverse of N*(N-1)/2

    ///N must be a square of an integery, namely (I + I*(I-1)) = I*I

    int level1N = 2;
    assert(N == 3);

    assert(level1N*(level1N + 1) / 2 == N);

    strength_t sTot = 0.0f;
    count_t nTot = 0.0f;

    long ptr = 0;

    for (int i = 0; i < level1N; i++) {
//      cprintf(-3, "%f & ", TV[i]->getMean());
        sTot += TV[ptr]->getMean();
//      nTot += TV[ptr]->getCount();
        ptr++;
    }
    for (int k = 0; k < level1N - 1; k++)
        for (int j = k + 1; j < level1N; j++) {
            if (ptr >= N)
                break;
            sTot -= TV[ptr]->getMean();
//          nTot += TV[ptr]->getCount();
            ptr++;
        }

    if (level1N != 2) {
        assert(0);
        nTot = 0.0f;
        //LOG(0, "OR RULE WRONG NR OF ARGS");
    } else {
        strength_t sA = TV[0]->getMean();
        strength_t sB = TV[1]->getMean();
        count_t nA = TV[0]->getCount();
        count_t nB = TV[1]->getCount();

        count_t A = sA * nB;
        count_t B = sB * nA;

        nTot = nA + nB - (A + B) / 2;
    }

    strength_t sOR = sTot;
    confidence_t KKK = IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL;
    count_t nOR = nTot  * KKK;

//  cprintf(-3, " = %f\n", sAND);

    return checkTruthValue( new SimpleTruthValue(sOR, nOR) );
}

//===========================================================================//
TruthValue* NOTFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Logical NOT...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sNOT = 1 - sAB;
    count_t nNOT = nAB;

    return checkTruthValue( new SimpleTruthValue(sNOT, nNOT) );
}

//===========================================================================//
TruthValue* ORFormula2::simpleCompute(TruthValue** TV, int N, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Logical OR...\n");

    NOTFormula notF;
    TruthValue** NOT_TVs = notF.multiCompute(TV, N, U);
    TruthValue* NOT_ANDTV = SymmetricANDFormula().simpleCompute(NOT_TVs, N, U);
    TruthValue* ORTV = notF.simpleCompute( &NOT_ANDTV, 1, U );

    delete NOT_ANDTV;

    for (int i = 0; i < N; i++)
        delete NOT_TVs[i];
    delete[] NOT_TVs;

    return ORTV;
}

//===========================================================================//
TruthValue* OldORFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    cprintf(-3, "OldOR...\n");

    strength_t mean = 1 - ((1 - TV[0]->getMean()) * (1 - TV[1]->getMean()));
    count_t count1 = TV[0]->getCount();
    count_t count2 = TV[1]->getCount();
    count_t count = count1 > count2 ? count2 : count1;

    TruthValue* retTV = new SimpleTruthValue(mean, count);

    return checkTruthValue(retTV);
}

/*=============================================================================
   Params: {set1, set2} where sizes are equal.
=============================================================================*/

//===========================================================================//
TruthValue* SubsetEvalFormula::compute(TruthValue** TVs, int N, long U) const
{
    OC_ASSERT((N % 2) == 0, "N = %d must be pair", N);

    if (N == 0) {
        cprintf(0, "SubsetEval: No MemberLinks available");
        return new SimpleTruthValue(0,0);
    }

    int Nsub = N/2;
    int Nsuper = Nsub;

    TruthValue** TVsub = TVs;
    TruthValue** TVsuper = TVsub + Nsub;

    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "SubSetEval...\n");

    strength_t fs = 0.0f, s = 0.0f;

    for (int i = 0; i < Nsuper; i++) {
        fs += f(TVsub[i]->getMean(), TVsuper[i]->getMean());
        s += TVsub[i]->getMean();
    }

    strength_t sSS = fs / std::max(s, TV_MIN);
    count_t nSS = (count_t)Nsuper;

    return new SimpleTruthValue(sSS, nSS);
    //  return checkTruthValue( new SimpleTruthValue(sSS, nSS) );
}

//===========================================================================//
strength_t SubsetEvalFormulaTimes::f(strength_t a, strength_t b) const
{
    return a*b;
}

//===========================================================================//
strength_t SubsetEvalFormulaMin::f(strength_t a, strength_t b) const
{
    return std::min(a, b);
}

/*=============================================================================
   Args: The list of truthvalues of atoms for which the predicate expression
   has been evaluated
=============================================================================*/

//===========================================================================//
TruthValue* FORALLFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "For all...\n");

    strength_t sForAll = 1.0f;
    count_t nForAll = 0.0f;
    count_t wForAll = 0.0f;

    for (int i = 0; i < N; i++) {
        count_t n_i = TV[i]->getCount();
        count_t w_i = sqrt(n_i);
        nForAll += n_i;
        wForAll += w_i;
        sForAll += (TV[i]->getMean() * w_i);
    }

    return checkTruthValue( new SimpleTruthValue(sForAll / wForAll, nForAll) );
}

//===========================================================================//
TruthValue* PredicateTVFormula::simpleCompute(TruthValue** TV,
                                              int N, long U) const
{
    strength_t sForAll = 0.0f;
    count_t nForAll = 0.0f;

    for (int i = 0; i < N; i++) {
        count_t n_i = TV[i]->getCount();
        nForAll += n_i;
        sForAll += (TV[i]->getMean() * n_i);
    }

    return checkTruthValue( new SimpleTruthValue(sForAll / nForAll, nForAll) );
}

//===========================================================================//
TruthValue* EXISTFormula::simpleCompute(TruthValue** TV, int N, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Exists...\n");

    NotFormula notF;
    TruthValue** NOT_TVs = notF.multiCompute(TV, N, U);
    TruthValue* NOT_EXTV = FORALLFormula().simpleCompute(NOT_TVs, N, U);
    TruthValue* EXTV = notF.simpleCompute( &NOT_EXTV, 1, U );

    delete NOT_EXTV;

    for (int i = 0; i < N; i++)
        delete NOT_TVs[i];
    delete[] NOT_TVs;

    return EXTV;

    /*    float sTemp = 1.0f;

    for (int i = 0; i < N; i++)
    sTemp *= (1-TV[i]->getMean());

    float sEx = 1-sTemp;

    return checkTruthValue( new SimpleTruthValue(sEx, N) );*/
}

//===========================================================================//
TruthValue* InhSubstFormula::simpleCompute(TruthValue** TV,
                                           int N, long U) const
{
    cprintf(-3, "InferenceMindAgent::InhSubstFormula\n");

    PLNFormulaBodyFor_Atom2;
//    DebugPLNBodyFor_Atom2;

    //float s2 = ((sA>0) ? (sAB / sA) : 0);

    count_t k = 800;
    float d1 = nA / (nA + k);
    float c_PAT = 0.75;

    count_t n3 = c_PAT * nB * d1 * sA;
    strength_t s3 = sB;

    return checkTruthValue(  new SimpleTruthValue(s3, n3) );
}

/*=============================================================================
   Implementation of the methods from Formula.h file
=============================================================================*/

//===========================================================================//
template<int _TVN>
TruthValue* Formula<_TVN>::simpleCompute(TruthValue** TV, int N, long U) const
{
    //cprintf(-3, "Formula<_TVN>::simpleCompute(N = %d)\n", N);
    return this->simpleCompute(TV, N, U);
}

//===========================================================================//
template<int _TVN>
TruthValue** Formula<_TVN>::multiCompute(TruthValue** TV, int N, long U) const
{
    //cprintf(-3, "Formula<_TVN>::multiCompute(N = %d)\n", N);
    assert(!(TVN / N));

    TruthValue** ret = new TruthValue*[N/TVN];

    for (int group = 0; group < N / TVN; group++) {
        ret[group] = simpleCompute(&TV[group*TVN], TVN, U);
    }

    return ret;
}

//===========================================================================//
template<int _TVN>
TruthValue* Formula<_TVN>::compute(TruthValue** TV, int N, long U) const
{
    //cprintf(-3, "Formula<_TVN>::compute(N = %d)\n", N);
    TruthValue* result;
    // computation with primary TVS
    TruthValue* primaryResult = this->simpleCompute(TV, N, U);
    // Check for existence of CompositeTruthValues and makes computation
    // for each VersionHandle
    bool hasCompositeTruthValue = false;
    std::set<VersionHandle> versionHandles;
    for (int i = 0; i < N; i++) {
        if (TV[i]->getType() == COMPOSITE_TRUTH_VALUE) {
            hasCompositeTruthValue = true;
            CompositeTruthValue* ctv = (CompositeTruthValue*) TV[i];
            versionHandles.insert(ctv->vh_begin(), ctv->vh_end());
        }
    }
    // Temporary ForwardChainer demo hack
    if (0) { //hasCompositeTruthValue) {
        CompositeTruthValue* compositeResult =
            new CompositeTruthValue(*primaryResult, NULL_VERSION_HANDLE);
        delete primaryResult;
        foreach(VersionHandle vh, versionHandles) {
            std::auto_ptr<TruthValue*> versionedTVs(new TruthValue*[N]);
            int numberOfCompatibleTvs = 0;
            for (int i = 0; i < N; i++) {
                if (TV[i]->getType() == COMPOSITE_TRUTH_VALUE) {
                    const TruthValue& versionedTV =
                        ((CompositeTruthValue*) TV[i])->getVersionedTV(vh);
                    if (!versionedTV.isNullTv()) {
                        versionedTVs.get()[numberOfCompatibleTvs++]
                            = (TruthValue*) & versionedTV;
                    }
                }
            }
            TruthValue* versionedResult = this->simpleCompute(versionedTVs.get(), numberOfCompatibleTvs, U);
            compositeResult->setVersionedTV(*versionedResult, vh);
            delete versionedResult;
        }
        result = compositeResult;
    } else {
        result = primaryResult;
    }
    return result;
}

// All instantiations of Formula<_TVN> so that link error
// (undefined reference) does not happen
template TruthValue* Formula<1>::compute(TruthValue** TV, int N, long U) const;
template TruthValue* Formula<2>::compute(TruthValue** TV, int N, long U) const;
template TruthValue* Formula<3>::compute(TruthValue** TV, int N, long U) const;
template TruthValue* Formula<4>::compute(TruthValue** TV, int N, long U) const;
template TruthValue* Formula<5>::compute(TruthValue** TV, int N, long U) const;
template TruthValue* Formula<AND_MAX_ARITY>::compute(TruthValue** TV, int N, long U) const;
// AND_MAX_ARITY == OR_MAX_ARITY == FORALL_MAX_ARITY
//template TruthValue* Formula<OR_MAX_ARITY>::compute(TruthValue** TV, int N, long U) const;
//template TruthValue* Formula<FORALL_MAX_ARITY>::compute(TruthValue** TV, int N, long U) const;


}} //namespace opencog::pln
