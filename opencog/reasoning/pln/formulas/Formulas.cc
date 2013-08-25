/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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
#include <assert.h>
#include <memory>
#include <algorithm>

#include <opencog/util/numeric.h>
#include <opencog/util/macros.h>

#include "../PLN.h"
#include "FormulasIndefinite.h"
#include "Formulas.h"
#include "../PLNUtils.h"

using namespace std;

const bool PLNdebug = true;

//namespace haxx
//{
//    int contractInclusionExclusionFactorial(int total);
//};

///fabricio: method that checks if all TV are of indefinite type (return false otherwise)
///In order to avoid "multiple definition" error and to not create a definition file only to
///definine this function, I declared it as inline
inline bool isAllIndefiniteTruthValueType(const TVSeq& TV)
{
    for (unsigned int i = 0; i < TV.size(); i++) {
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

#define PLNFormulaBodyFor_Atom \
    assert(TV.size() == 1);   \
    assert(TV[0]); \
    OC_ASSERT(!TV[0]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    strength_t sA = TV[0]->getMean(); \
    count_t nA = TV[0]->getCount(); \

#define PLNFormulaBodyFor_Link \
    assert(TV.size() == 1);   \
    assert(TV[0]); \
    OC_ASSERT(!TV[0]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    TruthValuePtr linkAB = TV[0]; \
    strength_t sAB = linkAB->getMean(); \
    count_t nAB = linkAB->getCount(); \
     
#define PLNFormulaBodyFor_Atom2 \
    assert(TV.size() == 2); \
    assert(TV[0]); \
    assert(TV[1]); \
    OC_ASSERT(!TV[0]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[1]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    strength_t sA = TV[0]->getMean(); \
    count_t nA = TV[0]->getCount(); \
    strength_t sB = TV[1]->getMean(); \
    count_t nB = TV[1]->getCount(); \
     
#define PLNFormulaBodyFor_Link1Node2 \
    assert(TV.size() == 3); \
    assert(TV[0]); \
    assert(TV[1]); \
    assert(TV[2]); \
    OC_ASSERT(!TV[0]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[1]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[2]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    TruthValuePtr linkAB = TV[0]; \
    TruthValuePtr atomA = TV[1]; \
    TruthValuePtr atomB = TV[2]; \
    strength_t sAB = linkAB->getMean(); \
    strength_t sA = atomA->getMean(); \
    strength_t sB = atomB->getMean(); \
    count_t nAB = linkAB->getCount(); \
    count_t nA = atomA->getCount(); \
    count_t nB = atomB->getCount();

#define PLNFormulaBodyFor_Link2Node2 \
    assert(TV.size() == 4); \
    assert(TV[0]); \
    assert(TV[1]); \
    assert(TV[2]); \
    assert(TV[3]); \
    OC_ASSERT(!TV[0]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[1]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[2]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[3]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    TruthValuePtr linkAB = TV[0]; \
    TruthValuePtr linkBA = TV[1]; \
    TruthValuePtr atomA = TV[2]; \
    TruthValuePtr atomB = TV[3]; \
    strength_t sAB = linkAB->getMean(); \
    strength_t sBA = linkBA->getMean(); \
    strength_t sA = atomA->getMean(); \
    strength_t sB = atomB->getMean(); \
    count_t nAB = linkAB->getCount(); \
    count_t nBA = linkBA->getCount(); \
    count_t nA = atomA->getCount(); \
    count_t nB = atomB->getCount();

#define PLNFormulaBodyFor_Link2Node3 \
    assert(TV.size() == 5); \
    assert(TV[0]); \
    assert(TV[1]); \
    assert(TV[2]); \
    assert(TV[3]); \
    assert(TV[4]); \
    OC_ASSERT(!TV[0]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[1]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[2]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[3]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    OC_ASSERT(!TV[4]->isNullTv(), "null TV, maybe you are doing contextual reasoning and the TV for that context has not been defined"); \
    TruthValuePtr linkAB = TV[0]; \
    TruthValuePtr linkBC = TV[1]; \
    TruthValuePtr atomA = TV[2]; \
    TruthValuePtr atomB = TV[3]; \
    TruthValuePtr atomC = TV[4]; \
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
TruthValue* IdentityFormula::simpleCompute(const TVSeq& TV, long U) const
{
    assert(TV.size() > 0);
    return TV[0]->clone();
}

/*=============================================================================
    simpleCompute() methods take parameters in the order:
    {link Tvalues}, { node Tvalues }
=============================================================================*/

//===========================================================================//
TruthValue* InversionFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Invert...\n");

    // @todo remove once Indefinitizer is well defined
    ///IndefiniteTV check
    // if (isAllIndefiniteTruthValueType(TV)) {
    //     cprintf(-3, "IndefiniteSymmetricBayesFormula\n");
    //     return IndefiniteSymmetricBayesFormula().simpleCompute(TV, U);
    // }

    PLNFormulaBodyFor_Link1Node2;
    DebugPLNBodyFor_Link1Node2;

    strength_t sBA = sAB * sA / std::max(sB, 0.00001f);
    count_t nBA = nAB * nB / std::max(nA, 0.00001f);

    return checkTruthValue( new SimpleTruthValue(sBA, nBA) );
}

//===========================================================================//
TruthValue* ImplicationBreakdownFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "InferenceMindAgent::ImplicationBreakdown\n");

    // @todo remove once Indefinitizer is defined
    ///IndefiniteTV check
    // if (isAllIndefiniteTruthValueType(TV)) {
    //     cprintf(-3, "IndefiniteSymmetricImplicationBreakdown\n");
    //     return IndefiniteSymmetricImplicationBreakdownFormula().simpleCompute(TV, U);
    // }

    PLNFormulaBodyFor_Link1Node2;
    DebugPLNBodyFor_Link1Node2;

    count_t n2 = std::min(nAB, nA);

    // sB is used for P(B|Not A)
    // so it must assume that B and Not A are independent
    // @todo, we should make the formula look more general with sB
    // being possibly P(B|Not A), and the caller of that formula
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
TruthValue* ImplicationConstructionFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "InferenceMindAgent::ImplicationConstruction\n");

    PLNFormulaBodyFor_Link1Node2;
    DebugPLNBodyFor_Link1Node2;

    strength_t s2 = ((sA > 0) ? (sAB / sA) : 0);


    return checkTruthValue(  new SimpleTruthValue(s2, nAB) );
}

//===========================================================================//
TruthValue* NotFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "InferenceMindAgent::NotRule\n");

    PLNFormulaBodyFor_Atom;

    return checkTruthValue(  new SimpleTruthValue(1.0f - sA, nA) );
}

//===========================================================================//
TruthValue* DeductionSimpleFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "InferenceMindAgent::deduct(handleA, linkType, nodeType)\n");

    /// @todo remove once Indefinitizer is here
    ///IndefiniteTV check
    // if (isAllIndefiniteTruthValueType(TV, N)) {
    //     cprintf(-3, "IndefiniteSymmetricDeductionFormula\n");
    //     return IndefiniteSymmetricDeductionFormula().simpleCompute(TV, U);
    // }

    TruthValuePtr linkTEMP = TV[0];

    PLNFormulaBodyFor_Link2Node3; OC_UNUSED(nC);
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
TruthValue* DeductionGeometryFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Geom: deduct\n");

    PLNFormulaBodyFor_Link2Node3; OC_UNUSED(nC); OC_UNUSED(nAB);
    DebugPLNBodyFor_Link2Node3;

    TVSeq baTV(1, linkAB);
    baTV.push_back(atomA);
    baTV.push_back(atomB);
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

	OC_UNUSED(sAC); OC_UNUSED(nAC);
    return NULL; // @todo ???
    //    return checkTruthValue(  new SimpleTruthValue(sAC,nAC) );
}

//===========================================================================//
TruthValue* RevisionFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Revision...\n");
    
    /// @todo remove once we have Indefinitizer
    ///IndefiniteTV check
    // if (isAllIndefiniteTruthValueType(TV)) {
    //     cprintf(-3, "IndefiniteSymmetricRevisionFormula\n");
    //     return IndefiniteSymmetricRevisionFormula().simpleCompute(TV, U);
    // }

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
TruthValue* Inh2SimFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Inh2Sim...\n");

    PLNFormulaBodyFor_Link2Node2; OC_UNUSED(nB);
    DebugPLNBodyFor_Link2Node2;

    strength_t sABsim =
        1 / ( ( 1 + sA / std::max(sB, TV_MIN)) / std::max(sAB - 1, TV_MIN));

    count_t nABsim = nAB + nBA - sAB * nA;

    return checkTruthValue( new SimpleTruthValue(sABsim, nABsim) );
}

//===========================================================================//
TruthValue* Sim2InhFormula::simpleCompute(const TVSeq& TV, long U) const
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
TruthValue* AndBreakdownFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Andbreak...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sA = sAB;
    count_t nA = nAB;

    return checkTruthValue( new SimpleTruthValue(sA, nA) );
}

//===========================================================================//
TruthValue* ModusPonensFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Modus Ponens...\n");

    PLNFormulaBodyFor_Atom2; OC_UNUSED(nB);
    //    DebugPLNBodyFor_atom2;

    // Note that sB corresponds to sAB
    // DefaultNodeProbability is supposed to replace the unknown P(B|Not A)
    strength_t sC = sA * sB + DefaultNodeProbability * (1 - sA);
    count_t nC = nA;

    return checkTruthValue( new SimpleTruthValue(sC, nC) );
}

//===========================================================================//
TruthValue* Inh2ImpFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Inh2Imp...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Imp2InhFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Imp2Inh...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    //    float sAB = sAB;
    //    float nAB = nAB;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Mem2InhFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Mem2Inh...\n");

    /// @todo once we have Indefinitizer
    ///IndefiniteTV check
    // if (isAllIndefiniteTruthValueType(TV)) {
    //     cprintf(-3, "IndefiniteMem2InhFormula\n");
    //     return IndefiniteMem2InhFormula().simpleCompute(TV, U);
    // }


    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sABinh = sAB;
    count_t nABinh = nAB * MembershipToExtensionalInheritanceCountDiscountFactor;

    return checkTruthValue( new SimpleTruthValue(sABinh, nABinh) );
}

//===========================================================================//
TruthValue* Mem2EvalFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Mem2Eval...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    //    float sABext = sAB;
    //float nABext = nAB;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Eval2InhFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Eval2Inh...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    //    float sABext = sAB;
    //float nABext = nAB;

    return checkTruthValue( new SimpleTruthValue(sAB, nAB) );
}

//===========================================================================//
TruthValue* Ext2IntFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Ext2Int...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sABint = sAB;
    count_t nABint = nAB * ExtensionToIntensionCountDiscountFactor;

    return checkTruthValue( new SimpleTruthValue(sABint, nABint) );
}

//===========================================================================//
TruthValue* Int2ExtFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "Int2Ext...\n");

    PLNFormulaBodyFor_Link;
    DebugPLNBodyFor_Link;

    strength_t sABint = sAB;
    count_t nABint = nAB * IntensionToExtensionCountDiscountFactor;

    return checkTruthValue( new SimpleTruthValue(sABint, nABint) );
}

//===========================================================================//
TruthValue* SymmetricAndFormula::simpleCompute(const TVSeq& TV, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Logical And...\n");
    cprintf(-3, "SymmetricAnd...\n");

    /* Indefinite Formula */
    // @todo indefinite formula should be handled using an Indefinitizer anyway

    // if (isAllIndefiniteTruthValueType(TV, N)) {
    //     cprintf(-3, "IndefiniteSymmetricAndFormula\n");
    //     if (TV.size() == 2) {
    //         return IndefiniteSymmetricAndFormula().simpleCompute(TV, U);
    //     }

    //     IndefiniteTruthValue* _TV[2];
    //     IndefiniteTruthValue* result = (IndefiniteTruthValue*)TV[0];
    //     for (int i = 1; i < N; i++) {
    //         _TV[0] = result;
    //         _TV[1] = (IndefiniteTruthValue*)TV[i];
    //         result = (IndefiniteTruthValue*)IndefiniteSymmetricAndFormula().simpleCompute((TruthValue**)_TV, 2, U);
    //     }
    //     return result;
    // }
    /* End of Indefinite Formulas */


    strength_t sTot = 1.0f;
    confidence_t conTot = 1.0f;

    for (unsigned int i = 0; i < TV.size(); i++) {
        cprintf(-3, "%f,%f & ", TV[i]->getMean(), TV[i]->getConfidence());
        sTot *= TV[i]->getMean();
        conTot *= TV[i]->getConfidence();
    }

    strength_t sAnd = sTot;
    count_t nAnd = SimpleTruthValue::confidenceToCount(conTot);
    //float KKK = IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL;
    //KKK * conTot / (1 - conTot); /// The standard count=>confidence formula!

//  cprintf(-3, " = %f\n", sAnd);

//    assert(nTot <= 1.0f);

    TruthValue* retTV = new SimpleTruthValue(sAnd, nAnd);


    return checkTruthValue(retTV);
}

//===========================================================================//
TruthValue* AsymmetricAndFormula::simpleCompute(const TVSeq& TV, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Logical And2...\n");

    PLNFormulaBodyFor_Atom2; OC_UNUSED(nA);
    //    DebugPLNBodyFor_Atom2;

    strength_t sAnd = sA * sB;
    count_t nAnd = nB;

    return checkTruthValue( new SimpleTruthValue(sAnd, nAnd) );
}

//===========================================================================//
TruthValue* OldAndFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "OldAnd...\n");


    strength_t mean = TV[0]->getMean() * TV[1]->getMean();
    count_t count1 = TV[0]->getCount();
    count_t count2 = TV[1]->getCount();
    count_t count = count1 > count2 ? count2 : count1;

    TruthValue* retTV = new SimpleTruthValue(mean, count);

    return checkTruthValue(retTV);
}

//===========================================================================//
TruthValue* OrFormula::simpleCompute(const TVSeq& TV, long U) const
{
    int N = TV.size();

    cprintf(-3,  "Logical Or with : ");
    for (int k = 0;k < N;k++)
        cprintf(-3,  "#%d:%s ", k, TV[k]->toString().c_str());
    cprintf(-3, "\n");

    TruthValuePtr res1 = TV[0];
    TruthValuePtr res2 = TV[1];
    //unique_ptr<const TruthValue> ptr1, ptr2;

    if (N > 2) {
//  TVType* next[2];

        int N1 = (int)(N / 2);
        int N2 = (int)(N / 2.0 + 0.501);

        TVSeq TV1(TV.begin(), TV.begin()+N1);
        TVSeq TV2(TV.begin()+N1, TV.begin()+N1+N2);

        cprintf(-3,  "Division: %d - %d\n", N1, N2);

        if (N1 == 1) { //Either (>1, >1) or (1, >1).
            res2 = TruthValuePtr(simpleCompute(TV2, U));
            //ptr2 = unique_ptr<const TruthValue>(res2);
        } else {
            res1 = TruthValuePtr(simpleCompute(TV1, U));
            res2 = TruthValuePtr(simpleCompute(TV2, U));
            //ptr1 = unique_ptr<const TruthValue>(res1);
            //ptr2 = unique_ptr<const TruthValue>(res2);
        }
    }

    strength_t sTot = res1->getMean() + res2->getMean();
    count_t nTot = 0.0f;

    strength_t sA = res1->getMean();
    strength_t sB = res2->getMean();
    count_t nA = res1->getCount();
    count_t nB = res2->getCount();

    count_t A = sA * nB;
    count_t B = sB * nA;

    nTot = nA + nB - (A + B) / 2;

    cprintf(-3,  "nA=%.4f nB=%.4f nTot=%.4f\n", nA, nB, nTot);

    return checkTruthValue( new SimpleTruthValue(sTot, nTot) );
}

//===========================================================================//
TruthValue* ExcludingOrFormula::simpleCompute(const TVSeq& TV, long U) const
{
//  LOG(3, "Logical Or...\n");
//  int level1N = haxx::contractInclusionExclusionFactorial(N); //2*(int)(sqrt(N)); //inverse of N*(N-1)/2

    ///N must be a square of an integery, namely (I + I*(I-1)) = I*I

    int N = TV.size();

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

    strength_t sOr = sTot;
    confidence_t KKK = IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL;
    count_t nOr = nTot  * KKK;

//  cprintf(-3, " = %f\n", sAnd);

    return checkTruthValue( new SimpleTruthValue(sOr, nOr) );
}

//===========================================================================//
TruthValue* OrFormula2::simpleCompute(const TVSeq& TV, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Logical Or...\n");

    NotFormula notF;
    TVSeq Not_TVs = notF.multiCompute(TV, U);
    TruthValuePtr Not_AndTV(SymmetricAndFormula().simpleCompute(Not_TVs, U));
    TruthValue* OrTV = notF.simpleCompute(TVSeq(1, Not_AndTV), U);

    //for (unsigned int i = 0; i < Not_TVs.size(); i++)
    //    delete Not_TVs[i];

    return OrTV;
}

//===========================================================================//
TruthValue* OldOrFormula::simpleCompute(const TVSeq& TV, long U) const
{
    cprintf(-3, "OldOr...\n");
    // No longer needed as NOT_TV entries are smart pointers
    //for (unsigned int i = 0; i < NOT_TVs.size(); i++)
    //    delete NOT_TVs[i];

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
TruthValue* SubsetEvalFormula::compute(const TVSeq& TVs, long U) const
{
    int N = TVs.size();

    OC_ASSERT((N % 2) == 0, "N = %d must be pair", N);

    if (N == 0) {
        cprintf(0, "SubsetEval: No MemberLinks available");
        return new SimpleTruthValue(0,0);
    }

    int Nsub = N/2;
    int Nsuper = Nsub;

    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "SubSetEval...\n");

    strength_t fs = 0.0f, s = 0.0f;

    for (int i = 0; i < Nsuper; i++) {
        fs += f(TVs[i]->getMean(), TVs[Nsub + i]->getMean());
        s += TVs[i]->getMean();
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
TruthValue* ForAllFormula::simpleCompute(const TVSeq& TV, long U) const
{
    int N = TV.size();

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
TruthValue* PredicateTVFormula::simpleCompute(const TVSeq& TV, long U) const
{
    strength_t sForAll = 0.0f;
    count_t nForAll = 0.0f;

    for (unsigned int i = 0; i < TV.size(); i++) {
        count_t n_i = TV[i]->getCount();
        nForAll += n_i;
        sForAll += (TV[i]->getMean() * n_i);
    }

    return checkTruthValue( new SimpleTruthValue(sForAll / nForAll, nForAll) );
}

//===========================================================================//
TruthValue* ExistFormula::simpleCompute(const TVSeq& TV, long U) const
{
    //if (Log::getDefaultLevel() >= 3) cprintf(-3, "Exists...\n");

    NotFormula notF;
    TVSeq Not_TVs = notF.multiCompute(TV, U);
    TruthValuePtr Not_EXTV(ForAllFormula().simpleCompute(Not_TVs, U));
    TruthValue* EXTV = notF.simpleCompute(TVSeq(1, Not_EXTV), U );

    return EXTV;

    /*    float sTemp = 1.0f;

    for (int i = 0; i < N; i++)
    sTemp *= (1-TV[i]->getMean());

    float sEx = 1-sTemp;

    return checkTruthValue( new SimpleTruthValue(sEx, N) );*/
}

//===========================================================================//
TruthValue* InhSubstFormula::simpleCompute(const TVSeq& TV, long U) const
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

    return checkTruthValue( new SimpleTruthValue(s3, n3) );
}

TruthValue* ContextFreeToSensitiveFormula::simpleCompute(const TVSeq& TV, long U) const {
    OC_ASSERT(TV.size() == 2);
    strength_t CX_s = TV[0]->getMean();
    confidence_t CX_c = TV[0]->getConfidence();
    strength_t A_s = TV[1]->getMean();
    confidence_t A_c = TV[1]->getConfidence();
    
    strength_t c = CX_c*A_c*(1 - binaryEntropy(CX_s)*binaryEntropy(A_s));
    count_t count = SimpleTruthValue::confidenceToCount(c);
    return checkTruthValue(new SimpleTruthValue(A_s, count));
}

/*=============================================================================
   Implementation of the methods from Formula.h file
=============================================================================*/

//===========================================================================//
template<int _TVN>
TruthValue* Formula<_TVN>::simpleCompute(const TVSeq& TV, long U) const
{
    //cprintf(-3, "Formula<_TVN>::simpleCompute(N = %d)\n", N);
    return this->simpleCompute(TV, U);
}

//===========================================================================//
template<int _TVN>
TVSeq Formula<_TVN>::multiCompute(const TVSeq& TV, long U) const
{
    int N = TV.size();
    //cprintf(-3, "Formula<_TVN>::multiCompute(N = %d)\n", N);
    assert(!(TVN / N));

    TVSeq ret(N/TVN);

    for (int group = 0; group < N / TVN; group++) {
        /// @todo maybe can be optimized
        TVSeq groupTV(TV.begin() + group*TVN, TV.begin() + group*TVN + TVN);
        ret[group] = TruthValuePtr(simpleCompute(groupTV, U));
    }

    return ret;
}

//===========================================================================//
template<int _TVN>
TruthValue* Formula<_TVN>::compute(const TVSeq& TV, long U) const
{
    int N = TV.size();
	OC_UNUSED(N);
    //cprintf(-3, "Formula<_TVN>::compute(N = %d)\n", N);

    return this->simpleCompute(TV, U);
}

// All instantiations of Formula<_TVN> so that link error
// (undefined reference) does not happen
template TruthValue* Formula<1>::compute(const TVSeq& TV, long U) const;
template TruthValue* Formula<2>::compute(const TVSeq& TV, long U) const;
template TruthValue* Formula<3>::compute(const TVSeq& TV, long U) const;
template TruthValue* Formula<4>::compute(const TVSeq& TV, long U) const;
template TruthValue* Formula<5>::compute(const TVSeq& TV, long U) const;
template TruthValue* Formula<AND_MAX_ARITY>::compute(const TVSeq& TV, long U) const;
// AND_MAX_ARITY == OR_MAX_ARITY == FORALL_MAX_ARITY
//template TruthValue* Formula<OR_MAX_ARITY>::compute(const TVSeq& TV, int N, long U) const;
//template TruthValue* Formula<FORALL_MAX_ARITY>::compute(const TVSeq& TV, int N, long U) const;


}} //namespace opencog::pln
