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

/*
Policy decisions:
- EquivalenceLink
	Exists [X for which P]
	AndLink
		MemberLink X Universe
		EvaluationLink P X
  (X is given an arbitrary label; it may turn out that it is equivalent with other things)
- EquivalenceLink
	ForAll [X P]
	IntensionalInheritanceLink Universe P

[ ForAll X P <=> -Exists X -P
<=>
EquivalenceLink
	IntensionalInheritanceLink Universe P
	-AndLink <tv>
		MemberLink X Universe
		 EvaluationLink -P X
		 = MemberLink X SatisfyingSet(-P)
	=-MemberLink X
		AndLink
			Universe
			SatisfyingSet(-P)
		=-OrLink
			-Arbitrary
			-Universe
			SatisfyingSet(P)
	= MemberLink X
		OrLink
			-Universe
			SatisfyingSet(P)
	= ImplicationLink
		MemberLink X Universe
			MemberLink X
				SatisfyingSet(P)

	[But MemberLink X Universe <1> (by def. of Universe).]

[ Useful to define
	EqLink
		Arbitrary X
		MemberLink X Universe ? ]

//[ Eg. "exists x s.t. FriendOf(A, x) && exists x s.t. FriendOf(x,B) => "
//<=> "exists x s.t. FriendOf(A, x) && FriendOf(x,B)" 

---
If for PN of arity 0, PN = CN,
and for PN with arity 1, PN X = CN (etc.),
then something like

EquivalenceLink
    EvaluationLink P $X
    IntInhLink $X P?

Ie. having the property expressed by the predicate P (of arity 1) should be
equivalent to intensionally inheriting from a CN (defined as) having the property.

"Converting" between CN and PN should be semantically transparent.

(
Then, SatisfyingSet P
)

Then, we could also say,

EquivalenceLink
    ForAll [x P(x)]
    IntensionalInheritanceLink Universe P

where Universe is a special concept that all CNs int. inherit from.
(Or, in a local case, it can be replaced by a local concept.)
---

- Each node has Identity and Number. "Exists X" <=> create a random label node,
with no identity and unknown number. What is Identity? Simply the collection of
Links!

- An internal node has no specific "identifier" apart from its connections.
An external node has an identifier. It is primitive and can be considered a "qualia":
an irreducible entity. Example qualias include GoalNodes, whose truth depends on the externally
given fitness function (and/or on other GoalNodes which eventually depend on the fitn. f.),
and the Universe node.

- The knowledge-state of the system is determined by 2 factors: the externally given
knowledge and the goal node strength values. The externally given knowledge should not be
directly substituted for self-learnt nodes, but the similarity between INVENTED link clusters
and GIVEN clusters should be realized by the system itself, making a MERGE.
//- Eg. "If event S associates with event E in a manner P, then from the presence of S I can
//infer E (as P). If E implies E2, then S implies also E." Given that S is a symbol statement
//expressed by a human by an interface system P to designated E, NM also associates 

- The seamlessness of the connection between these PLN inference control design issues and
my own concept theory is, to me, striking. The fact that on one hand we have a math.
valid inference system, and on the other hand we have a "formalized Nietzschean semantics",
integrating nicely, suggests that the design can be made extremely robust, natural and
powerful.

- Exists1 x P(x)
<=> ImplicationLink P (EquivalenceLink x1)
- ExistsN x P(x) <tv>
<=> ImplicationLink P (EquivalenceLink x1) <tv / N>

! SatisfyingSets vs. concept applicability? (X might not satisfy P nor -P.) Ie. excluded middle rule doesn't work. Avoiding the mutual existence of P and Not(P) should be a heuristic only, or handled via revision.
! Multi-deduction?
*/


#ifndef PLN_FORMULAS
#define PLN_FORMULAS

#include "Formula.h"

namespace opencog {
namespace pln {

// Must keep this up to date:
const int FORMULA_MAX_ARITY = 100;

const int AND_MAX_ARITY = FORMULA_MAX_ARITY;
const int OR_MAX_ARITY = FORMULA_MAX_ARITY;
const int FORALL_MAX_ARITY = FORMULA_MAX_ARITY;

// TV_MIN is usually used to avoid division by zero or that sort of things
const float TV_MIN = 0.000001f;

#define DEDUCTION_TERM_WEIGHT 1.0f

#define REVISION_STRENGTH_DEPENDENCY 0.0f
#define REVISION_COUNT_DEPENDENCY 0.0f

/*const float DEDUCTION_TERM_WEIGHT = 1.0f;

const float REVISION_STRENGTH_DEPENDENCY = 0.0f;
const float REVISION_COUNT_DEPENDENCY = 0.0f;
*/
const float MembershipToExtensionalInheritanceCountDiscountFactor = 1.0f;
const float IntensionToExtensionCountDiscountFactor = 1.0f;
const float ExtensionToIntensionCountDiscountFactor = 1.0f;

const float IndependenceAssumptionDiscount = 1.0f;
const float IndependenceAssumptionGeometryDiscount = 1.0f;

const float DefaultNodeProbability = 1 / DefaultU;

/**
 * Returns a clone of the input. Useful for inference rules that
 * outputs a TV identical to one of its premises.
 */ 
class IdentityFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/*=============================================================================
    simpleCompute() methods take parameters in the order:
    {link Tvalues}, { node Tvalues }
=============================================================================*/ 

/**
 * 
 */ 
class InversionFormula : public Formula<3>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class ImplicationBreakdownFormula : public Formula<3>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class ImplicationConstructionFormula : public Formula<3>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * NotFormula takes 1 TV and returns the its negation.
 * TV->mean = 1 - TV[0]->mean
 * TV->count = TV[0]->count
 */ 
class NotFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class DeductionSimpleFormula : public Formula<5>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class DeductionGeometryFormula : public Formula<5>, public InversionFormula
{
private:
    strength_t g(strength_t sA, strength_t sB,
                 strength_t sC, strength_t sAB) const;
    //strength_t g2(TruthValue* A, TruthValue* B ,TruthValue* C)
    strength_t g2(strength_t sA, strength_t sB,
                  strength_t sC, strength_t sAB) const;
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class RevisionFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Inh2SimFormula : public Formula<4>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Sim2InhFormula : public Formula<3>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class AndBreakdownFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class ModusPonensFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Inh2ImpFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Imp2InhFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Mem2InhFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Mem2EvalFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Eval2InhFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Ext2IntFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class Int2ExtFormula : public Formula<1>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * TV.strength = Prod_i TV[i].strength
 * TV.confidence = Prod_i TV[i].confidence
 */ 
class SymmetricAndFormula : public Formula<AND_MAX_ARITY>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * p(A),P(B|A)
 */ 
class AsymmetricAndFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * Formula that behaves like old TruthValue::andOperation() method.
 */
class OldAndFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class OrFormula : public Formula<OR_MAX_ARITY>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
class ExcludingOrFormula : public Formula<OR_MAX_ARITY>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * 
 */ 
//class OrFormula2 : public SymmetricAndFormula, public NotFormula
class OrFormula2 : public Formula<OR_MAX_ARITY>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * Formula that behaves like old TruthValue::orOperation() method.
 */
class OldOrFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};


/*=============================================================================
   Params: {set1, set2} where sizes are equal.
=============================================================================*/ 

/**
 * 
 */
class SubsetEvalFormula : public ArityFreeFormula
{
private:
    //by default compute uses min
    virtual strength_t f(strength_t a, strength_t b) const
    {
        return std::min(a, b);
    }
public:

    /*virtual TruthValue * compute(const TVSeq& tv1, int N1,
                                 const TVSeq& tv2, int N2) const
    {
        return NULL;
        }*/


    /**
     * it assumes that TVs = TVsub @ TVsuper and that
     * TVsub and TVsuper have same size
     *
     * Note that it is still possible to use the method with the signature
     *
     * TruthValue* compute(const TVSeq& TVsub, const TVSeq& TVsuper,
     *                     long U = DefaultU) const {
     *
     * as it is defined in formula.h
     */
    virtual TruthValue* compute(const TVSeq& TVs, long U = DefaultU) const;

};

/**
 *
 */
class SubsetEvalFormulaTimes : SubsetEvalFormula
{
protected:
    strength_t f(strength_t a, strength_t b) const;
};

/**
 *
 */
class SubsetEvalFormulaMin : SubsetEvalFormula
{
protected:
    strength_t f(strength_t a, strength_t b) const;
};

/*=============================================================================
   Args: The list of truthvalues of atoms for which the predicate expression
   has been evaluated
=============================================================================*/ 

/**
 * Compute the ForAll formula as follows (it is a heuristic and seems not
 * mathematically justified, and therefore could probably be improved)
 * TV->count = Sum_i TV[i]->count
 * TV->mean = Sum_i (TV[i]->mean * sqrt(TV[i]->count)) / total_weight
 * where total_weight = Sum_i sqrt(TV[i]->count)
 */
class ForAllFormula : public Formula<FORALL_MAX_ARITY>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 *
 */
class PredicateTVFormula : public Formula<FORALL_MAX_ARITY>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 *
 */
//class ExistFormula : public FORALLFormula, public NotFormula
class ExistFormula : public Formula<FORALL_MAX_ARITY>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 *
 */
class InhSubstFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

/**
 * Formula used in ContextFreeToSensitiveRule to determine the TV of a
 * contextual knowledge given its context free equivalent.
 *
 * Used for the inference
 *
 * C <TV1>
 * A <TV2>
 * |-
 * ContextLink <TV3>
 *     C
 *     A
 *
 * TV3.s = TV2.s
 * TV3.c = TV1.c*TV2.c*(1-H(TV1.s)*H(TV2.s))
 * 
 * That formula is a heuristic with not much justification, for an
 * accurate computation see @todo add link to a future wikipage.
 */
class ContextFreeToSensitiveFormula : public Formula<2>
{
public:
    TruthValue* simpleCompute(const TVSeq& TV, long U = DefaultU) const;
};

}} // namespace opencog::pln

#endif
