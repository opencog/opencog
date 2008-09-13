#ifndef RULES_H
#define RULES_H

#include "../iAtomTableWrapper.h"
#include "../AtomTableWrapper.h"
//#include "../CoreWrapper.h"
#include "../PLNatom.h"
#include "../formulas/Formulas.h"
#include "../utils/NMPrinter.h"

#include <SimpleTruthValue.h>
#include <Atom.h>
#include <ClassServer.h>
#include <CogServer.h>
#include <Link.h>
#include <Node.h>
#include <TLB.h>

#include "Rule.h"
#include "GenericRule.h"
#include "RuleFunctions.h"

/*	PLNRules - Must define, for each rule:
	Handle compute(Handle* h,const int n, Handle CX = NULL) const
	MPs o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
	atom i2oType(Handle* h, const int n) const

	And inputFilter must be initialized in the constructor!
*/

namespace haxx
{
	extern reasoning::iAtomTableWrapper* defaultAtomTableWrapper;
}

#include "and/ANDBreakdownRule.h"
#include "and/ANDPartitionRule.h"
#include "and/ANDSubstRule.h"
#include "and/ANDRuleArityFree.h"
#include "and/ANDRule.h"
#include "and/ANDRuleSimple.h"
#include "and/AND2ORRule.h"								/* Not Used */

#include "or/ORRule.h"
#include "or/ORPartitionRule.h"
#include "or/OR2ANDRule.h"									/* Not Used */

#include "not/NotEvaluatorRule.h"
#include "not/NotEliminationRule.h"						/* Not Used */

#include "convert/Equi2ImpRule.h"						/* Not Used */
#include "convert/Eval2MemRule.h"						/* Not Used */
#include "convert/Exist2ForAllRule.h"					/* Not Used */
#include "convert/Ext2ExtRule.h"						/* Not Used */
#include "convert/Inclusion2EvalRule.h"					/* Not Used */
#include "convert/Inh2EvalRule.h"						/* Not Used */
#include "convert/Inh2SimRule.h"						/* Not Used */
#include "convert/Link2LinkRule.h"
#include "convert/Sim2InhRule.h"						/* Not Used */

#include "implication/ImplicationBreakdownRule.h"
#include "implication/ImplicationRedundantExpansionRule.h"
#include "implication/ImplicationTailExpansionRule.h"
#include "implication/StrictImplicationBreakdownRule.h"

#include "auxiliary/CrispTheoremRule.h"
#include "auxiliary/LookupRule.h"
#include "auxiliary/PrintRule.h"							/* Not Used */
#include "auxiliary/ScholemFunctionProductionRule.h"
#include "auxiliary/SubsetEvalRule.h"						/* Not Used */
#include "auxiliary/VarInstantiationRule.h"					/* Not Used */

#include "simsubst/SimSubstRule1.h"						/* Not Used */
#include "simsubst/SimSubstRule2.h"						/* Not Used */

#include "unify/BaseCrispUnificationRule.h"
#include "unify/CrispUnificationRule.h"
#include "unify/CustomCrispUnificationRule.h"
#include "unify/CustomCrispUnificationRuleComposer.h"
#include "unify/StrictCrispUnificationRule.h"

#include "inference/DeductionRule.h"
#include "inference/HypothesisRule.h"
#include "inference/InversionRule.h"
#include "inference/QuantifierRule.h"
#include "inference/RevisionRule.h"						/* Not Used */

#define USE_ALL_AVAILABLE_INFORMATION_FOR_AND_RULE_COMPUTATION 1
#define MAX_ARITY_FOR_PERMUTATION 5
#define Abs(a) ( ((a)>0) ? (a) : (-a) )

//const bool RuleResultFreshness = true;

/*
	The basic problem of an individual is distinguishing between _who he "really" is_
	and the "identity" (mental image and related ingrained patterns of thought) that
	society had imposed upon him. The most important fact about the distinction is,
	however, that it is equally fictionary as her individual identity and collective identity.
	In pragmatics, the relevant point is the utility of the distinction, which in this case is 		
	significant, given the goal structure of any individualist.
*/
#endif // RULES_H
