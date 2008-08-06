#ifndef RULES_H
#define RULES_H

#include "Rule.h"
#include "GenericRule.h"
#include <SimpleTruthValue.h>
#include "../iAtomTableWrapper.h"
//#include "../CoreWrapper.h"
#include "../Ptlatom.h"
#include "../PLNFormulas/PLNFormulas.h"
#include "../NMPrinter.h"

#include <Atom.h>
#include <ClassServer.h>
#include <CogServer.h>
#include <Link.h>
#include <Node.h>
#include <TLB.h>

#include "RuleFunctions.h"

/*	PLNRules - Must define, for each rule:
	Handle compute(Handle* h,const int n, Handle CX = NULL) const
	MPs o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
	atom i2oType(Handle* h, const int n) const

	And inputFilter must be initialized in the constructor!
*/

#include "ANDRules/ANDBreakdownRule.h"
#include "ANDRules/ANDPartitionRule.h"
#include "ANDRules/ANDSubstRule.h"
#include "ANDRules/ANDRuleArityFree.h"
#include "ANDRules/ANDRule.h"
#include "ANDRules/ANDRuleSimple.h"
#include "ANDRules/AND2ORRule.h"								/* Not Used */

#include "ORRules/ORRule.h"
#include "ORRules/ORPartitionRule.h"
#include "ORRules/OR2ANDRule.h"									/* Not Used */

#include "NOTRules/NotEvaluatorRule.h"
#include "NOTRules/NotEliminationRule.h"						/* Not Used */

#include "ConversionRules/Equi2ImpRule.h"						/* Not Used */
#include "ConversionRules/Eval2MemRule.h"						/* Not Used */
#include "ConversionRules/Exist2ForAllRule.h"					/* Not Used */
#include "ConversionRules/Ext2ExtRule.h"						/* Not Used */
#include "ConversionRules/Inclusion2EvalRule.h"					/* Not Used */
#include "ConversionRules/Inh2EvalRule.h"						/* Not Used */
#include "ConversionRules/Inh2SimRule.h"						/* Not Used */
#include "ConversionRules/Link2LinkRule.h"
#include "ConversionRules/Sim2InhRule.h"						/* Not Used */

#include "ImplicationRules/ImplicationBreakdownRule.h"
#include "ImplicationRules/ImplicationRedundantExpansionRule.h"
#include "ImplicationRules/ImplicationTailExpansionRule.h"
#include "ImplicationRules/StrictImplicationBreakdownRule.h"

#include "AuxiliarRules/CrispTheoremRule.h"
#include "AuxiliarRules/LookupRule.h"
#include "AuxiliarRules/PrintRule.h"							/* Not Used */
#include "AuxiliarRules/ScholemFunctionProductionRule.h"
#include "AuxiliarRules/SubsetEvalRule.h"						/* Not Used */
#include "AuxiliarRules/VarInstantiationRule.h"					/* Not Used */

#include "SimSubstRules/SimSubstRule1.h"						/* Not Used */
#include "SimSubstRules/SimSubstRule2.h"						/* Not Used */

#include "UnificationRules/BaseCrispUnificationRule.h"
#include "UnificationRules/CrispUnificationRule.h"
#include "UnificationRules/CustomCrispUnificationRule.h"
#include "UnificationRules/CustomCrispUnificationRuleComposer.h"
#include "UnificationRules/StrictCrispUnificationRule.h"

#include "InferenceRules/DeductionRule.h"
#include "InferenceRules/HypothesisRule.h"
#include "InferenceRules/InversionRule.h"
#include "InferenceRules/QuantifierRule.h"
#include "InferenceRules/RevisionRule.h"						/* Not Used */

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
