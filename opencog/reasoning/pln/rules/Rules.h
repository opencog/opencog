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

#ifndef RULES_H
#define RULES_H

#include "../iAtomSpaceWrapper.h"
#include "../AtomSpaceWrapper.h"
//#include "../CoreWrapper.h"
#include "../PLNatom.h"
#include "../formulas/Formulas.h"
#include "../utils/NMPrinter.h"

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>

#include "Rule.h"
#include "GenericRule.h"
#include "RuleFunctions.h"

/*	PLNRules - Must define, for each rule:
	Handle compute(Handle* h,const int n, Handle CX = NULL) const
	MPs o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
	atom i2oType(Handle* h, const int n) const

	And inputFilter must be initialized in the constructor!
*/

#include "and/AndBreakdownRule.h"
#include "and/AndPartitionRule.h"
#include "and/AndSubstRule.h"
#include "and/AndRuleArityFree.h"
#include "and/AndRule.h"
#include "and/AndRuleSimple.h"
//#include "and/And2OrRule.h"

#include "or/OrRule.h"
#include "or/OrPartitionRule.h"
//#include "or/Or2AndRule.h"

#include "not/NotRule.h"
//#include "not/NotEliminationRule.h"

#include "convert/Link2LinkRule.h"
#include "convert/Equi2ImpRule.h"
//#include "convert/Eval2MemRule.h"
//#include "convert/Exist2ForAllRule.h"
//#include "convert/Ext2ExtRule.h"
//#include "convert/Inclusion2EvalRule.h"
//#include "convert/Inh2EvalRule.h"
//#include "convert/Inh2SimRule.h"
//#include "convert/Sim2InhRule.h"

#include "implication/ImplicationBreakdownRule.h"
#include "implication/ImplicationRedundantExpansionRule.h"
#include "implication/ImplicationTailExpansionRule.h"
#include "implication/StrictImplicationBreakdownRule.h"

#include "auxiliary/CrispTheoremRule.h"
#include "auxiliary/LookupRule.h"
//#include "auxiliary/PrintRule.h"
#include "auxiliary/ScholemFunctionProductionRule.h"
#include "auxiliary/SubsetEvalRule.h"
#include "auxiliary/IntensionalInheritanceRule.h"

//#include "auxiliary/VarInstantiationRule.h"

#include "substitution/InheritanceSubstRule.h"

#include "instantiation/ForAllInstantiationRule.h"
#include "instantiation/AverageInstantiationRule.h"

#include "inference/DeductionRule.h"
#include "inference/HypothesisRule.h"
#include "inference/InversionRule.h"
#include "inference/QuantifierRule.h"
// #include "inference/RevisionRule.h"

#include "context/ContextualizerRule.h"
#include "context/DecontextualizerRule.h"
#include "context/ContextFreeToSensitiveRule.h"

#define USE_ALL_AVAILABLE_INFORMATION_FOR_AND_RULE_COMPUTATION 1
#define MAX_ARITY_FOR_PERMUTATION 5

/** 
 * The basic problem of an individual is distinguishing between <i>who he
 * "really" is</i> and the "identity" (mental image and related ingrained
 * patterns of thought) that society had imposed upon him. The most important
 * fact about the distinction is, however, that it is equally fictionary as her
 * individual identity and collective identity. In pragmatics, the relevant
 * point is the utility of the distinction, which in this case is
 * significant, given the goal structure of any individualist.
 */

#endif // RULES_H
