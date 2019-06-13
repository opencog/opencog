/*
 * opencog/tests/learning/miner/MinerUTestUtils.h
 *
 * Copyright (C) 2018 SingularityNET Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeEval.h>

#include <opencog/learning/miner/HandleTree.h>
#include <opencog/learning/miner/Miner.h>

namespace opencog {

class MinerUTestUtils
{
public:
	/**
	 * Add
	 *
	 * (Concept "texts")
	 *
	 * to as.
	 */
	static Handle add_texts_cpt(AtomSpace& as);

	/**
	 * Add
	 *
	 * (Predicate "minsup")
	 *
	 * to as.
	 */
	static Handle add_minsup_prd(AtomSpace& as);

	/**
	 * Add
	 *
	 * (Predicate <mode>)
	 *
	 * to as.
	 */
	static Handle add_isurp_prd(AtomSpace& as, const std::string& mode);

	/**
	 * Add
	 *
	 * (Lambda (Variable "$X") (Variable "$X"))
	 *
	 * to as.
	 */
	static Handle add_top(AtomSpace& as);

	/**
	 * Add
	 *
	 * (Evaluation <tv>
	 *   (Predicate "minsup")
	 *   (List
	 *     pattern
	 *     (Concept "texts")
	 *     minsup))
	 *
	 * to as.
	 */
	static Handle add_minsup_eval(AtomSpace& as,
	                              const Handle& pattern,
	                              int minsup,
	                              TruthValuePtr tv=TruthValue::DEFAULT_TV());

	/**
	 * Repeat add_minsup_eval over a sequence of patterns.
	 */
	static Handle add_minsup_evals(AtomSpace& as,
	                               const HandleSeq& patterns,
	                               int minsup,
	                               TruthValuePtr tv=TruthValue::DEFAULT_TV());

	/**
	 * Insert
	 *
	 * (Evaluation
	 *   (Predicate <mode>)
	 *   (List pattern (Concept "texts")))
	 *
	 * to as.
	 */
	static Handle add_isurp_eval(AtomSpace& as,
	                             const std::string& mode,
	                             const Handle& pattern);

	/**
	 * Given
	 *
	 * (Evaluation
	 *   (Predicate "minsup")
	 *   (List pattern (Concept "texts") minsup))
	 *
	 * return pattern.
	 *
	 * Note: also works for isurp constructs
	 */
	static Handle get_pattern(const Handle& minsup_eval);
	static HandleSeq get_patterns(const HandleSeq& minsup_evals);

	/**
	 * Add
	 *
	 * (Evaluation
	 *   (GroundedPredicate "scm: absolutely-true")
	 *   h)
	 *
	 * to as.
	 */
	static Handle add_abs_true_eval(AtomSpace& as, const Handle& h);

	/**
	 * Add
	 *
	 * (Lambda
	 *   (VariableList X1 ... Xn)
	 *   (And X1 ... Xn))
	 *
	 * to as.
	 */
	static Handle add_nconjunct(AtomSpace& as, unsigned n);

	/**
	 * Add
	 *
	 * (VariableNode <prefix-i>)
	 *
	 * to as.
	 */
	static Handle add_variable(AtomSpace& as,
	                           const std::string& prefix,
	                           int i);

	/**
	 * Add
	 *
	 * (VariableNode <prefix-0>)
	 * ...
	 * (VariableNode <prefix-n-1>)
	 *
	 * to as.
	 */
	static HandleSeq add_variables(AtomSpace& as,
	                               const std::string& prefix,
	                               int n);

	/**
	 * Add a query to as, run the URE forward to generate patterns,
	 * then backward to gather the results, and output them.
	 */
	static Handle ure_pm(AtomSpace& as,
	                     SchemeEval& scm,
	                     const Handle& pm_rb,
	                     const AtomSpace& texts_as,
	                     int minsup,
	                     int max_iter=-1,
	                     Handle initpat=Handle::UNDEFINED,
	                     bool incremental_expansion=false,
	                     unsigned max_conjuncts=UINT_MAX,
	                     double complexity_penalty=0.0);
	static Handle ure_pm(AtomSpace& as,
	                     SchemeEval& scm,
	                     const Handle& pm_rb,
	                     const HandleSeq& texts, int minsup,
	                     int max_iter=-1,
	                     Handle initpat=Handle::UNDEFINED,
	                     bool incremental_expansion=false,
	                     unsigned max_conjuncts=UINT_MAX,
	                     double complexity_penalty=0.0);

	/**
	 * Configure the C++ Miner and run it.
	 */
	static HandleTree cpp_pm(const AtomSpace& texts_as,
	                         int minsup=1,
	                         int conjuncts=1,
	                         const Handle& initpat=Handle::UNDEFINED,
	                         int maxdepth=-1,
	                         double info=1.0);
	static HandleTree cpp_pm(const HandleSeq& texts,
	                         int minsup=1,
	                         int conjuncts=1,
	                         const Handle& initpat=Handle::UNDEFINED,
	                         int maxdepth=-1,
	                         double info=1.0);

	/**
	 * Add
	 *
	 * Lambda
	 *   X
	 *   Inheritance
	 *     X
	 *     cpt
	 *
	 * to as.
	 */
	static Handle add_is_cpt_pattern(AtomSpace& as, const Handle& cpt);

	/**
	 * Add
	 *
	 * Lambda
	 *   X
	 *   Inheritance
	 *     X
	 *     {Concept "ugly", Concept "man", Concept "soda_drinker"}
	 *
	 * to as.
	 */
	static Handle add_ugly_pattern(AtomSpace& as);
	static Handle add_man_pattern(AtomSpace& as);
	static Handle add_soda_drinker_pattern(AtomSpace& as);

	/**
	 * Add
	 *
	 * Lambda
	 *   X
	 *   And
	 *     Inheritance
	 *       X
	 *       Concept "ugly"
	 *     Inheritance
	 *       X
	 *       Concept "man"
	 *
	 * to as.
	 */
	static Handle add_ugly_man_pattern(AtomSpace& as);

	/**
	 * Add the following pattern
	 *
	 * Lambda
	 *   X
	 *   And
	 *     Inheritance
	 *       X
	 *       Concept "man"
	 *     Inheritance
	 *       X
	 *       Concept "soda drinker"
	 *     Inheritance
	 *       X
	 *       Concept "ugly"
	 *
	 * to as.
	 */
	static Handle add_ugly_man_soda_drinker_pattern(AtomSpace& as);

	static void configure_mandatory_rules(SchemeEval& scm);
	static void configure_optional_rules(SchemeEval& scm,
	                                     bool incremental_expansion,
	                                     unsigned max_conjuncts=UINT_MAX,
	                                     unsigned max_variables=UINT_MAX);
	static void configure_ISurprisingness(SchemeEval& scm,
	                                      const Handle& isurp_rb,
	                                      const std::string& mode,
	                                      unsigned max_conjuncts);

	/**
	 * Run I-Surprisingness reasoning on the current atomspace and
	 * return the result sorted by I-Surprisingness.
	 */
	static HandleSeq ure_isurp(AtomSpace& as,
	                           SchemeEval& scm,
	                           const Handle& isurp_rb,
	                           const std::string& mode,
	                           unsigned max_conjuncts);
};

} // ~namespace opencog
