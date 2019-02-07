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

#include "MinerUTestUtils.h"

#include <boost/range/algorithm/sort.hpp>

#include <opencog/guile/SchemeSmob.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/rule-engine/forwardchainer/ForwardChainer.h>
#include <opencog/rule-engine/backwardchainer/BackwardChainer.h>

using namespace opencog;

#define an as.add_node
#define al as.add_link

Handle MinerUTestUtils::add_texts_cpt(AtomSpace& as)
{
	return an(CONCEPT_NODE, "texts");
}

Handle MinerUTestUtils::add_minsup_prd(AtomSpace& as)
{
	return an(PREDICATE_NODE, "minsup");
}

Handle MinerUTestUtils::add_isurp_prd(AtomSpace& as)
{
	return an(PREDICATE_NODE, "I-Surprisingness");
}

Handle MinerUTestUtils::add_top(AtomSpace& as)
{
	Handle X = an(VARIABLE_NODE, "$X");
	return al(LAMBDA_LINK, X, X);
}

Handle MinerUTestUtils::add_minsup_eval(AtomSpace& as,
                                        const Handle& pattern,
                                        int minsup,
                                        TruthValuePtr tv)
{
	Handle minsup_eval_h = al(EVALUATION_LINK,
	                          add_minsup_prd(as),
	                          al(LIST_LINK,
	                             pattern,
	                             add_texts_cpt(as),
	                             an(NUMBER_NODE, std::to_string(minsup))));
	// Warning: if minsup_eval_h existed, this may erase its TV
	minsup_eval_h->setTruthValue(tv);
	return minsup_eval_h;
}

Handle MinerUTestUtils::add_minsup_evals(AtomSpace& as,
                                         const HandleSeq& patterns,
                                         int minsup,
                                         TruthValuePtr tv)
{
	HandleSeq minsup_evals;
	for (const Handle& pat : patterns)
		minsup_evals.push_back(add_minsup_eval(as, pat, minsup, tv));
	return al(SET_LINK, minsup_evals);
}

Handle MinerUTestUtils::add_isurp_eval(AtomSpace& as,
                                       const Handle& pattern)
{
	Handle isurp_eval_h = al(EVALUATION_LINK,
	                         add_isurp_prd(as),
	                         al(LIST_LINK,
	                            pattern,
	                            add_texts_cpt(as)));
	return isurp_eval_h;
}

Handle MinerUTestUtils::get_pattern(const Handle& minsup_eval)
{
	return minsup_eval->getOutgoingAtom(1)->getOutgoingAtom(0);
}

HandleSeq MinerUTestUtils::get_patterns(const HandleSeq& minsup_evals)
{
	HandleSeq patterns;
	for (const Handle& minsup_eval : minsup_evals)
		patterns.push_back(get_pattern(minsup_eval));
	return patterns;
}

Handle MinerUTestUtils::add_abs_true_eval(AtomSpace& as, const Handle& h)
{
	return al(EVALUATION_LINK,
	          an(GROUNDED_PREDICATE_NODE, "scm: absolutely-true"),
	          h);
}

Handle MinerUTestUtils::add_nconjunct(AtomSpace& as, unsigned n)
{
	return al(LAMBDA_LINK, al(AND_LINK, add_variables(as, "$X-", n)));
}

Handle MinerUTestUtils::add_variable(AtomSpace& as,
                                     const std::string& prefix,
                                     int i)
{
	return an(VARIABLE_NODE, prefix + std::to_string(i));
}

HandleSeq MinerUTestUtils::add_variables(AtomSpace& as,
                                         const std::string& prefix,
                                         int n)
{
	HandleSeq vars;
	for (int i = 0; i < n; i++)
		vars.push_back(add_variable(as, prefix, i));
	return vars;
}

Handle MinerUTestUtils::ure_pm(AtomSpace& as,
                               SchemeEval& scm,
                               const Handle& pm_rb,
                               const AtomSpace& texts_as,
                               int minsup,
                               int maximum_iterations,
                               Handle initpat,
                               TruthValuePtr incremental_expansion,
                               int max_conjuncts,
                               double complexity_penalty)
{
	HandleSet texts;
	texts_as.get_handles_by_type(std::inserter(texts, texts.end()),
	                             opencog::ATOM, true);
	return ure_pm(as, scm, pm_rb, texts, minsup, maximum_iterations, initpat,
	              incremental_expansion, max_conjuncts, complexity_penalty);
}

Handle MinerUTestUtils::ure_pm(AtomSpace& as,
                               SchemeEval& scm,
                               const Handle& pm_rb,
                               const HandleSet& texts,
                               int minsup,
                               int maximum_iterations,
                               Handle initpat,
                               TruthValuePtr incremental_expansion,
                               int max_conjuncts,
                               double complexity_penalty)
{
	// Make (Member text (Concept "texts)) links
	for (const Handle& text : texts)
		al(MEMBER_LINK, text, add_texts_cpt(as));

	// If init is not defined then use top
	if (not initpat)
		initpat = add_top(as);

	// Add the axiom that initpat has enough support, and use it as
	// source for the forward chainer
	bool es = MinerUtils::enough_support(initpat, texts, minsup);

	// If it doesn't have enough support return the empty solution
	if (not es)
		return al(SET_LINK);

	// Add incremental conjunction expansion if necessary
	bool inc_exp_enabled = incremental_expansion != TruthValue::FALSE_TV();
	configure_optional_rules(scm, incremental_expansion, max_conjuncts);

	// Otherwise prepare the source
	TruthValuePtr tv = TruthValue::TRUE_TV();
	Handle source = add_minsup_eval(as, initpat, minsup, tv);

	// Run the forward chainer from the initial pattern
	ForwardChainer fc(as, pm_rb, source);
	fc.get_config().set_maximum_iterations(maximum_iterations);
	fc.get_config().set_retry_exhausted_sources(inc_exp_enabled);
	fc.get_config().set_complexity_penalty(complexity_penalty);
	fc.do_chain();

	// Run the pattern matcher query to gather the knowledge of
	// interest, i.e. patterns reaching the minimum support, and
	// return the results.
	Handle patvar = an(VARIABLE_NODE, "$patvar"),
		target = add_minsup_eval(as, patvar, minsup),
		vardecl = al(TYPED_VARIABLE_LINK, patvar, an(TYPE_NODE, "LambdaLink")),
		abs_true = add_abs_true_eval(as, target),
		bl = al(BIND_LINK, vardecl, al(AND_LINK, target, abs_true), target),
		results = bindlink(&as, bl);

	return results;
}

HandleTree MinerUTestUtils::cpp_pm(const AtomSpace& texts_as,
                                   int minsup,
                                   int conjuncts,
                                   const Handle& initpat,
                                   int maxdepth,
                                   double info)
{
	MinerParameters param(minsup, conjuncts, initpat, maxdepth, info);
	Miner pm(param);
	return pm(texts_as);
}

HandleTree MinerUTestUtils::cpp_pm(const HandleSet& texts,
                                   int minsup,
                                   int conjuncts,
                                   const Handle& initpat,
                                   int maxdepth,
                                   double info)
{
	MinerParameters param(minsup, conjuncts, initpat, maxdepth, info);
	Miner pm(param);
	return pm(texts);
}

Handle MinerUTestUtils::add_is_cpt_pattern(AtomSpace& as, const Handle& cpt)
{
	Handle X = an(VARIABLE_NODE, "$X"),
		is_cpt = al(INHERITANCE_LINK, X, cpt),
		pattern = al(LAMBDA_LINK,
		                 X,
		                 is_cpt);
	return pattern;
}

Handle MinerUTestUtils::add_ugly_pattern(AtomSpace& as)
{
	return add_is_cpt_pattern(as, an(CONCEPT_NODE, "ugly"));
}

Handle MinerUTestUtils::add_man_pattern(AtomSpace& as)
{
	return add_is_cpt_pattern(as, an(CONCEPT_NODE, "man"));
}

Handle MinerUTestUtils::add_soda_drinker_pattern(AtomSpace& as)
{
	return add_is_cpt_pattern(as, an(CONCEPT_NODE, "soda_drinker"));
}

Handle MinerUTestUtils::add_ugly_man_pattern(AtomSpace& as)
{
	Handle X = an(VARIABLE_NODE, "$X"),
		man = an(CONCEPT_NODE, "man"),
		ugly = an(CONCEPT_NODE, "ugly"),
		is_man = al(INHERITANCE_LINK, X, man),
		is_ugly = al(INHERITANCE_LINK, X, ugly),
		is_ugly_man = al(AND_LINK, is_ugly, is_man),
		pattern = al(LAMBDA_LINK, X, is_ugly_man);
	return pattern;
}

Handle MinerUTestUtils::add_ugly_man_soda_drinker_pattern(AtomSpace& as)
{
	Handle X = an(VARIABLE_NODE, "$X"),
		man = an(CONCEPT_NODE, "man"),
		soda_drinker = an(CONCEPT_NODE, "soda drinker"),
		ugly = an(CONCEPT_NODE, "ugly"),
		is_man = al(INHERITANCE_LINK, X, man),
		is_soda_drinker = al(INHERITANCE_LINK, X, soda_drinker),
		is_ugly = al(INHERITANCE_LINK, X, ugly),
		is_ugly_man_soda = al(AND_LINK, is_ugly, is_man, is_soda_drinker),
		pattern = al(LAMBDA_LINK, X, is_ugly_man_soda);
	return pattern;
}

void MinerUTestUtils::configure_mandatory_rules(SchemeEval& scm)
{
	std::string rs = scm.eval("(configure-mandatory-rules (Concept \"pm-rbs\"))");
	logger().debug() << "MinerUTest::configure_mandatory_rules() rs = " << rs;
}

void MinerUTestUtils::configure_optional_rules(SchemeEval& scm,
                                               TruthValuePtr incremental_expansion,
                                               int max_conjuncts)
{
	std::string call = "(configure-optional-rules (Concept \"pm-rbs\")";
	call += " #:incremental-expansion ";
	call += SchemeSmob::tv_to_string(incremental_expansion);
	call += " #:max-conjuncts ";
	call += std::to_string(max_conjuncts);
	call += ")";
	std::string rs = scm.eval(call);
	logger().debug() << "MinerUTest::configure_optional_rules() rs = " << rs;
}

void MinerUTestUtils::configure_ISurprisingness(SchemeEval& scm,
                                                const Handle& isurp_rb,
                                                unsigned max_conjuncts)
{
	std::string call = "(configure-I-Surprisingness (Concept \""
		+ isurp_rb->get_name() + "\") ";
	call += std::to_string(max_conjuncts);
	call += ")";
	std::string rs = scm.eval(call);
	logger().debug() << "MinerUTest::configure_ISurprisingness() rs = " << rs;
}

HandleSeq MinerUTestUtils::ure_isurp(AtomSpace& as,
                                     SchemeEval& scm,
                                     const Handle& isurp_rb,
                                     unsigned max_conjuncts)
{
	configure_ISurprisingness(scm, isurp_rb, max_conjuncts);
	Handle X = an(VARIABLE_NODE, "$X"),
		target = add_isurp_eval(as, X),
		vardecl = al(TYPED_VARIABLE_LINK, X, an(TYPE_NODE, "LambdaLink"));
	BackwardChainer bc(as, isurp_rb, target, vardecl);
	bc.do_chain();
	Handle isurp_results = bc.get_results();
	HandleSeq isurp_results_seq = isurp_results->getOutgoingSet();
	// Sort according to surprisingness
	boost::sort(isurp_results_seq, [](const Handle& lh, const Handle& rh) {
			return lh->getTruthValue()->get_mean() > rh->getTruthValue()->get_mean();
		});
	return isurp_results_seq;
}
