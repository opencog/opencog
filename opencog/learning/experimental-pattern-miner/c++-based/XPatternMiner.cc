/*
 * XPatternMiner.cc
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#include "XPatternMiner.h"

#include <opencog/util/algorithm.h>
#include <opencog/util/dorepeat.h>
#include <opencog/util/random.h>

#include <opencog/atoms/execution/Instantiator.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atoms/core/LambdaLink.h>
#include <opencog/atomutils/TypeUtils.h>
#include <opencog/atomutils/Unify.h>
#include <opencog/query/BindLinkAPI.h>

namespace opencog
{

XPMParameters::XPMParameters(int ms, const Handle& ipat, int maxp)
	: minsup(ms), initpat(ipat), maxpats(maxp)
{
	// Provide initial pattern if none
	if (not initpat) {
		Handle X = createNode(VARIABLE_NODE, "$X");
		initpat = createLambdaLink(X, X);
	}

	// Provide a variable declaration if none
	ScopeLinkPtr sc = ScopeLinkCast(initpat);
	OC_ASSERT(sc != nullptr, "bug or user error, pattern must be a scope");
	if (not sc->get_vardecl()) {
		Handle vardecl = sc->get_variables().get_vardecl(),
			body = sc->get_body();
		initpat = createLink(initpat->get_type(), vardecl, body);
	}
}

XPatternMiner::XPatternMiner(AtomSpace& as, const XPMParameters& prm)
	: text_as(as), param(prm) {}

HandleSet XPatternMiner::operator()()
{
	HandleSet texts;
	text_as.get_handles_by_type(std::inserter(texts, texts.end()),
	                            opencog::ATOM, true);
	return specialize(param.initpat, texts);
}

HandleSet XPatternMiner::specialize(const Handle& pattern,
                                    const HandleSet& texts)
{
	return specialize(pattern, HandleUCounter(texts));
}

HandleSet XPatternMiner::specialize(const Handle& pattern,
                                    const HandleUCounter& texts)
{
	// If the pattern is constant, then no specialization is possible
	if (pattern->get_type() != LAMBDA_LINK)
		return HandleSet();

	// Do not keep unnecessary unmatching texts
	// TODO: we can probably assume that it is already filtered, and
	// move the filtering to the specialize method above
	HandleUCounter fltexts = filter_texts(pattern, texts);

	// Make sure the pattern has enough support
	if (not enough_support(fltexts))
		return HandleSet();

	// If the pattern is a variable, then build shallow patterns and
	// complete them.
	if (get_body(pattern)->get_type() == VARIABLE_NODE)
		return specialize_varpat(pattern, fltexts);

	// Otherwise, if the pattern is structured, calculate the mappings
	// from variables to values, get the patterns of these values now
	// interpreted as texts, and use these patterns to expand the
	// initial pattern by composition.
	HandleUCounterMap var2subtexts = gen_var2subtexts(pattern, texts);

	// For each variable, create a trivial pattern, with its variable
	// as body and its type as vardecl, take the list of values, pass
	// it as texts and call specialize to generate all sub-patterns.
	HandleMultimap var2subpats;
	for (const auto& el : var2subtexts) {
		const Handle& var = el.first;
		const HandleUCounter& subtexts = el.second;
		Handle varpattern = mk_varpattern(pattern, var);
		// TODO: maybe we can use specialize_varpat, if no support
		// checking is necessary.
		var2subpats[var] = specialize(varpattern, subtexts);
	}

	// Replacing each variable by its patterns in a cartesian product
	// fashion (not forgetting updating their vardecls accordingly)
	return product_compose(pattern, fltexts, var2subpats);
}

HandleSet XPatternMiner::specialize_varpat(const Handle& varpat,
                                           const HandleUCounter& texts)
{
	OC_ASSERT(get_body(varpat)->get_type() == VARIABLE_NODE);

	HandleUCounterMap pat2texts = shallow_patterns(texts);
	HandleSet patterns;
	for (const auto& el : pat2texts) {
		const Handle& shapat = el.first;
		const HandleUCounter& shapat_texts = el.second;

		// Make sure the shallow pattern has enough support before
		// including it in the solution set and specializing it.
		if (not enough_support(shapat_texts))
			continue;

		// Add shallow pattern to solution set, and specialize it
		patterns.insert(shapat);
		set_union_modify(patterns, specialize(shapat, shapat_texts));
	}
	return patterns;
}

const Variables& XPatternMiner::get_variables(const Handle& pattern) const
{
	ScopeLinkPtr sc = ScopeLinkCast(pattern);
	if (sc)
		return ScopeLinkCast(pattern)->get_variables();
	static Variables empty_variables;
	return empty_variables;
}

const Handle& XPatternMiner::get_vardecl(const Handle& pattern) const
{
	ScopeLinkPtr sc = ScopeLinkCast(pattern);
	if (sc)
		return ScopeLinkCast(pattern)->get_vardecl();
	return Handle::UNDEFINED;
}

const Handle& XPatternMiner::get_body(const Handle& pattern) const
{
	ScopeLinkPtr sc = ScopeLinkCast(pattern);
	if (sc)
		return ScopeLinkCast(pattern)->get_body();
	return pattern;
}

Handle XPatternMiner::mk_varpattern(const Handle& pattern,
                                    const Handle& var) const
{
	Handle subvardecl = filter_vardecl(get_vardecl(pattern), var);
	return Handle(createLambdaLink(subvardecl, var));
}

bool XPatternMiner::enough_support(const HandleUCounter& texts) const
{
	return param.minsup <= texts.total_count();
}

HandleUCounter XPatternMiner::filter_texts(const Handle& pattern,
                                           const HandleUCounter& texts) const
{
	HandleUCounter filtrd;
	for (const auto& text : texts)
		if (match(pattern, text.first))
			filtrd.insert(text);
	return filtrd;
}

bool XPatternMiner::match(const Handle& pattern, const Handle& text) const
{
	// If constant pattern, matching ammounts to equality
	if (pattern->get_type() != LAMBDA_LINK)
		return content_eq(pattern, text);

	// Otherwise see if pattern matches text
	return 0 < matched_results(pattern, text)->get_arity();
}

Handle XPatternMiner::matched_results(const Handle& pattern,
                                      const Handle& text) const
{
	// No need to run if most abstract
	if (most_abstract(pattern))
		return createLink(SET_LINK, text);

	AtomSpace tmp_as;
	tmp_as.add_atom(text);
	AtomSpace tmp_pattern_as(&tmp_as);
	Handle tmp_pattern = tmp_pattern_as.add_atom(pattern),
		vardecl = get_vardecl(tmp_pattern),
		body = get_body(tmp_pattern),
		gl = tmp_as.add_link(GET_LINK, vardecl, body),
		result = satisfying_set(&tmp_as, gl);
	return result;
}

HandleUCounterMap XPatternMiner::shallow_patterns(const HandleUCounter& texts)
{
	HandleUCounterMap pat2texts;
	for (const auto& text : texts) {
		// Node or empty link, nothing to abstract
		if (text.first->is_node() or text.first->get_arity() == 0) {
			pat2texts[text.first].insert(text);
			continue;
		}

		// Non empty link, let's abstract away the arguments
		HandleSeq rnd_vars = gen_rand_variables(text.first->get_arity());
		Handle vardecl = rnd_vars.size() == 1 ? rnd_vars[0]
			: pattern_as.add_link(VARIABLE_LIST, rnd_vars),
			body = pattern_as.add_link(text.first->get_type(), rnd_vars),
			pattern = pattern_as.add_link(LAMBDA_LINK, vardecl, body);
		pat2texts[pattern].insert(text);
	}
	return pat2texts;
}

HandleUCounterMap XPatternMiner::gen_var2subtexts(const Handle& pattern,
                                                  const HandleUCounter& texts) const
{
	HandleUCounterMap var2subtexts;
	const Variables& vars = get_variables(pattern);
	for (const auto& text : texts) {
		Handle results = matched_results(pattern, text.first);
		for (const Handle& values : results->getOutgoingSet()) {
			for (const auto& v : gen_var2val(vars, values)) {
				const Handle& var = v.first;
				const Handle& val = v.second;
				var2subtexts[var][val] += 1;
			}
		}
	}
	return var2subtexts;
}

HandleMap XPatternMiner::gen_var2val(const Variables& vars,
                                     const Handle& values) const
{
	const HandleSeq& varseq = vars.varseq;

	// If there is only one variable then the list of value is the
	// value itself
	if (varseq.size() == 1)
		return {{varseq[0], values}};

	// Otherwise unpack the list of values
	const HandleSeq& valueseq = values->getOutgoingSet();
	OC_ASSERT(varseq.size() == valueseq.size());
	HandleMap result;
	for (size_t i = 0; i < varseq.size(); i++)
		result[varseq[i]] = valueseq[i];
	return result;
}

HandleSeq XPatternMiner::gen_rand_variables(size_t n) const
{
	HandleSeq variables;
	dorepeat (n)
		variables.push_back(gen_rand_variable());
	return variables;
}

Handle XPatternMiner::gen_rand_variable() const
{
	return createNode(VARIABLE_NODE, randstr("$PM-", 2));
}

bool XPatternMiner::most_abstract(const Handle& pattern) const
{
	return pattern->get_type() == LAMBDA_LINK and
		get_vardecl(pattern)->get_type() == VARIABLE_NODE and
		get_body(pattern)->get_type() == VARIABLE_NODE;
}

HandleSet XPatternMiner::product_compose(const Handle& pattern,
                                         const HandleUCounter& texts,
                                         const HandleMultimap& var2pats) const
{
	HandleSet patterns;
	for (auto map_it = var2pats.begin(); map_it != var2pats.end(); map_it++) {
		const Handle& variable = map_it->first;
		const HandleSet& subpatterns = map_it->second;

		// Reconstruct var2pats without that variable, and the ones
		// preceeding, and recursively call product_compose to obtain
		// patterns without the subpatterns of that variables.
		HandleMultimap revar2pats(std::next(map_it), var2pats.end());
		set_union_modify(patterns, product_compose(pattern, texts, revar2pats));

		// Combine the patterns of that variables with the patterns of
		// the other variables.
		for (auto subpat_it = subpatterns.begin();
		     subpat_it != subpatterns.end(); ++subpat_it) {
			// Perform a single substitution of variable by subpat
			Handle npat = compose(pattern, {{variable, *subpat_it}});
			HandleUCounter fltexts = filter_texts(npat, texts);
			if (enough_support(fltexts)) {
				patterns.insert(npat);

				// Recursively call product_compose on nap and the
				// remaining variables
				set_union_modify(patterns,
				                 product_compose(npat, fltexts, revar2pats));
			}

			// Add the remaining subpatterns sub-patterns to var2pats,
			// and recursively call product_compose over pattern
			HandleMultimap revar2pats_revar(revar2pats);
			revar2pats_revar[variable].insert(std::next(subpat_it), subpatterns.end());
			set_union_modify(patterns,
			                 product_compose(pattern, texts, revar2pats_revar));
		}
	}
	return patterns;
}

Handle XPatternMiner::compose(const Handle& pattern,
                              const HandleMap& var2pat) const
{
	// Split var2pat into 2 mappings, variable to sub-vardecl and
	// variable to sub-body
	HandleMap var2subdecl, var2subody;
	for (const auto& el : var2pat) {
		var2subdecl[el.first] = get_vardecl(el.second);
		var2subody[el.first] = get_body(el.second);
	}

	// Get variable declaration of the composition
	Handle vardecl = vardecl_compose(get_vardecl(pattern), var2subdecl);

	// Turn the map into a vector of new bodies
	const Variables variables = get_variables(pattern);
	HandleSeq subodies = variables.make_values(var2subody);

	// Perform composition of the pattern body with the sub-bodies)
	Handle body = variables.substitute_nocheck(get_body(pattern), subodies);
	body = Unify::consume_ill_quotations(vardecl, body);

	// Create the composed pattern
	if (vardecl)
		return createLink(HandleSeq{vardecl, body}, pattern->get_type());

	// No variable, the pattern is the body itself
	return body;
}

Handle XPatternMiner::vardecl_compose(const Handle& vardecl,
                                      const HandleMap& var2subdecl)
{
	OC_ASSERT((bool)vardecl, "Not implemented");

	Type t = vardecl->get_type();

	// Base cases

	if (t == VARIABLE_NODE) {
		auto it = var2subdecl.find(vardecl);
		// Compose if the variable maps to another variable
		// declaration
		if (it != var2subdecl.end())
			return it->second;
		return vardecl;
	}

	// Recursive cases

	if (t == VARIABLE_LIST) {
		HandleSeq oset;
		for (const Handle& h : vardecl->getOutgoingSet()) {
			Handle nh = vardecl_compose(h, var2subdecl);
			if (nh) {
				if (nh->get_type() == VARIABLE_LIST)
					for (const Handle nhc : nh->getOutgoingSet())
						oset.push_back(nhc);
				else
					oset.push_back(nh);
			}
		}

		if (oset.empty())
			return Handle::UNDEFINED;
		if (oset.size() == 1)
			return oset[0];
		return createLink(oset, t);
	}
	else if (t == TYPED_VARIABLE_LINK) {
		return vardecl_compose(vardecl->getOutgoingAtom(0), var2subdecl);
	}
	else {
		OC_ASSERT(false, "Not implemented");
		return Handle::UNDEFINED;
	}
}

} // namespace opencog
