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
	return xspecialize(param.initpat, texts);
}

void XPatternMiner::specialize(const Handle& pattern, int distance)
{
	// No specialization at such distance
	if (distance == 0)
		return;

	// According to the a priori property, if the pattern frequency is
	// below the minimum, its specializations will be as well.
	if (freq(pattern) < param.minsup)
		return;

	// Recursive case
	HandleSet npats = next_specialize(pattern);
	insert(npats);
	if ((int)patterns.size() != param.maxpats)
		for (const Handle& pat : npats)
			specialize(pat, distance - 1);
}

HandleSet XPatternMiner::xspecialize(const Handle& pattern,
                                     const HandleSet& texts)
{
	// TODO wait a minute!
	// // Make sure the pattern to specialize has enough support
	// if (freq(pattern) < param.minsup)
	// 	return HandleSet();

	// If the pattern is constant, then no specialization is possible
	if (pattern->get_type() != LAMBDA_LINK)
		return HandleSet();

	// If the pattern is a variable, then build shallow patterns and
	// complete them.
	if (get_body(pattern)->get_type() == VARIABLE_NODE) {
		HandleMultimap pat2texts = shallow_patterns(texts);
		HandleSet patterns;
		for (const auto& el : pat2texts)
			set_union_modify(patterns, xspecialize(el.first, el.second));
		return patterns;
	}

	// Otherwise, if the pattern is structured, calculate the mappings
	// from variables to values, get the patterns of these values now
	// interpreted as texts, and use these patterns to expand the
	// initial pattern.
	HandleMapSet var2vals = gen_var2vals(pattern, texts);

	// For each variable, create a trivial pattern, with its variable
	// as body and its type as vardecl, take the list of values, pass
	// it as texts and call xspecialize.
	HandleMultimap var2subpats;
	for (const Handle& var : get_variables(pattern).varset) {
		HandleSet subtexts = select_values(var2vals, var);
		Handle subvardecl = filter_vardecl(get_vardecl(pattern), var);
		Handle subpattern = Handle(createLambdaLink(subvardecl, var));
		HandleSet subpatterns = xspecialize(subpattern, subtexts);
		var2subpats[var] = subpatterns;
	}

	// Replacing each variable with its patterns in a cartesian
	// product fashion (not forgetting updating their vardecl
	// accordingly)
	return product_compose(pattern, var2subpats);
}

unsigned XPatternMiner::freq(const Handle& pattern) const
{
	AtomSpace tmp_as(&text_as);

	Handle
		vardecl = get_vardecl(pattern),
		body = get_body(pattern),
		// the rewrite is the body wrapped in a DontExec. That is to
		// count correctly. Indeed if we were to use a GetLink, extra
		// permutations would be considered when the pattern contains
		// unordered links.
		rewrite = tmp_as.add_link(DONT_EXEC_LINK, body),
		bl = tmp_as.add_link(BIND_LINK, vardecl, body, rewrite),
		result = bindlink(&tmp_as, bl);

	return result->get_arity();
}

const Variables& XPatternMiner::get_variables(const Handle& pattern) const
{
	return ScopeLinkCast(pattern)->get_variables();
}

const Handle& XPatternMiner::get_vardecl(const Handle& pattern) const
{
	return ScopeLinkCast(pattern)->get_vardecl();
}

const Handle& XPatternMiner::get_body(const Handle& pattern) const
{
	return ScopeLinkCast(pattern)->get_body();
}

void XPatternMiner::insert(const HandleSet& npats)
{
	for (auto it = npats.begin(); it != npats.end()
		     // Make sure we don't add more patterns than maximum
		     and (int)patterns.size() != param.maxpats; ++it) {
		patterns.insert(*it);
	}
}

HandleSet XPatternMiner::next_specialize(const Handle& pattern) const
{
	// TODO, perhaps...
	return HandleSet();
}

HandleMultimap XPatternMiner::shallow_patterns(const HandleSet& texts)
{
	HandleMultimap pat2texts;
	for (const Handle& text : texts) {
		// Node or empty link, nothing to abstract
		if (text->is_node() or text->get_arity() == 0) {
			pat2texts[text].insert(text);
			continue;
		}

		// Non empty link, let's abstract away the arguments
		HandleSeq rnd_vars = gen_rand_variables(text->get_arity());
		Handle vardecl = rnd_vars.size() == 1 ? rnd_vars[0]
			: pattern_as.add_link(VARIABLE_LIST, rnd_vars),
			body = pattern_as.add_link(text->get_type(), rnd_vars),
			pattern = pattern_as.add_link(LAMBDA_LINK, vardecl, body);
		pat2texts[pattern].insert(text);
	}
	return pat2texts;
}

HandleMapSet XPatternMiner::gen_var2vals(const Handle& pattern,
                                         const HandleSet& texts) const
{
	AtomSpace tmp_as;
	Handle inputs = tmp_as.add_link(SET_LINK, HandleSeq(texts.begin(), texts.end())),
		mpl = tmp_as.add_link(MAP_LINK, pattern, inputs);

	Instantiator inst(&tmp_as);
	Handle rh(inst.execute(mpl));

	HandleMapSet results;
	for (const Handle& values : rh->getOutgoingSet())
		results.insert(gen_var2val(get_variables(pattern), values));
	return results;
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

HandleSet XPatternMiner::select_values(const HandleMapSet& var2vals,
                                       const Handle& var) const
{
	HandleSet values;
	for (const HandleMap& var2val : var2vals)
		values.insert(var2val.at(var));
	return values;
}

HandleSet XPatternMiner::product_compose(const Handle& pattern,
                                         const HandleMultimap& var2pats) const
{
	HandleSet patterns;
	for (auto map_it = var2pats.begin(); map_it != var2pats.end(); map_it++) {
		const Handle& variable = map_it->first;
		const HandleSet& subpatterns = map_it->second;

		// Reconstruct var2pats without that variable, and the ones
		// preceeding, and recursively call product_compose to obtain
		// patterns without the subpatterns of that variables.
		HandleMultimap re_var2pats(std::next(map_it), var2pats.end());
		set_union_modify(patterns, product_compose(pattern, re_var2pats));

		// Combine the patterns of that variables with the patterns of
		// the other variables.
		for (auto subpat_it = subpatterns.begin();
		     subpat_it != subpatterns.end(); ++subpat_it) {
			// Perform a single substitution of variable by subpat
			Handle npat = compose(pattern, {{variable, *subpat_it}});

			// Add the remaining subpatterns sub-patterns to var2pats,
			// and recursively call product_substitute over it to
			// generate substitutions combined with subpat
			re_var2pats[variable].insert(std::next(subpat_it), subpatterns.end());
			set_union_modify(patterns, product_compose(npat, re_var2pats));
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

	// Create the substituted BindLink
	return createLink(HandleSeq{vardecl, body}, pattern->get_type());
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
		else
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
