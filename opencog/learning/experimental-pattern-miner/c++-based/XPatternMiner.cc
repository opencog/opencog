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
                                     const HandleSet& texts) const
{
	// TODO wait a minute!
	// // Make sure the pattern to specialize has enough support
	// if (freq(pattern) < param.minsup)
	// 	return HandleSet();

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
	HandleMultimap var2patterns;
	for (const Handle& var : get_variables(pattern).varset) {
		HandleSet var_texts = select_values(var2vals, var);
		Handle var_vardecl = filter_vardecl(get_vardecl(pattern), var);
		Handle var_pattern = Handle(createLambdaLink(var_vardecl, var));
		HandleSet var_patterns = xspecialize(var_pattern, var_texts);
		var2patterns[var] = var_patterns;
	}

	// Replacing each variable with its patterns in a cartesian
	// product fashion (not forgetting updating their vardecl
	// accordingly)
	return product_substitute(pattern, var2patterns);
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

HandleMultimap XPatternMiner::shallow_patterns(const HandleSet& texts) const
{
	HandleMultimap pat2texts;
	for (const Handle& text : texts) {
		// Node or empty link, nothing to abstract
		if (text->is_node() or text->get_arity()) {
			pat2texts[text].insert(text);
			continue;
		}

		// Non empty link, let's abstract away the arguments
		HandleSeq rnd_vars = gen_rand_variables(text->get_arity());
		Handle vardecl = rnd_vars.size() == 1 ? rnd_vars[0]
			: createLink(rnd_vars, VARIABLE_LIST),
			body = createLink(rnd_vars, text->get_type()),
			pattern = Handle(createLambdaLink(vardecl, body));
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
	return createNode(VARIABLE_NODE, randstr("PatternMiner-", 2));
}

HandleSet XPatternMiner::select_values(const HandleMapSet& var2vals,
                                       const Handle& var) const
{
	HandleSet values;
	for (const HandleMap& var2val : var2vals)
		values.insert(var2val.at(var));
	return values;
}

HandleSet XPatternMiner::product_substitute(const Handle& pattern,
                                            const HandleMultimap& var2pats) const
{
	HandleSet patterns;
	for (auto map_it = var2pats.begin(); map_it != var2pats.end(); map_it++) {
		const Handle& variable = map_it->first;
		const HandleSet& subpatterns = map_it->second;

		// Reconstruct var2pats without that variable, and the ones
		// preceeding, and recursively call product_substitute to
		// obtain patterns without the subpatterns of that variables.
		HandleMultimap re_var2pats(std::next(map_it), var2pats.end());
		set_union_modify(patterns, product_substitute(pattern, re_var2pats));

		// Combine the patterns of that variables with the patterns of
		// the other variables.
		for (auto subpat_it = subpatterns.begin();
		     subpat_it != subpatterns.end(); ++subpat_it) {
			// Perform a single substitution of variable by subpat
			Handle npat = substitute(pattern, {{variable, *subpat_it}});

			// Add the remaining subpatterns sub-patterns to var2pats,
			// and recursively call product_substitute over it to
			// generate substitutions combined with subpat
			re_var2pats[variable].insert(std::next(subpat_it), subpatterns.end());
			set_union_modify(patterns, product_substitute(npat, re_var2pats));
		}
	}
	return patterns;
}

Handle XPatternMiner::substitute(const Handle& pattern,
                                 const HandleMap& var2pat) const
{
	// TODO, see Unify::substitute. Probably generalize it.
	return Handle::UNDEFINED;
}

} // namespace opencog
