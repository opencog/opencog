/*
 * MinerUtils.cc
 *
 * Copyright (C) 2018 SingularityNET Foundation
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

#include "MinerUtils.h"

#include <opencog/util/dorepeat.h>
#include <opencog/util/random.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atoms/core/LambdaLink.h>
#include <opencog/atoms/core/RewriteLink.h>
#include <opencog/atoms/pattern/PatternLink.h>
#include <opencog/atomutils/TypeUtils.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/query/BindLinkAPI.h>

#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/unique.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/range/algorithm_ext/erase.hpp>

namespace opencog
{

HandleSetSeq MinerUtils::shallow_abstract(const Valuations& valuations, unsigned ms)
{
	// Base case
	if (valuations.novar())
		return HandleSetSeq();

	// Recursive case
	HandleSetSeq shabs_per_var{front_shallow_abstract(valuations, ms)};
	HandleSetSeq remaining = shallow_abstract(valuations.erase_front(), ms);
	shabs_per_var.insert(shabs_per_var.end(), remaining.begin(), remaining.end());
	return shabs_per_var;
}

HandleSet MinerUtils::front_shallow_abstract(const Valuations& valuations, unsigned ms)
{
	HandleSet shabs;

	// No more variable to specialize from
	if (valuations.novar())
		return HandleSet();

	// Variable to specialize
	Handle var = valuations.front_variable();

	// Strongly connected valuations associated to that variable
	const SCValuations& var_scv(valuations.get_scvaluations(var));

	////////////////////////////
	// Shallow abtractions    //
	////////////////////////////

	// For each valuation create an abstraction (shallow pattern) of
	// the value associated to variable, and associate the remaining
	// valuations to it.
	HandleUCounter shapats;
	// Calculate how many valuations will be encompassed by these
	// shallow abstractions
	unsigned val_count = valuations.size() / var_scv.size();
	for (const HandleSeq& valuation : var_scv.valuations) {
		Handle shabs = val_shallow_abstract(valuation[0]);
		if (shabs)
			shapats[val_shallow_abstract(valuation[0])] += val_count;
	}

	// Only consider the shallow abstractions that reach the minimum
	// support
	for (const auto& shapat : shapats)
		if (ms <= shapat.second)
			shabs.insert(shapat.first);

	////////////////////////////////
	// Variable factorizations    //
	////////////////////////////////

	// Add all subsequent factorizable variables
	HandleSeq remvars(std::next(valuations.variables.varseq.begin()),
	                  valuations.variables.varseq.end());
	HandleUCounter facvars;
	for (const Handle& rv : remvars) {
		// Strongly connected valuations associated to that variable
		const SCValuations& rv_scv(valuations.get_scvaluations(rv));

		// Index of rv in rv_scv
		unsigned rv_idx = rv_scv.index(rv);

		// Whether var and rv are in the same strongly connected
		// valuations
		bool same_scv = rv_scv == var_scv;

		// Ref to keep track of the number of text instances where the
		// value of var is equal to the value to rv
		unsigned& rv_count = facvars[rv];

		// If they are in different stronly connected valuations, then
		// put all values of rv in a set, to quickly check if any
		// value is in.
		HandleUCounter rv_vals = same_scv ?
			HandleUCounter() : rv_scv.values(rv_idx);

		// Calculate how many valuations will be encompassed by this
		// variable factorization
		unsigned val_fac_count = val_count;
		if (not same_scv)
			val_fac_count /= rv_scv.size();

		for (const HandleSeq& valuation : var_scv.valuations) {
			// Value associated to var
			const Handle& val = valuation[0];

			// If the value of var is equal to that of rv, then
			// increase rv factorization count
			if (same_scv) {
				if (content_eq(val, valuation[rv_idx])) {
					rv_count += val_fac_count;
				}
			}
			else {
				auto it = rv_vals.find(val);
				if (it != rv_vals.end()) {
					rv_count += val_fac_count * it->second;
				}
			}

			// IF the minimum support has been reached, no need to
			// keep counting
			if (ms <= rv_count)
				break;
		}
	}

	// Only consider variable factorizations reaching the minimum
	// support
	for (const auto& fvar : facvars)
		if (ms <= fvar.second)
			shabs.insert(fvar.first);

	return shabs;
}

Handle MinerUtils::val_shallow_abstract(const Handle& value)
{
	// Node or empty link, nothing to abstract
	if (value->is_node() or value->get_arity() == 0)
		return value;

	Type tt = value->get_type();
	HandleSeq rnd_vars = gen_rand_variables(value->get_arity());
	Handle vardecl = variable_list(rnd_vars);

	// Links wrapped with LocalQuoteLink
	if (tt == AND_LINK) {
		return lambda(vardecl, local_quote(createLink(rnd_vars, tt)));
	}

	// Links wrapped with QuoteLink and UnquoteLinks
	if (tt == BIND_LINK or       // TODO: should probabably be replaced
                                // by scope link and its subtypes
	    tt == EVALUATION_LINK or
	    nameserver().isA(tt, FUNCTION_LINK) or
	    nameserver().isA(tt, VIRTUAL_LINK))
	{
		// TODO: comment out the following lines when issue #1843 on the
		// atomspace repository has been fixed (see more about that
		// below).

		// // Wrap variables in UnquoteLink
		// HandleSeq uq_vars;
		// for (Handle& var : rnd_vars)
		// 	uq_vars.push_back(unquote(var));

		// return lambda(vardecl, quote(createLink(uq_vars, tt)));

		// TODO: ignore these links for now!!! In order to support them
		// we first need to address issue #1843 on the atomspace
		// repository. That is because otherwise the quotations inside
		// these patterns get wrongly consumed down the line (especially
		// while being used in the specialization rule defined in
		// rules/specialization.scm).
		return Handle::UNDEFINED;
	}

	// Links to ignore (till supported)
	if (tt == DEFINE_LINK)
		return Handle::UNDEFINED;

	// Generic non empty link, abstract away all the arguments
	return lambda(vardecl, createLink(rnd_vars, tt));
}

Handle MinerUtils::variable_list(const HandleSeq& vars)
{
	return vars.size() == 1 ? vars[0]
		: createLink(vars, VARIABLE_LIST);
}

Handle MinerUtils::lambda(const Handle& vardecl, const Handle& body)
{
	return createLink(LAMBDA_LINK, vardecl, body);
}

Handle MinerUtils::quote(const Handle& h)
{
	return createLink(QUOTE_LINK, h);
}

Handle MinerUtils::unquote(const Handle& h)
{
	return createLink(UNQUOTE_LINK, h);
}

Handle MinerUtils::local_quote(const Handle& h)
{
	return createLink(LOCAL_QUOTE_LINK, h);
}

// TODO: take care of removing local quote in the composed
// sub-patterns, if it doesn't already
// TODO: replace by PutLink, if possible
Handle MinerUtils::compose(const Handle& pattern, const HandleMap& var2pat)
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

	// Perform composition of the pattern body with the sub-bodies)
	// TODO: perhaps use RewriteLink partial_substitute
	const Variables& variables = get_variables(pattern);
	Handle body = variables.substitute_nocheck(get_body(pattern), var2subody);
	body = RewriteLink::consume_quotations(vardecl, body, true);
	// If root AndLink then simplify the pattern
	if (body->get_type() == AND_LINK) {
		body = remove_useless_clauses(vardecl, body);
		body = remove_unary_and(body);
	}

	// Filter vardecl
	vardecl = filter_vardecl(vardecl, body);

	// Create the composed pattern
	if (vardecl)
		return createLink(HandleSeq{vardecl, body}, pattern->get_type());

	// No variable, the pattern is the body itself
	return body;
}

Handle MinerUtils::vardecl_compose(const Handle& vardecl, const HandleMap& var2subdecl)
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

Handle MinerUtils::remove_unary_and(const Handle& h)
{
	if (h->get_type() == AND_LINK and h->get_arity() == 1)
		return h->getOutgoingAtom(0);
	return h;
}

Handle MinerUtils::alpha_conversion(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return sc->alpha_convert();
	return pattern;
}

HandleSet MinerUtils::get_texts(const Handle& texts_cpt)
{
	// Retrieve all members of texts_cpt
	HandleSet texts;
	IncomingSet member_links = texts_cpt->getIncomingSetByType(MEMBER_LINK);
	for (const LinkPtr l : member_links) {
		Handle member = l->getOutgoingAtom(0);
		if (member != texts_cpt)
			texts.insert(member);
	}
	return texts;
}

unsigned MinerUtils::support(const Handle& pattern,
                             const HandleSet& texts,
                             unsigned ms)
{
	// Partition the pattern into strongly connected components
	HandleSeq cps(get_component_patterns(pattern));

	// Likely a constant pattern
	if (cps.empty())
	    return 1;

	// Otherwise calculate the frequency of each component
	std::vector<unsigned> freqs;
	boost::transform(cps, std::back_inserter(freqs),
	                 [&](const Handle& cp)
	                 { return component_support(cp, texts, ms); });

	// Return the product of all frequencies
	return boost::accumulate(freqs, 1, std::multiplies<unsigned>());
}

unsigned MinerUtils::component_support(const Handle& component,
                                       const HandleSet& texts,
                                       unsigned ms)
{
	if (totally_abstract(component))
		return texts.size();
	return restricted_satisfying_set(component, texts, ms)->get_arity();
}

bool MinerUtils::enough_support(const Handle& pattern,
                                const HandleSet& texts,
                                unsigned ms)
{
	return ms <= support(pattern, texts, ms);
}

HandleSetSeq MinerUtils::shallow_abstract(const Handle& pattern,
                                          const HandleSet& texts,
                                          unsigned ms)
{
	return shallow_abstract(Valuations(pattern, texts), ms);
}

Handle MinerUtils::mk_pattern(const Handle& vardecl, const HandleSeq& clauses)
{
	Handle fvd = filter_vardecl(vardecl, clauses);
	Handle body = 1 < clauses.size() ? createLink(clauses, AND_LINK) : clauses[0];
	if (fvd != nullptr and body != nullptr)
		return Handle(createLambdaLink(fvd, body));
	return Handle::UNDEFINED;
}

HandleSeq MinerUtils::get_component_patterns(const Handle& pattern)
{
	PatternLink pl(MinerUtils::get_vardecl(pattern),
	               MinerUtils::get_body(pattern));
	HandleSeq compats;
	const HandleSeqSeq comps(pl.get_components());
	for (unsigned i = 0; i < comps.size(); ++i)
	{
		Handle comp = mk_pattern(MinerUtils::get_vardecl(pattern), comps[i]);
		if (comp)
			compats.push_back(comp);
	}
	return compats;
}

HandleSeq MinerUtils::get_conjuncts(const Handle& pattern)
{
	if (pattern->get_type() == LAMBDA_LINK) {
		Handle body = get_body(pattern);
		if (body->get_type() == AND_LINK) {
			Handle vardecl = get_vardecl(pattern);
			HandleSeq conjs;
			for (const Handle& clause : body->getOutgoingSet()) {
				Handle conj = mk_pattern(vardecl, {clause});
				if (conj)
					conjs.push_back(conj);
			}
			return conjs;
		}
		return {pattern};
	}
	return {};
}

Handle MinerUtils::restricted_satisfying_set(const Handle& pattern,
                                             const HandleSet& texts,
                                             unsigned ms)
{
	static AtomSpace tmp_texts_as; // TODO: fix to be thread safe
	tmp_texts_as.clear();
	HandleSeq tmp_texts;
	for (const auto& text : texts)
		tmp_texts.push_back(tmp_texts_as.add_atom(text));

	// Avoid pattern matcher warning
	if (totally_abstract(pattern) and conjuncts(pattern) == 1)
		return tmp_texts_as.add_link(SET_LINK, tmp_texts);

	// Run the pattern matcher
	AtomSpace tmp_query_as(&tmp_texts_as);
	Handle tmp_pattern = tmp_query_as.add_atom(pattern),
		vardecl = get_vardecl(tmp_pattern),
		body = get_body(tmp_pattern),
		gl = tmp_query_as.add_link(GET_LINK, vardecl, body),
		results = satisfying_set(&tmp_texts_as, gl, ms);
	return results;
}

bool MinerUtils::totally_abstract(const Handle& pattern)
{
	// Check whether it is an abstraction to begin with
	if (pattern->get_type() != LAMBDA_LINK)
		return false;

	// If some variables are typed then the abstraction isn't total
	const Variables& vars = get_variables(pattern);
	if (not vars._simple_typemap.empty() or not vars._deep_typemap.empty())
		return false;

	// Make sure the body is either a variable, or a conjunction of
	// variables
	Handle body = get_body(pattern);
	if (body->get_type() == VARIABLE_NODE)
		return true;
	if (body->get_type() != AND_LINK)
		return false;
	for (const Handle& ch : body->getOutgoingSet())
		if (ch->get_type() != VARIABLE_NODE)
			return false;
	return true;
}

HandleSeq MinerUtils::gen_rand_variables(size_t n)
{
	HandleSeq variables;
	dorepeat (n)
		variables.push_back(gen_rand_variable());
	return variables;
}

Handle MinerUtils::gen_rand_variable()
{
	return createNode(VARIABLE_NODE, randstr("$PM-"));
}

const Variables& MinerUtils::get_variables(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return RewriteLinkCast(pattern)->get_variables();
	static Variables empty_variables;
	return empty_variables;
}

Handle MinerUtils::get_vardecl(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc) {
		Handle vardecl = sc->get_vardecl();
		if (not vardecl)
			vardecl = sc->get_variables().get_vardecl();
		return vardecl;
	}
	return Handle::UNDEFINED;
}

const Handle& MinerUtils::get_body(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return RewriteLinkCast(pattern)->get_body();
	return pattern;
}

unsigned MinerUtils::conjuncts(const Handle& pattern)
{
	if (pattern->get_type() == LAMBDA_LINK) {
		if (get_body(pattern)->get_type() == AND_LINK)
			return get_body(pattern)->get_arity();
		return 1;
	}
	return 0;
}

Handle MinerUtils::remove_useless_clauses(const Handle& pattern)
{
	Handle vardecl = get_vardecl(pattern),
		body = get_body(pattern);
	body = remove_useless_clauses(vardecl, body);
	return Handle(createLambdaLink(vardecl, body));
}

Handle MinerUtils::remove_useless_clauses(const Handle& vardecl,
                                          const Handle& body)
{
	// Copy clauses
	HandleSeq clauses = body->get_type() == AND_LINK ?
		body->getOutgoingSet() : HandleSeq{body};
	// Remove useless ones
	remove_useless_clauses(vardecl, clauses);
	// Reconstruct AndLink if necessary
	return clauses.size() == 1 ? clauses[0] : Handle(createLink(clauses, AND_LINK));
}

void MinerUtils::remove_useless_clauses(const Handle& vardecl, HandleSeq& clauses)
{
	remove_constant_clauses(vardecl, clauses);
	remove_redundant_clauses(clauses);
}

void MinerUtils::remove_constant_clauses(const Handle& vardecl, HandleSeq& clauses)
{
	// Get Variables
	VariableListPtr vl = createVariableList(vardecl);
	const HandleSet& vars = vl->get_variables().varset;

	// Remove constant clauses
	auto is_constant = [&](const Handle& clause) {
		return not any_unquoted_unscoped_in_tree(clause, vars); };
	boost::remove_erase_if(clauses, is_constant);
}

void MinerUtils::remove_redundant_clauses(HandleSeq& clauses)
{
	// Check that each clause is not a subtree of another clause,
	// remove it otherwise.
	for (auto it = clauses.begin(); it != clauses.end();) {
		// Take all clauses except *it
		HandleSeq others(clauses.begin(), it);
		others.insert(others.end(), std::next(it), clauses.end());

		// Make sure *it is not a subtree of any other
		if (is_unquoted_unscoped_in_any_tree(others, *it))
			it = clauses.erase(it);
		else
			++it;
	}
}

Handle MinerUtils::expand_conjunction(const Handle& cnjtion, const Handle& pattern)
{
	// Copy variables from cnjtion and pattern, as they are gonna be modified
	Variables cnjtion_vars = get_variables(cnjtion);
	Variables pattern_vars = get_variables(pattern);

	// For each variable of pattern that is in cnjtion, alpha convert
	// to avoid variable name collision.
	HandleMap aconv;
	for (const Handle& var : pattern_vars.varset) {
		if (cnjtion_vars.is_in_varset(var)) {
			Handle nvar;
			bool used;
			do {
				nvar = createNode(VARIABLE_NODE, randstr(var->get_name() + "-"));
				// Make sure it is not in cnjtion_vars or pattern_vars
				used = cnjtion_vars.is_in_varset(nvar) or pattern_vars.is_in_varset(nvar);
			} while (used);
			aconv[var] = nvar;
		}
	}

	// Expand cnjtion_vars with pattern_vars, alpha converting
	// pattern_vars if necessary
	if (not aconv.empty()) {
		Handle avdecl = pattern_vars.substitute_nocheck(get_vardecl(pattern), aconv);
		pattern_vars = createVariableList(avdecl)->get_variables();
	}
	cnjtion_vars.extend(pattern_vars);

	// Alpha-convert pattern body if necessary
	Handle pattern_body = aconv.empty() ? get_body(pattern)
		: get_variables(pattern).substitute_nocheck(get_body(pattern), aconv);

	// Expand cnjtion_body with pattern, flattening cnjtion_body if necessary
	const Handle& cnjtion_body = get_body(cnjtion);
	HandleSeq nclauses = cnjtion_body->get_type() == AND_LINK ?
		cnjtion_body->getOutgoingSet() : HandleSeq{cnjtion_body};
	nclauses.push_back(pattern_body);

	// Remove redundant clauses
	boost::sort(nclauses);
	boost::erase(nclauses,
	             boost::unique<boost::return_found_end>(nclauses));

	// Recreate combined pattern
	Handle nvardecl = cnjtion_vars.get_vardecl(),
		nbody = createLink(nclauses, AND_LINK),
		npattern = Handle(createLambdaLink(nvardecl, nbody));

	return npattern;
}

} // namespace opencog
