/*
 * Miner.cc
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

#include "Miner.h"

#include <opencog/util/Logger.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/dorepeat.h>
#include <opencog/util/random.h>

#include <opencog/atoms/execution/Instantiator.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atoms/core/LambdaLink.h>
#include <opencog/atomutils/TypeUtils.h>
#include <opencog/unify/Unify.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/query/BindLinkAPI.h>

#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/algorithm/transform.hpp>

#include <functional>

namespace opencog
{

// TODO:
// 7. make sure that filtering is still meaningfull

XPMParameters::XPMParameters(unsigned ms, unsigned iconjuncts,
                             const Handle& ipat, int maxd, double io)
	: minsup(ms), initconjuncts(iconjuncts), initpat(ipat),
	  maxdepth(maxd), info(io)
{
	// Provide initial pattern if none
	if (not initpat) {
		HandleSeq vars = Miner::gen_rand_variables(initconjuncts);
		Handle vardecl = 1 < vars.size() ?
		                     createLink(vars, VARIABLE_LIST) : vars[0];
		Handle body = 1 < vars.size() ?
		                  createLink(vars, AND_LINK) : vars[0];
		initpat = createLambdaLink(vardecl, body);
	}

	// Wrap a Lambda around if none
	if (initpat->get_type() != LAMBDA_LINK)
		initpat = createLink(LAMBDA_LINK, initpat);

	// Provide a variable declaration if none
	RewriteLinkPtr sc = RewriteLinkCast(initpat);
	if (not sc->get_vardecl()) {
		Handle vardecl = sc->get_variables().get_vardecl(),
			body = sc->get_body();
		initpat = createLink(initpat->get_type(), vardecl, body);
	}

	// Overwrite initconjuncts if necessary
	initconjuncts = Miner::conjuncts(initpat);
}

Miner::Miner(AtomSpace& as, const XPMParameters& prm)
	: text_as(as), param(prm) {}

HandleTree Miner::operator()()
{
	HandleSet texts;
	text_as.get_handles_by_type(std::inserter(texts, texts.end()),
	                            opencog::ATOM, true);
	// If the initial pattern is specialized, this initial filtering
	// may save some computation
	HandleSet fltexts = filter_texts(param.initpat, texts);
	return specialize(param.initpat, fltexts, param.maxdepth);
}

HandleTree Miner::specialize(const Handle& pattern,
                             const HandleSet& texts,
                             int maxdepth)
{
	// return specialize_alt(pattern, texts, Valuations(pattern, texts), maxdepth);
	return specialize(pattern, texts, Valuations(pattern, texts), maxdepth);
}

HandleTree Miner::specialize(const Handle& pattern,
                             const HandleSet& texts,
                             const Valuations& valuations,
                             int maxdepth)
{
	// One of the termination criteria has been reached
	if (terminate(pattern, texts, valuations, maxdepth))
		return HandleTree();

	// Produce specializations from other variables than the front
	// one.
	HandleTree patterns = specialize(pattern, texts, valuations.erase_front(),
	                                 maxdepth);

	// Produce specializations from shallow abstractions on the front
	// variable, and so recusively.
	HandleTree shabs_pats = specialize_shabs(pattern, texts, valuations,
	                                         maxdepth);

	// Merge specializations to patterns while discarding duplicates
	patterns = merge_patterns({patterns, shabs_pats});

	return patterns;
}

HandleTree Miner::specialize_alt(const Handle& pattern,
                                 const HandleSet& texts,
                                 const Valuations& valuations,
                                 int maxdepth)
{
	// One of the termination criteria has been reached
	if (terminate(pattern, texts, valuations, maxdepth))
		return HandleTree();

	HandleTree patterns;
	Variables vars = get_variables(pattern);

	// Calculate all shallow abstractions of pattern
	HandleSetSeq shabs = shallow_abstract(valuations, param.minsup);

	// Generate all associated specializations
	for (unsigned i = 0; i < shabs.size(); i++) {
		for (const Handle& shapat : shabs[i]) {
			// Compose pattern with shapat to obtain a specialization,
			// and recursively specialize the result
			HandleTree npats = specialize_shapat(pattern, texts,
			                                     vars.varseq[i], shapat,
			                                     maxdepth);

			// Insert specializations
			patterns = merge_patterns({patterns, npats});
		}
	}
	return patterns;
}

HandleTree Miner::specialize_shabs(const Handle& pattern,
                                   const HandleSet& texts,
                                   const Valuations& valuations,
                                   int maxdepth)
{
	// Generate shallow patterns of the first variable of the
	// valuations and associate the remaining valuations (excluding
	// that variable) to them.
	HandleSet shapats = front_shallow_abstract(valuations, param.minsup);

	// No shallow abstraction to use for specialization
	if (shapats.empty())
		return HandleTree();

	// For each shallow abstraction, create a specialization from
	// pattern by composing it, and recursively specialize the result
	// with the new resulting valuations.
	HandleTree patterns;
	Handle var = valuations.front_variable();
	for (const auto& shapat : shapats)
	{
		// Specialize pattern by composing it with shapat, and
		// specialize the result recursively
		HandleTree npats
			= specialize_shapat(pattern, texts, var, shapat, maxdepth);

		// Insert specializations
		patterns = merge_patterns({patterns, npats});
	}
	return patterns;
}

HandleTree Miner::specialize_shapat(const Handle& pattern,
                                    const HandleSet texts,
                                    const Handle& var,
                                    const Handle& shapat,
                                    int maxdepth)
{
	// Perform the composition (that is specialize)
	Handle npat = compose(pattern, {{var, shapat}});

	// If the specialization has too few conjuncts, dismiss it.
	if (conjuncts(npat) < param.initconjuncts)
		return HandleTree();

	// Generate the corresponding text
	HandleSet fltexts = filter_texts(npat, texts);

	// That specialization doesn't have enough support, skip it
	// and its specializations.
	if (not enough_support(npat, fltexts))
		return HandleTree();

	// Specialize npat from all variables (with new valuations)
	HandleTree nvapats = specialize(npat, fltexts, maxdepth - 1);

	// Return npat and its children
	return HandleTree(npat, {nvapats});
}

bool Miner::terminate(const Handle& pattern,
                      const HandleSet& texts,
                      const Valuations& valuations,
                      int maxdepth) const
{	
	return
		// We have reached the maximum depth
		maxdepth == 0 or
		// The pattern is constant, no specialization is possible
		pattern->get_type() != LAMBDA_LINK or
		// There is no more variable to specialize from
		valuations.novar() or
		// The pattern doesn't have enough support
		// TODO: it seems the text is always filtered prior anyway
		not enough_support(pattern, texts);
}

bool Miner::enough_support(const Handle& pattern,
                           const HandleSet& texts) const
{
	return param.minsup <= freq(pattern, texts, param.minsup);
}

unsigned Miner::freq(const Handle& pattern,
                     const HandleSet& texts,
                     int maxf) const
{
	HandleSeq cps(get_component_patterns(pattern));
	// HandleSeq cps(get_conjuncts(pattern));

	// Likely a constant pattern
	if (cps.empty())
	    return 1;

	// Otherwise aggregate the frequencies in a heuristic fashion
	std::vector<unsigned> freqs;
	boost::transform(cps, std::back_inserter(freqs),
	                 [&](const Handle& cp)
	                 { return freq_component(cp, texts, maxf); });
	return freq(freqs);
}

unsigned Miner::freq_component(const Handle& component,
                               const HandleSet& texts,
                               int maxf) const
{
	if (totally_abstract(component))
		return texts.size();
	return restricted_satisfying_set(component, texts, maxf)->get_arity();
}

unsigned Miner::freq(const std::vector<unsigned>& freqs) const
{
	double minf = *boost::min_element(freqs),
		timesf = boost::accumulate(freqs, 1, std::multiplies<unsigned>()),
		f = param.info * minf + (1 - param.info) * timesf;
	return std::floor(f);
}

HandleSet Miner::filter_texts(const Handle& pattern,
                              const HandleSet& texts) const
{
	// No need to filter if most abstract
	if (totally_abstract(pattern))
		return texts;

	// If it has more than one conjunct, then TODO: it's probably the
	// union of the texts of all its conjuncts.
	if (1 < conjuncts(pattern))
		return texts;

	// Otherwise it is a single conjunct pattern
	HandleSet filtrd;
	for (const auto& text : texts)
		if (match(pattern, text))
			filtrd.insert(text);
	return filtrd;
}

bool Miner::match(const Handle& pattern, const Handle& text) const
{
	// If constant pattern, matching ammounts to equality
	if (pattern->get_type() != LAMBDA_LINK)
		return content_eq(pattern, text);

	// Otherwise see if pattern matches text
	return (bool)matched_results(pattern, text);
}

Handle Miner::matched_results(const Handle& pattern, const Handle& text) const
{
	// If I use a temporary atomspace on stack, then the atoms in it
	// get deleted, grrrr, would need to use a smart pointer.
	tmp_as.clear();
	tmp_as.add_atom(text);
	AtomSpace tmp_pattern_as(&tmp_as);
	Handle tmp_pattern = tmp_pattern_as.add_atom(pattern),
		tmp_text = 	tmp_as.add_atom(text),
		ml = tmp_as.add_link(MAP_LINK, tmp_pattern, tmp_text);
	Instantiator inst(&tmp_as);
	return inst.execute(ml);
}

HandleSetSeq Miner::shallow_abstract(const Handle& pattern,
                                     const HandleSet& texts,
                                     unsigned ms)
{
	return shallow_abstract(Valuations(pattern, texts), ms);
}

HandleSetSeq Miner::shallow_abstract(const Valuations& valuations, unsigned ms)
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

HandleSet Miner::front_shallow_abstract(const Valuations& valuations, unsigned ms)
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
	for (const HandleSeq& valuation : var_scv.valuations)
		shapats[val_shallow_abstract(valuation[0])] += val_count;

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

Handle Miner::val_shallow_abstract(const Handle& value)
{
	// Node or empty link, nothing to abstract
	if (value->is_node() or value->get_arity() == 0)
		return value;

	Type tt = value->get_type();
	HandleSeq rnd_vars = gen_rand_variables(value->get_arity());
	Handle vardecl = variable_list(rnd_vars),
		body = createLink(rnd_vars, tt);

	// Links wrapped with LocalQuoteLink
	if (tt == AND_LINK) {
		return lambda(vardecl, local_quote(body));
	}
	// Links wrapped with QuoteLink and UnquoteLinks
	if (tt == BIND_LINK or
	    tt == EVALUATION_LINK or
	    tt == EXECUTION_OUTPUT_LINK)
	{
		// Wrap variables in UnquoteLink
		HandleSeq uq_vars;
		for (Handle& var : rnd_vars)
			uq_vars.push_back(unquote(var));

		return lambda(vardecl, quote(body));
	}

	// Generic non empty link, let's abstract away all the arguments
	return lambda(vardecl, body);
}

Handle Miner::variable_list(const HandleSeq& vars)
{
	return vars.size() == 1 ? vars[0]
		: createLink(vars, VARIABLE_LIST);
}

Handle Miner::lambda(const Handle& vardecl, const Handle& body)
{
	return createLink(LAMBDA_LINK, vardecl, body);
}

Handle Miner::quote(const Handle& h)
{
	return createLink(QUOTE_LINK, h);
}

Handle Miner::unquote(const Handle& h)
{
	return createLink(UNQUOTE_LINK, h);
}

Handle Miner::local_quote(const Handle& h)
{
	return createLink(LOCAL_QUOTE_LINK, h);
}

// TODO: take care of removing local quote in the composed
// sub-patterns, if it doesn't already
// TODO: replace by PutLink, if possible
Handle Miner::compose(const Handle& pattern, const HandleMap& var2pat)
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
	HandleSeq subodies = variables.make_sequence(var2subody);

	// Perform composition of the pattern body with the sub-bodies)
	// TODO: perhaps use RewriteLink partial_substitute
	Handle body = variables.substitute_nocheck(get_body(pattern), subodies);
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

Handle Miner::vardecl_compose(const Handle& vardecl, const HandleMap& var2subdecl)
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

Handle Miner::remove_unary_and(const Handle& h)
{
	if (h->get_type() == AND_LINK and h->get_arity() == 1)
		return h->getOutgoingAtom(0);
	return h;
}

Handle Miner::alpha_conversion(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return sc->alpha_convert();
	return pattern;
}

Handle Miner::mk_pattern(const Handle& vardecl, const HandleSeq& clauses)
{
	Handle fvd = filter_vardecl(vardecl, clauses);
	Handle body = 1 < clauses.size() ? createLink(clauses, AND_LINK) : clauses[0];
	if (fvd != nullptr and body != nullptr)
		return Handle(createLambdaLink(fvd, body));
	return Handle::UNDEFINED;
}

HandleSeq Miner::get_component_patterns(const Handle& pattern)
{
	PatternLink pl(Miner::get_vardecl(pattern),
	               Miner::get_body(pattern));
	HandleSeq compats;
	const HandleSeqSeq comps(pl.get_components());
	for (unsigned i = 0; i < comps.size(); ++i)
	{
		Handle comp = mk_pattern(Miner::get_vardecl(pattern), comps[i]);
		if (comp)
			compats.push_back(comp);
	}
	return compats;
}

HandleSeq Miner::get_conjuncts(const Handle& pattern)
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

Handle Miner::restricted_satisfying_set(const Handle& pattern,
                                        const HandleSet& texts,
                                        int maxf)
{
	static AtomSpace tmp_text_as;
	tmp_text_as.clear();
	HandleSeq tmp_texts;
	for (const auto& text : texts)
		tmp_texts.push_back(tmp_text_as.add_atom(text));

	// Avoid pattern matcher warning
	// TODO: support 1 < conjuncts
	if (totally_abstract(pattern) and conjuncts(pattern) == 1)
		return tmp_text_as.add_link(SET_LINK, tmp_texts);

	// Run the pattern matcher
	AtomSpace tmp_query_as(&tmp_text_as);
	Handle tmp_pattern = tmp_query_as.add_atom(pattern),
		vardecl = get_vardecl(tmp_pattern),
		body = get_body(tmp_pattern),
		gl = tmp_query_as.add_link(GET_LINK, vardecl, body),
		results = (maxf < 0 ? satisfying_set(&tmp_text_as, gl)
		           : satisfying_set(&tmp_text_as, gl, maxf));
	return results;
}

bool Miner::totally_abstract(const Handle& pattern)
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

HandleSeq Miner::gen_rand_variables(size_t n)
{
	HandleSeq variables;
	dorepeat (n)
		variables.push_back(gen_rand_variable());
	return variables;
}

Handle Miner::gen_rand_variable()
{
	return createNode(VARIABLE_NODE, randstr("$PM-"));
}

const Variables& Miner::get_variables(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return RewriteLinkCast(pattern)->get_variables();
	static Variables empty_variables;
	return empty_variables;
}

Handle Miner::get_vardecl(const Handle& pattern)
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

const Handle& Miner::get_body(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return RewriteLinkCast(pattern)->get_body();
	return pattern;
}

unsigned Miner::conjuncts(const Handle& pattern)
{
	if (pattern->get_type() == LAMBDA_LINK) {
		if (get_body(pattern)->get_type() == AND_LINK)
			return get_body(pattern)->get_arity();
		return 1;
	}
	return 0;
}

Handle Miner::remove_useless_clauses(const Handle& vardecl,
                                     const Handle& body)
{
	Handle res = remove_constant_clauses(vardecl, body);

	// Check that each clause isn't a subtree of another clause
	const HandleSeq& outs = res->getOutgoingSet();
	HandleSeq nouts;
	for (auto it = outs.begin(); it != outs.end(); ++it) {
		HandleSeq others(outs.begin(), it);
		others.insert(others.end(), std::next(it), outs.end());
		if (not is_unquoted_unscoped_in_any_tree(others, *it))
			nouts.push_back(*it);
	}
	res = createLink(nouts, AND_LINK);

	return res;
}

Handle Miner::remove_constant_clauses(const Handle& vardecl,
                                      const Handle& clauses)
{
	VariableListPtr vl = createVariableList(vardecl);
	HandleSet vars = vl->get_variables().varset;

	// Remove constant clauses
	Type t = clauses->get_type();
	HandleSeq hs;
	if (t == AND_LINK) {
		for (const Handle& clause : clauses->getOutgoingSet()) {
			if (any_unquoted_unscoped_in_tree(clause, vars)) {
				hs.push_back(clause);
			}
		}
	} else if (any_unquoted_unscoped_in_tree(clauses, vars)) {
		return clauses;
	}
	return createLink(hs, AND_LINK);
}

} // namespace opencog
