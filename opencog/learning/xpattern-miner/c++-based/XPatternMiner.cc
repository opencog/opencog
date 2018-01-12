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

namespace opencog
{

// TODO:
// 1. test if removing duplicates while merging is really necessary
// 2. test if associating valuations to shabs is really necessary
// 3. test if cash is really necessary
// 4. test if using counter over valuations can speed up
// 5. have valuations reflect disconnected components, and split them
//    into strongly connected components when querying the pattern matcher.

XPMParameters::XPMParameters(unsigned ms, unsigned iconjuncts,
                             const Handle& ipat, int maxd, int maxp)
	: minsup(ms), initconjuncts(iconjuncts), initpat(ipat),
	  maxdepth(maxd), maxpats(maxp)
{
	// Provide initial pattern if none
	if (not initpat) {
		HandleSeq vars = XPatternMiner::gen_rand_variables(initconjuncts);
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
	initconjuncts = XPatternMiner::conjuncts(initpat);
}

XPatternMiner::XPatternMiner(AtomSpace& as, const XPMParameters& prm)
	: text_as(as), param(prm) {}

HandleTree XPatternMiner::operator()()
{
	HandleSet texts;
	text_as.get_handles_by_type(std::inserter(texts, texts.end()),
	                            opencog::ATOM, true);
	return specialize(param.initpat, texts, param.maxdepth);
}

HandleTree XPatternMiner::specialize(const Handle& pattern,
                                     const HandleSet& texts,
                                     int maxdepth)
{
	// If the initial pattern is specialized, this initial filtering
	// may save some computation
	HandleUCounter fltexts = filter_texts(pattern, HandleUCounter(texts));
	return specialize(pattern, fltexts, maxdepth);
}

HandleTree XPatternMiner::specialize(const Handle& pattern,
                                     const HandleUCounter& texts,
                                     int maxdepth)
{
	return specialize(pattern, texts, mk_valuations(pattern, texts), maxdepth);
}

HandleTree XPatternMiner::specialize(const Handle& pattern,
                                     const HandleUCounter& texts,
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

	// Produce specializations from mapping non-front variables to the
	// front one, and so recursively.
	HandleTree vared_pats = specialize_vared(pattern, texts, valuations,
	                                         maxdepth);

	// Merge specializations to patterns while discarding duplicates
	patterns = merge_patterns({patterns, shabs_pats, vared_pats});

	return patterns;
}

HandleTree XPatternMiner::specialize_shabs(const Handle& pattern,
                                           const HandleUCounter& texts,
                                           const Valuations& valuations,
                                           int maxdepth)
{
	// Generate shallow patterns of the first variable of the
	// valuations and associate the remaining valuations (excluding
	// that variable) to them.
	HandleValuationsMap sha2vals = shallow_abstract(valuations);

	// No shallow abstraction to use for specialization
	if (sha2vals.empty())
		return HandleTree();

	// For each shallow abstraction, create a specialization from
	// pattern by composing it, and recursively specialize the result
	// with the remaining valuations.
	HandleTree patterns;
	Handle var = valuations.front_variable();
	for (const auto& s2v : sha2vals)
	{
		// Alpha convert to make sure it doesn't share variables with
		// pattern
		Handle subpat = alpha_conversion(s2v.first);

		// Perform the composition (that is specialize)
		Handle npat = compose(pattern, {{var, subpat}});

		// If the specialization has too few conjuncts, dismiss it.
		if (conjuncts(npat) < param.initconjuncts)
			continue;
		
		// Generate the corresponding text
		// TODO: there is probably a better way to pass just the valuations and
		// to reconstitute the next valuations based on the obtained
		// pattern.
		HandleUCounter fltexts = filter_texts(npat, texts);

		// That specialization doesn't have enough support, skip it
		// and its specializations.
		// TODO: instead you can just count s2v.second
		if (not enough_support(npat, fltexts))
			continue;

		// TODO: add a cash, so that if npat and its children have
		// already be produced, do not re-calculate its
		// specializations. Use a XPatternMiner member, say an ordered
		// or unordered set with rapid element access to hold the
		// produced patterns thus far.

		// Specialize npat from all variables (with new valuations)
		HandleTree nvapats = specialize(npat, fltexts, maxdepth - 1);

		// Insert specializations
		// HandleTree npats(npat, {rempats, nvapats});
		HandleTree npats(npat, {nvapats});
		patterns = merge_patterns({patterns, npats});
	}
	return patterns;
}

HandleTree XPatternMiner::specialize_vared(const Handle& pattern,
                                           const HandleUCounter& texts,
                                           const Valuations& valuations,
                                           int maxdepth)
{
	// Generate all variable sets compatible with co-occurrence of
	// values (equal to the front value of each valuation).
	HandleValuationsMap vs2vals = variable_reduce(valuations);

	// No variable reduction to use for specialization
	if (vs2vals.empty())
		return HandleTree();

	// For each variable set, create a specialization where all
	// variables in that set are substituted by the front variable,
	// and the resulting pattern recursively specialized with the
	// remaining valuations.
	HandleTree patterns;
	Handle var = valuations.front_variable();
	for (const auto v2v : vs2vals) {
		// Perform the composition (that is specialize)
		Handle npat = compose(pattern, {{v2v.first, var}});

		// If the specialization has too few conjuncts, dismiss it.
		if (conjuncts(npat) < param.initconjuncts)
			continue;

		// Generate the corresponding text
		// TODO: there is probably a better way to pass just the valuations and
		// to reconstitute the next valuations based on the obtained
		// pattern.
		HandleUCounter fltexts = filter_texts(npat, texts);

		// That specialization doesn't have enough support, skip it
		// and its specializations.
		// TODO: instead you can just count s2v.second
		if (not enough_support(npat, fltexts))
			continue;

		// TODO: add a cash, so that if npat and its children have
		// already be produced, do not re-calculate its
		// specializations. Use a XPatternMiner member, say an ordered
		// or unordered set with rapid element access to hold the
		// produced patterns thus far.

		// Specialize npat from all variables (with new valuations)
		HandleTree nvapats = specialize(npat, fltexts, maxdepth - 1);

		// Insert specializations
		HandleTree npats(npat, {nvapats});
		patterns = merge_patterns({patterns, npats});
	}
	return patterns;
}

HandleValuationsMap XPatternMiner::variable_reduce(const Valuations& valuations) const
{
	// No more variable to specialize from
	if (valuations.novar())
		return HandleValuationsMap();

	// Variable to specialize
	Handle var = valuations.front_variable();

	// For each valuation pick up variables with values equal to the
	// front value
	HandleValuationsMap vs2vals;
	for (const HandleSeq& valuation : valuations.values) {
		for (unsigned i = 1; i < valuation.size(); ++i) {
			if (content_eq(valuation[0], valuation[i]))
				// TODO: for now we don't care about updating the
				// valuations
				vs2vals.insert({valuations.variable(i), valuations});
		}
	}
	return vs2vals;
}

bool XPatternMiner::terminate(const Handle& pattern,
                              const HandleUCounter& texts,
                              const Valuations& valuations,
                              int maxdepth) const
{
	
	return
		// We have reached the maximum depth
		maxdepth == 0 or
		// The pattern is constant, no specialization is possible
		pattern->get_type() != LAMBDA_LINK or
		// There is no move variable to specialize from
		valuations.novar() or
		// The pattern doesn't have enough support
		// TODO: it seems the text is always filtered prior anyway
		not enough_support(pattern, texts);
}

Valuations XPatternMiner::mk_valuations(const Handle& pattern,
                                        // TODO: replace by HandleSet
                                        const HandleUCounter& texts)
{
	Handle satset = restrict_satisfying_set(pattern, texts);
	return Valuations(get_variables(pattern), satset);
}

bool XPatternMiner::enough_support(const Handle& pattern,
                                   const HandleUCounter& texts) const
{
	return param.minsup <= freq(pattern, texts, param.minsup);
}

bool XPatternMiner::enough_support(const HandleUCounter& texts) const
{
	return param.minsup <= freq(texts, param.minsup);
}

unsigned XPatternMiner::freq(const Handle& pattern,
                             const HandleUCounter& texts,
                             int maxf) const
{
	if (totally_abstract(pattern))
	{
		unsigned ng = conjuncts(pattern);
		int nmf = maxf < 0 ? maxf : (int)std::ceil((double)maxf/(double)ng);
		return ng * freq(texts, nmf);
	}

	// If the pattern has more than one conjunct, then it is assumed that
	// the count of each text in texts is 1, thus we just need to run
	// the pattern over the whole texts without worrying about counts
	if (1 < conjuncts(pattern))
		return restrict_satisfying_set(pattern, texts, maxf)->get_arity();

	// Otherwise, assuming the texts has already been filtered, it is
	// merely the texts total count. TODO: it would safer to put some
	// assert to check if that assumption really hold
	return freq(texts, maxf);
}

unsigned XPatternMiner::freq(const HandleUCounter& texts, int maxf) const
{
	if (maxf < 0)
		return texts.total_count();

	// Otherwise only count up to maxf
	unsigned count = 0;
	for (const auto& text : texts) {
		count += text.second;
		if (maxf <= (int)count)
			return count;
	}
	return count;
}

HandleUCounter XPatternMiner::filter_texts(const Handle& pattern,
                                           const HandleUCounter& texts) const
{
	// No need to filter if most abstract
	if (totally_abstract(pattern))
		return texts;

	// If it has more than one conjunct, then TODO: it's probably the
	// union of the texts of all its conjuncts.
	if (1 < conjuncts(pattern))
		return texts;

	// Otherwise it is a single conjunct pattern
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
	return (bool)matched_results(pattern, text);
}

Handle XPatternMiner::matched_results(const Handle& pattern,
                                      const Handle& text) const
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

Handle XPatternMiner::restrict_satisfying_set(const Handle& pattern,
                                              const HandleUCounter& texts,
                                              int maxf) const
{	
	// TODO: leaking!!!
	AtomSpace* tmp_text_as = new AtomSpace();
	HandleSeq tmp_texts;
	for (const auto& text : texts)
		tmp_texts.push_back(tmp_text_as->add_atom(text.first));

	// Avoid pattern matcher warning
	// TODO: support 1 < conjuncts
	if (totally_abstract(pattern) and conjuncts(pattern) == 1)
		return tmp_text_as->add_link(SET_LINK, tmp_texts);

	// Run the pattern matcher
	AtomSpace tmp_query_as(tmp_text_as);
	Handle tmp_pattern = tmp_query_as.add_atom(pattern),
		vardecl = get_vardecl(tmp_pattern),
		body = get_body(tmp_pattern),
		gl = tmp_query_as.add_link(GET_LINK, vardecl, body),
		results = (maxf < 0 ? satisfying_set(tmp_text_as, gl)
		           : satisfying_set(tmp_text_as, gl, maxf));
	return results;
}

// TODO: maybe we want to associate the corresponding filter text as well
HandleValuationsMap XPatternMiner::shallow_abstract(const Valuations& valuations)
{
	// No more variable to specialize from
	if (valuations.novar())
		return HandleValuationsMap();

	// Variable to specialize
	Handle var = valuations.front_variable();

	// For each valuation create an abstraction (shallow pattern) of
	// the value associated to variable, and associate the remaining
	// valuations to it.
	HandleValuationsMap sha2vals;
	for (const HandleSeq& valuation : valuations.values) {
		auto val_it = valuation.begin();
		Handle shapat = shallow_abstract(*val_it);
		auto s2v_it = sha2vals.find(shapat);
		if (s2v_it == sha2vals.end()) {
			// Construct the remaining valuations
			Variables rembles(valuations.variables);
			rembles.erase(var);
			s2v_it = sha2vals.insert({shapat, Valuations(rembles)}).first;
		}
		// Append the remaining values to existing valuations
		// TODO: Instead of copying the entire remaining values,
		// one could use boost::range, or simply C arrays. On the
		// other hand this might be negligeable.
		HandleSeq remval(++val_it, valuation.end());
		if (not s2v_it->second.novar())
			s2v_it->second.values.push_back(remval); // TODO use emplace_back
	}
	return sha2vals;
}

Handle XPatternMiner::shallow_abstract(const Handle& text)
{
	// Node or empty link, nothing to abstract
	if (text->is_node() or text->get_arity() == 0)
		return text;

	Type tt = text->get_type();

	// Links wrapped with LocalQuoteLink
	if (tt == AND_LINK) {
		HandleSeq rnd_vars = gen_rand_variables(text->get_arity());
		Handle vardecl = variable_list(rnd_vars),
			body = pattern_as.add_link(tt, rnd_vars),
			pattern = lambda(vardecl, local_quote(body));
		return pattern;
	}
	// Links wrapped with QuoteLink and UnquoteLinks
	if (tt == BIND_LINK or
	    tt == EVALUATION_LINK or
	    tt == EXECUTION_OUTPUT_LINK)
	{
		HandleSeq rnd_vars = gen_rand_variables(text->get_arity());
		// Wrap variables in UnquoteLink
		HandleSeq uq_vars;
		for (Handle& var : rnd_vars)
			uq_vars.push_back(unquote(var));

		Handle vardecl = variable_list(rnd_vars),
			body = pattern_as.add_link(tt, uq_vars),
			pattern = lambda(vardecl, quote(body));
		return pattern;
	}

	// Generic non empty link, let's abstract away all the arguments
	HandleSeq rnd_vars = gen_rand_variables(text->get_arity());
	Handle vardecl = variable_list(rnd_vars),
		body = pattern_as.add_link(tt, rnd_vars),
		pattern = lambda(vardecl, body);
	return pattern;
}

Handle XPatternMiner::variable_list(const HandleSeq& vars)
{
	return vars.size() == 1 ? vars[0]
		: pattern_as.add_link(VARIABLE_LIST, vars);
}

Handle XPatternMiner::lambda(const Handle& vardecl, const Handle& body)
{
	return pattern_as.add_link(LAMBDA_LINK, vardecl, body);
}

Handle XPatternMiner::quote(const Handle& h)
{
	return pattern_as.add_link(QUOTE_LINK, h);
}

Handle XPatternMiner::unquote(const Handle& h)
{
	return pattern_as.add_link(UNQUOTE_LINK, h);
}

Handle XPatternMiner::local_quote(const Handle& h)
{
	return pattern_as.add_link(LOCAL_QUOTE_LINK, h);
}

bool XPatternMiner::totally_abstract(const Handle& pattern) const
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

// TODO: take care of removing local quote in the composed
// sub-patterns, if it doesn't already
// TODO: replace by PutLink, if possible
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
	HandleSeq subodies = variables.make_sequence(var2subody);

	// Perform composition of the pattern body with the sub-bodies)
	// TODO: perhaps use RewriteLink partial_substitute
	Handle body = variables.substitute_nocheck(get_body(pattern), subodies);
	body = RewriteLink::consume_ill_quotations(vardecl, body);
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

Handle XPatternMiner::remove_unary_and(const Handle& h)
{
	if (h->get_type() == AND_LINK and h->get_arity() == 1)
		return h->getOutgoingAtom(0);
	return h;
}

Handle XPatternMiner::alpha_conversion(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return sc->alpha_convert();
	return pattern;
}

HandleSeq XPatternMiner::gen_rand_variables(size_t n)
{
	HandleSeq variables;
	dorepeat (n)
		variables.push_back(gen_rand_variable());
	return variables;
}

Handle XPatternMiner::gen_rand_variable()
{
	return createNode(VARIABLE_NODE, randstr("$PM-"));
}

const Variables& XPatternMiner::get_variables(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return RewriteLinkCast(pattern)->get_variables();
	static Variables empty_variables;
	return empty_variables;
}

const Handle& XPatternMiner::get_vardecl(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return RewriteLinkCast(pattern)->get_vardecl();
	return Handle::UNDEFINED;
}

const Handle& XPatternMiner::get_body(const Handle& pattern)
{
	RewriteLinkPtr sc = RewriteLinkCast(pattern);
	if (sc)
		return RewriteLinkCast(pattern)->get_body();
	return pattern;
}

unsigned XPatternMiner::conjuncts(const Handle& pattern)
{
	if (pattern->get_type() == LAMBDA_LINK) {
		if (get_body(pattern)->get_type() == AND_LINK)
			return get_body(pattern)->get_arity();
		return 1;
	}
	return 0;
}

Handle XPatternMiner::remove_useless_clauses(const Handle& vardecl,
                                             const Handle& body)
{
	Handle res = Unify::remove_constant_clauses(vardecl, body);

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

std::string oc_to_string(const HandleUCounterMap& hucp)
{
	std::stringstream ss;
	ss << "size = " << hucp.size() << std::endl;
	size_t i = 0;
	for (const auto& el : hucp) {
		ss << "atom[" << i << "]:" << std::endl
		   << oc_to_string(el.first)
		   << "HandleUCounter[" << i << "]:" << std::endl
		   << oc_to_string(el.second);
		i++;
	}
	return ss.str();
}

} // namespace opencog
