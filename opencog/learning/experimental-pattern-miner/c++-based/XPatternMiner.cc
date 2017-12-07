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
#include <opencog/atomutils/Unify.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/query/BindLinkAPI.h>

namespace opencog
{

XPMParameters::XPMParameters(unsigned ms, unsigned igram,
                             const Handle& ipat, int maxd, int maxp)
	: minsup(ms), initgram(igram), initpat(ipat),
	  maxdepth(maxd), maxpats(maxp)
{
	// Provide initial pattern if none
	if (not initpat) {
		HandleSeq vars = XPatternMiner::gen_rand_variables(initgram);
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
	ScopeLinkPtr sc = ScopeLinkCast(initpat);
	if (not sc->get_vardecl()) {
		Handle vardecl = sc->get_variables().get_vardecl(),
			body = sc->get_body();
		initpat = createLink(initpat->get_type(), vardecl, body);
	}

	// Overwrite initgram if necessary
	initgram = XPatternMiner::gram(initpat);
}

XPatternMiner::XPatternMiner(AtomSpace& as, const XPMParameters& prm)
	: text_as(as), param(prm) {}

HandleTree XPatternMiner::operator()()
{
	HandleSet texts;
	text_as.get_handles_by_type(std::inserter(texts, texts.end()),
	                            opencog::ATOM, true);
	return specialize(param.initpat, texts, param.initgram, param.maxdepth);
}

HandleTree XPatternMiner::specialize(const Handle& pattern,
                                     const HandleSet& texts,
                                     unsigned mingram,
                                     int maxdepth)
{
	// If the initial pattern is specialized, this initial filtering
	// may save some computation
	HandleUCounter fltexts = filter_texts(pattern, HandleUCounter(texts));
	return specialize(pattern, fltexts, mingram, maxdepth);
}

HandleTree XPatternMiner::specialize(const Handle& pattern,
                                     const HandleUCounter& texts,
                                     unsigned mingram,
                                     int maxdepth)
{
	// We have reached the maximum depth
	if (maxdepth == 0)
		return HandleTree();

	// If the pattern is constant, then no specialization is possible
	if (pattern->get_type() != LAMBDA_LINK)
		return HandleTree();

	// Make sure the pattern has enough support
	if (not enough_support(pattern, texts))
		return HandleTree();

	// If the pattern is a variable, then differ to single variable
	// specialization
	if (get_body(pattern)->get_type() == VARIABLE_NODE)
		return specialize_varpat(pattern, texts, maxdepth);

	// Otherwise, if the pattern is structured, calculate the mappings
	// from variables to values, get the patterns of these values now
	// interpreted as texts, and use these patterns to expand the
	// initial pattern by composition.
	HandleUCounterMap var2subtexts = gen_var2subtexts(pattern, texts);

	// For each variable, create a trivial pattern, with its variable
	// as body and its type as vardecl, take the list of values, pass
	// it as texts and call specialize to generate all sub-patterns.
	HandleHandleTreeMap var2subpats;
	for (const auto& el : var2subtexts) {
		const Handle& var = el.first;
		const HandleUCounter& subtexts = el.second;
		Handle varpattern = mk_varpattern(pattern, var);
		var2subpats[var] = specialize_varpat(varpattern, subtexts, maxdepth);
	}

	// Replacing each variable by its patterns in a cartesian product
	// fashion (not forgetting updating their vardecls accordingly)
	return product_compose(pattern, texts, var2subpats, mingram, maxdepth);
}

HandleTree XPatternMiner::specialize_varpat(const Handle& varpat,
                                            const HandleUCounter& texts,
                                            int maxdepth)
{
	// We have reached the maximum depth
	if (maxdepth == 0)
		return HandleTree();

	OC_ASSERT(get_body(varpat)->get_type() == VARIABLE_NODE);

	HandleUCounterMap pat2texts = shallow_patterns(texts);
	HandleTree patterns;
	HandleTree::iterator pat_it = patterns.begin();
	for (const auto& el : pat2texts) {
		const Handle& shapat = el.first;
		const HandleUCounter& shapat_texts = el.second;

		// Make sure the shallow pattern has enough support before
		// including it in the solution set and specializing it.
		// Because we know that shapat is 1-gram and its text has
		// already been filtered by shallow_patterns, we only need to
		// look at the texts.
		if (not enough_support(shapat_texts))
			continue;

		// Add shallow pattern to solution set, and specialize it
		pat_it = patterns.insert(pat_it, shapat);
		HandleTree specs = specialize(shapat, shapat_texts, 0, maxdepth - 1);
		patterns.append_children(pat_it, specs.begin(), specs.end());
	}
	return patterns;
}

Handle XPatternMiner::mk_varpattern(const Handle& pattern,
                                    const Handle& var) const
{
	Handle subvardecl = filter_vardecl(get_vardecl(pattern), var);
	return Handle(createLambdaLink(subvardecl, var));
}

bool XPatternMiner::enough_support(const Handle& pattern,
                                   const HandleUCounter& texts) const
{
	return param.minsup <= freq(pattern, texts);
}

bool XPatternMiner::enough_support(const HandleUCounter& texts) const
{
	return param.minsup <= texts.total_count();
}

unsigned XPatternMiner::freq(const Handle& pattern,
                             const HandleUCounter& texts) const
{
	if (totally_abstract(pattern))
		return gram(pattern) * freq(texts);

	// If the pattern has more than one gram, then it is assumed that
	// the count of each text in texts is 1, thus we just need to run
	// the pattern over the whole texts without worrying about counts
	if (1 < gram(pattern))
		return restrict_satisfying_set(pattern, texts)->get_arity();

	// Otherwise, assuming the texts has already been filtered, it is
	// merely the texts total count. TODO: it would safer to put some
	// assert to check if that assumption really hold
	return freq(texts);
}

unsigned XPatternMiner::freq(const HandleUCounter& texts) const
{
	return texts.total_count();
}

HandleUCounter XPatternMiner::filter_texts(const Handle& pattern,
                                           const HandleUCounter& texts) const
{
	// No need to filter if most abstract
	if (totally_abstract(pattern))
		return texts;

	// If it has more grams than 1, then TODO: it's probably the union
	// of the texts of all its gram components.
	if (1 < gram(pattern))
		return texts;

	// Otherwise it is a 1-gram pattern
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
	// If I use a temporary atomspace on stack, then the atoms in it
	// get deleted, grrrr, would need to use a smart pointer.
	tmp_as.clear();
	tmp_as.add_atom(text);
	AtomSpace tmp_pattern_as(&tmp_as);
	Handle tmp_pattern = tmp_pattern_as.add_atom(pattern),
		vardecl = get_vardecl(tmp_pattern),
		body = get_body(tmp_pattern),
		gl = tmp_as.add_link(GET_LINK, vardecl, body),
		result = satisfying_set(&tmp_as, gl);
	return result;
}

Handle XPatternMiner::restrict_satisfying_set(const Handle& pattern,
                                              const HandleUCounter& texts) const
{
	AtomSpace tmp_text_as;
	for (const auto& text : texts)
		tmp_text_as.add_atom(text.first);

	AtomSpace tmp_query_as(&tmp_text_as);
	Handle tmp_pattern = tmp_query_as.add_atom(pattern),
		vardecl = get_vardecl(tmp_pattern),
		body = get_body(tmp_pattern),
		gl = tmp_query_as.add_link(GET_LINK, vardecl, body),
		results = satisfying_set(&tmp_text_as, gl);
	return results;
}

HandleUCounterMap XPatternMiner::shallow_patterns(const HandleUCounter& texts)
{
	HandleUCounterMap pat2texts;
	for (const auto& text : texts)
		pat2texts[shallow_pattern(text.first)].insert(text);

	return pat2texts;
}

Handle XPatternMiner::shallow_pattern(const Handle& text)
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

HandleUCounterMap XPatternMiner::gen_var2subtexts(const Handle& pattern,
                                                  const HandleUCounter& texts) const
{
	const Variables& vars = get_variables(pattern);

	// If we have more than 1-gram patterns, then hopefully each text
	// of texts has a count of one (TODO: make sure that's true). In
	// the such case we just run the pattern matcher and collect the
	// results without worrying about counts.
	if (1 < gram(pattern)) {
		Handle satset = restrict_satisfying_set(pattern, texts);
		return gen_var2vals(vars, satset->getOutgoingSet());
	}

	// Otherwise take counts into account
	HandleSeq values_seq;
	for (const auto& text : texts) {
		const HandleSeq& res =
			matched_results(pattern, text.first)->getOutgoingSet();
		values_seq.insert(values_seq.end(), res.begin(), res.end());
	}
	return gen_var2vals(vars, values_seq);
}

HandleUCounterMap XPatternMiner::gen_var2vals(const Variables& vars,
                                              const HandleSeq& values_seq) const
{
	HandleUCounterMap var2vals;
	for (const Handle& values : values_seq) {
		for (const auto& v : gen_var2val(vars, values)) {
			const Handle& var = v.first;
			const Handle& val = v.second;
			var2vals[var][val] += 1;
		}
	}
	return var2vals;
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

HandleTree XPatternMiner::gen_var_overlap_subpatterns(const Handle& pattern,
                                                      const Handle& variable,
                                                      const Handle& subpat,
                                                      int maxdepth) const
{
	ScopeLinkPtr sc_subpat = ScopeLinkCast(subpat);
	if (not sc_subpat)
		return HandleTree(subpat);

	// Define vs1 and vs2 to generate all variable overlaps
	const HandleSet& vs1 = get_variables(subpat).varset;
	HandleSet vs2 = get_variables(pattern).varset;
	vs2.erase(variable);

	// Make sure vs1 and vs2 don't intersect
	OC_ASSERT(is_disjoint(vs1, vs2));

	// Generate all overlaps
	HandleMapTree var_overlaps = gen_var_overlaps(vs1, vs2, maxdepth);

	// For each mapping generate an associated subpattern
	return gen_var_overlap_subpatterns(sc_subpat, var_overlaps);
}

HandleTree XPatternMiner::gen_var_overlap_subpatterns(ScopeLinkPtr sc_subpat,
                                                      const HandleMapTree&
                                                      var_overlaps) const
{
	HandleMapTree::iterator head_it = var_overlaps.begin();
	if (var_overlaps.is_valid(head_it))
		return gen_var_overlap_subpatterns(sc_subpat, head_it);
	return HandleTree();
}

HandleTree XPatternMiner::gen_var_overlap_subpatterns(ScopeLinkPtr sc_subpat,
                                                      HandleMapTree::sibling_iterator
                                                      sib) const
{
	HandleTree ht(sc_subpat->partial_substitute(*sib));
	for (auto ch_sib = sib.begin(); ch_sib != sib.end(); ++ch_sib) {
		HandleTree cht = gen_var_overlap_subpatterns(sc_subpat, ch_sib);
		ht.append_child(ht.begin(), cht.begin());
	}
	return ht;
}

HandleMapTree XPatternMiner::gen_var_overlaps(const HandleSet& vs1,
                                              const HandleSet& vs2,
                                              int maxdepth) const
{
	// Base cases
	if (maxdepth == 0 or vs1.empty())
		return HandleMapTree(HandleMap());

	// Recursive case
	Handle vs1_head = *vs1.begin();
	HandleSet vs1_tail = vs1;
	vs1_tail.erase(vs1_head);
	HandleMapTree old_vms = gen_var_overlaps(vs1_tail, vs2, maxdepth - 1);
	HandleMapTree new_vms(old_vms);
	for (const HandleMap& vm : old_vms) {
		for (const Handle& v2 : vs2) {
			HandleMap new_vm(vm);
			new_vm[vs1_head] = v2;
			HandleMapTree::iterator it =
				std::find(new_vms.begin(), new_vms.end(), vm);
			new_vms.append_child(it, new_vm);
		}
	}
	return new_vms;
}

HandleTree XPatternMiner::product_compose(const Handle& pattern,
                                          const HandleUCounter& texts,
                                          const HandleHandleTreeMap& var2pats,
                                          unsigned mingram,
                                          int maxdepth) const
{
	if (maxdepth == 0)
		return HandleTree();

	HandleTree patterns;
	auto map_it = var2pats.begin();
	if (map_it != var2pats.end()) {
		const Handle& variable = map_it->first;
		const HandleTree& subpatterns = map_it->second;

		// Reconstruct var2pats without that variable, and recursively
		// call product_compose to obtain patterns without the
		// subpatterns of that variables.
		HandleHandleTreeMap revar2pats(std::next(map_it), var2pats.end());
		patterns = product_compose(pattern, texts, revar2pats, mingram, maxdepth);

		// Combine the patterns of that variables with the patterns of
		// the remaining variables.
		for (auto subpat_it = subpatterns.begin();
		     subpat_it != subpatterns.end(); ++subpat_it) {
			// Calculate the depth of subpat, +1 because the root is
			// one step deeper from pattern.
			int subpatdepth = subpatterns.depth(subpat_it) + 1;

			// Make sure subpatdepth is less than or equal to
			// maxdepth.
			if (0 <= maxdepth and maxdepth < subpatdepth) {
				// For sure all children will have a greater depth so
				// they can be skipped.
				subpat_it.skip_children();
				continue;
			}

			// Alpha convert subpat_it to make sure it doesn't share
			// variables with pattern
			Handle subpat = alpha_conversion(*subpat_it);

			// Perform a single substitution of variable by subpat
			HandleTree vosubpats =
				gen_var_overlap_subpatterns(pattern, variable, subpat,
				                            maxdepth - subpatdepth);
			for (auto vosubpat_it = vosubpats.begin();
			     vosubpat_it != vosubpats.end(); ++vosubpat_it) {
				// Calculate the depth of the produced subpattern with
				// variable overlap
				int vosubpatdepth = subpatdepth + vosubpats.depth(vosubpat_it);

				// Make sure vosubpatdepth is less than or equal to
				// maxdepth.
				if (0 <= maxdepth and maxdepth < vosubpatdepth) {
					// For sure all children will have a greater depth so
					// they can be skipped.
					vosubpat_it.skip_children();
					continue;
				}

				Handle npat = compose(pattern, {{variable, *vosubpat_it}});
				// TODO: instead of filtering, it would probably be
				// better if the satisfying text were passed along the
				// pattern, in specialize and specialize_varpat, just
				// as shallow_patterns does, that way we'll already
				// have the filtered texts around.
				HandleUCounter fltexts = filter_texts(npat, texts);
				if (enough_support(npat, fltexts) and mingram <= gram(npat)) {
					// TODO: do not insert redundant patterns
					// TODO: maybe use patterns.insert()
					// TODO: fix that taking into account subpatterns
					// tree structure
					auto npat_it = patterns.empty() ? patterns.set_head(npat) :
						patterns.insert_after(patterns.begin(), npat);

					// Recursively call product_compose on npat and
					// the remaining variables
					HandleTree prod =
						product_compose(npat, fltexts, revar2pats,
						                mingram, maxdepth - vosubpatdepth);
					// These new patterns are necessarily
					// specializations of npat, which is why they are
					// inserted as children.
					patterns.append_children(npat_it, prod.begin(), prod.end());
				} else {
					// npat doesn't have enough support or gram, we
					// can skip the children of vosubpat_it, as they
					// too won't have enough support or gram.
					vosubpat_it.skip_children();

					// If vosubpat_it happens to be the root, then we
					// can also skip all children of subpat_it, as
					// they too won't generate overlapping
					// substitutions that have enough support or gram.
					if (vosubpats.depth(vosubpat_it) == 0)
						subpat_it.skip_children();
				}
			}
		}
	}

	return patterns;
}

// TODO: take care of removing local quote in the composed
// sub-patterns, if it doesn't already
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
	// TODO: perhaps use ScopeLink partial_substitute
	Handle body = variables.substitute_nocheck(get_body(pattern), subodies);
	body = ScopeLink::consume_ill_quotations(vardecl, body);
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
	ScopeLinkPtr sc = ScopeLinkCast(pattern);
	if (sc)
		return sc->alpha_conversion();
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
	ScopeLinkPtr sc = ScopeLinkCast(pattern);
	if (sc)
		return ScopeLinkCast(pattern)->get_variables();
	static Variables empty_variables;
	return empty_variables;
}

const Handle& XPatternMiner::get_vardecl(const Handle& pattern)
{
	ScopeLinkPtr sc = ScopeLinkCast(pattern);
	if (sc)
		return ScopeLinkCast(pattern)->get_vardecl();
	return Handle::UNDEFINED;
}

const Handle& XPatternMiner::get_body(const Handle& pattern)
{
	ScopeLinkPtr sc = ScopeLinkCast(pattern);
	if (sc)
		return ScopeLinkCast(pattern)->get_body();
	return pattern;
}

unsigned XPatternMiner::gram(const Handle& pattern)
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
	Handle res;
	res = Unify::remove_constant_clauses(vardecl, body);

	// Check that each clause isn't a subtree another another clause
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
