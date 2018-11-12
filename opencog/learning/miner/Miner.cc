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

#include <opencog/atoms/execution/Instantiator.h>
#include <opencog/atoms/core/LambdaLink.h>

#include <opencog/util/Logger.h>
#include <opencog/util/algorithm.h>

#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/algorithm/transform.hpp>

#include <functional>

namespace opencog
{

// TODO:
// 7. make sure that filtering is still meaningfull

MinerParameters::MinerParameters(unsigned ms, unsigned iconjuncts,
                                 const Handle& ipat, int maxd,
                                 double io)
	: minsup(ms), initconjuncts(iconjuncts), initpat(ipat),
	  maxdepth(maxd), info(io)
{
	// Provide initial pattern if none
	if (not initpat) {
		HandleSeq vars = MinerUtils::gen_rand_variables(initconjuncts);
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
	initconjuncts = MinerUtils::conjuncts(initpat);
}

Miner::Miner(const MinerParameters& prm)
	: param(prm) {}

HandleTree Miner::operator()(const AtomSpace& texts_as)
{
	HandleSet texts;
	texts_as.get_handles_by_type(std::inserter(texts, texts.end()),
	                             opencog::ATOM, true);
	return operator()(texts);
}

HandleTree Miner::operator()(const HandleSet& texts)
{
	// If the initial pattern is specialized, this initial filtering
	// may save some computation
	HandleSet fltexts = filter_texts(param.initpat, texts);
	return specialize(param.initpat, fltexts, param.maxdepth);
}

HandleTree Miner::specialize(const Handle& pattern,
                             const HandleSet& texts,
                             int maxdepth)
{
	// TODO: decide what to choose and remove or comment
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
	Variables vars = MinerUtils::get_variables(pattern);

	// Calculate all shallow abstractions of pattern
	HandleSetSeq shabs = MinerUtils::shallow_abstract(valuations, param.minsup);

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

HandleTree Miner::specialize_shabs(const Handle& pattern,
                                   const HandleSet& texts,
                                   const Valuations& valuations,
                                   int maxdepth)
{
	// Generate shallow patterns of the first variable of the
	// valuations and associate the remaining valuations (excluding
	// that variable) to them.
	HandleSet shapats = MinerUtils::front_shallow_abstract(valuations, param.minsup);

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
	Handle npat = MinerUtils::compose(pattern, {{var, shapat}});

	// If the specialization has too few conjuncts, dismiss it.
	if (MinerUtils::conjuncts(npat) < param.initconjuncts)
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

bool Miner::enough_support(const Handle& pattern,
                           const HandleSet& texts) const
{
	return param.minsup <= freq(pattern, texts, param.minsup);
}

unsigned Miner::freq(const Handle& pattern,
                     const HandleSet& texts,
                     unsigned ms) const
{
	HandleSeq cps(MinerUtils::get_component_patterns(pattern));

	// Likely a constant pattern
	if (cps.empty())
	    return 1;

	// Otherwise aggregate the frequencies in a heuristic fashion
	std::vector<unsigned> freqs;
	boost::transform(cps, std::back_inserter(freqs),
	                 [&](const Handle& cp)
	                 { return MinerUtils::component_support(cp, texts, ms); });
	return freq(freqs);
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
	if (MinerUtils::totally_abstract(pattern))
		return texts;

	// If it has more than one conjunct, then TODO: it's probably the
	// union of the texts of all its conjuncts.
	if (1 < MinerUtils::conjuncts(pattern))
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

	// Ignore text of these types
	Type tt = text->get_type();
	if (tt == DEFINE_LINK or
	    tt == DEFINED_SCHEMA_NODE)
		return false;

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
		tmp_text = tmp_as.add_atom(MinerUtils::quote(text)),
		ml = tmp_as.add_link(MAP_LINK, tmp_pattern, tmp_text);
	Instantiator inst(&tmp_as);
	return HandleCast(inst.execute(ml));
}

} // namespace opencog
