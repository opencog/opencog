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
                                 const Handle& ipat, int maxd)
	: minsup(ms), initconjuncts(iconjuncts), initpat(ipat),
	  maxdepth(maxd)
{
	// Provide initial pattern if none
	if (not initpat) {
		HandleSeq vars = MinerUtils::gen_rand_variables(initconjuncts);
		Handle vardecl = MinerUtils::variable_list(vars);
		Handle body = MinerUtils::mk_body(vars);
		initpat = MinerUtils::lambda(vardecl, body);
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
	initconjuncts = MinerUtils::n_conjuncts(initpat);
}

Miner::Miner(const MinerParameters& prm)
	: param(prm) {}

HandleTree Miner::operator()(const AtomSpace& texts_as)
{
	HandleSeq texts;
	texts_as.get_handles_by_type(std::inserter(texts, texts.end()),
	                             opencog::ATOM, true);
	return operator()(texts);
}

HandleTree Miner::operator()(const HandleSeq& texts)
{
	return specialize(param.initpat, texts, param.maxdepth);
}

HandleTree Miner::specialize(const Handle& pattern,
                             const HandleSeq& texts,
                             int maxdepth)
{
	// TODO: decide what to choose and remove or comment
	// return specialize_alt(pattern, texts, Valuations(pattern, texts), maxdepth);
	return specialize(pattern, texts, Valuations(pattern, texts), maxdepth);
}

HandleTree Miner::specialize(const Handle& pattern,
                             const HandleSeq& texts,
                             const Valuations& valuations,
                             int maxdepth)
{
	// One of the termination criteria has been reached
	if (terminate(pattern, texts, valuations, maxdepth))
		return HandleTree();

	// Produce specializations from other variables than the front
	// one.
	valuations.inc_focus_variable();
	HandleTree patterns = specialize(pattern, texts, valuations, maxdepth);
	valuations.dec_focus_variable();

	// Produce specializations from shallow abstractions on the front
	// variable, and so recusively.
	HandleTree shabs_pats = specialize_shabs(pattern, texts, valuations,
	                                         maxdepth);

	// Merge specializations to patterns while discarding duplicates
	patterns = merge_patterns({patterns, shabs_pats});

	return patterns;
}

HandleTree Miner::specialize_alt(const Handle& pattern,
                                 const HandleSeq& texts,
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
                      const HandleSeq& texts,
                      const Valuations& valuations,
                      int maxdepth) const
{
	return
		// We have reached the maximum depth
		maxdepth == 0 or
		// The pattern is constant, no specialization is possible
		pattern->get_type() != LAMBDA_LINK or
		// There is no more variable to specialize from
		valuations.no_focus() or
		// The pattern doesn't have enough support
		not MinerUtils::enough_support(pattern, texts, param.minsup);
}

HandleTree Miner::specialize_shabs(const Handle& pattern,
                                   const HandleSeq& texts,
                                   const Valuations& valuations,
                                   int maxdepth)
{
	// Generate shallow patterns of the first variable of the
	// valuations and associate the remaining valuations (excluding
	// that variable) to them.
	HandleSet shapats = MinerUtils::focus_shallow_abstract(valuations, param.minsup);

	// No shallow abstraction to use for specialization
	if (shapats.empty())
		return HandleTree();

	// For each shallow abstraction, create a specialization from
	// pattern by composing it, and recursively specialize the result
	// with the new resulting valuations.
	HandleTree patterns;
	Handle var = valuations.focus_variable();
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
                                    const HandleSeq& texts,
                                    const Handle& var,
                                    const Handle& shapat,
                                    int maxdepth)
{
	// Perform the composition (that is specialize)
	Handle npat = MinerUtils::compose(pattern, {{var, shapat}});

	// If the specialization has too few conjuncts, dismiss it.
	if (MinerUtils::n_conjuncts(npat) < param.initconjuncts)
		return HandleTree();

	// That specialization doesn't have enough support, skip it
	// and its specializations.
	if (not MinerUtils::enough_support(npat, texts, param.minsup))
		return HandleTree();

	// Specialize npat from all variables (with new valuations)
	HandleTree nvapats = specialize(npat, texts, maxdepth - 1);

	// Return npat and its children
	return HandleTree(npat, {nvapats});
}

} // namespace opencog
