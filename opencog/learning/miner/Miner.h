/*
 * Miner.h
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
#ifndef OPENCOG_MINER_H_
#define OPENCOG_MINER_H_

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/core/Variables.h>
#include <opencog/atoms/core/RewriteLink.h>
#include <opencog/atomspace/AtomSpace.h>

#include "HandleTree.h"
#include "Valuations.h"
#include "MinerUtils.h"

class MinerUTest;

namespace opencog
{

/**
 * Parameters for Miner. The terminology is taken from
 * Frequent Subtree Mining -- An Overview, from Yun Chi et al, when
 * possible.
 */
struct MinerParameters {

	/**
	 * CTor. Note that conjuncts will be overwritten by initpat, if
	 * provided.
	 */
	MinerParameters(unsigned minsup=1,
	                unsigned conjuncts=1,
	                const Handle& initpat=Handle::UNDEFINED,
	                int maxdepth=-1,
	                double info=1.0);

	// TODO: change frequency by support!!!
	// Minimum support. Mined patterns must have a frequency equal or
	// above this value.
	unsigned minsup;

	// Initial number of conjuncts. This value is overwritten by the
	// actual number of conjuncts of initpat, if provided.
	unsigned initconjuncts;

	// Initial pattern. All found patterns are specialization of
	// it. If UNDEFINED, then the initial pattern is the most abstract
	// one. i.e.
	//
	// Lambda
	//   X
	//   X
	//
	// That is the pattern matching the entire atomspace.
	Handle initpat;

	// Maximum depth from pattern to output. If negative, then no
	// depth limit. Depth is the number of specializations between the
	// initial pattern and the produced patterns.
	int maxdepth;

	// Modify how the frequency of strongly connected components is
	// calculated from the frequencies of its components. Specifically
	// it will go from f1*...*fn to min(f1,...,fn), where f1 to fn are
	// the frequencies of each component. This allows to dismiss
	// abstractions that are likely not to lead to specializations
	// with enough support.
	//
	// If the parameter equals to 0 the frequency of the whole pattern
	// is calculated as the product of f1 to fn, if it equals to 1 the
	// frequency of the whole pattern is calculated as the min of f1
	// to fn. And if the value is between, it is calculated as a
	// linear combination of both.
	//
	// It is called info for mutual information or its n-ary
	// generalizations (like interaction information). What it means
	// is that when info is low subsequent specializations are likely
	// independant, while when info is high subsequent specializations
	// are likely dependant and thus we can afford to estimate the
	// frequency of the specializations by the lowest frequency of its
	// component.
	double info;
};

/**
 * Experimental pattern miner. Mined patterns should be compatible
 * with the pattern matcher, that is if feed to the pattern matcher,
 * the latter should return as many candidates as the pattern's
 * frequency.
 */
class Miner
{
    friend class ::MinerUTest;
public:

	/**
	 * CTor
	 */
	Miner(const MinerParameters& param=MinerParameters());

	/**
	 * Mine the given AtomSpace and return a tree of patterns linked by
	 * specialization relationship (children are specializations of
	 * parent) with frequency equal to or above minsup, starting from
	 * the initial pattern, excluded.
	 */
	HandleTree operator()(const AtomSpace& texts_as);

	/**
	 * Like above but only mine amongst the provided text collection.
	 */
	HandleTree operator()(const HandleSeq& texts);

	/**
	 * Specialization. Given a pattern and a collection to text atoms,
	 * generate all specialized patterns of the given pattern.
	 */
	HandleTree specialize(const Handle& pattern,
	                      const HandleSeq& texts,
	                      int maxdepth=-1);

	/**
	 * Like above, where all valid texts have been converted into
	 * valuations.
	 */
	HandleTree specialize(const Handle& pattern,
	                      const HandleSeq& texts,
	                      const Valuations& valuations,
	                      int maxdepth);

	/**
	 * Alternate specialization that reflects how the URE would work.
	 */
	HandleTree specialize_alt(const Handle& pattern,
	                          const HandleSeq& texts,
	                          const Valuations& valuations,
	                          int maxdepth);

	// Parameters
	MinerParameters param;

private:

	mutable AtomSpace tmp_as;

	/**
	 * Return true iff maxdepth is null or pattern is not a lambda or
	 * doesn't have enough support. Additionally the second one check
	 * whether the valuation has any variable left to specialize from.
	 */
	bool terminate(const Handle& pattern,
	               const HandleSeq& texts,
	               const Valuations& valuations,
	               int maxdepth) const;

	/**
	 * Specialize the given pattern according to shallow abstractions
	 * obtained by looking at the valuations of the front variable of
	 * valuations, then recursively call Miner::specialize on these
	 * obtained specializations.
	 */
	HandleTree specialize_shabs(const Handle& pattern,
	                            const HandleSeq& texts,
	                            const Valuations& valuations,
	                            int maxdepth);

	/**
	 * Specialize the given pattern with the given shallow abstraction
	 * at the given variable, then call Miner::specialize on the
	 * obtained specialization.
	 */
	HandleTree specialize_shapat(const Handle& pattern,
	                             const HandleSeq& texts,
	                             const Handle& var,
	                             const Handle& shapat,
	                             int maxdepth);

	/**
	 * Calculate if the pattern has enough support w.r.t. to the given
	 * texts, that is whether its frequency is greater than or equal
	 * to minsup.
	 */
	bool enough_support(const Handle& pattern,
	                    const HandleSeq& texts) const;

	/**
	 * Given a pattern and a text corpus, calculate the pattern
	 * frequency, that is the number of matches if pattern is strongly
	 * connected.
	 *
	 * If pattern is not strongly connected AND some heuristic is in
	 * place TODO, then the definition of frequency deviates from the
	 * usual one and corresponds to the minimum frequency over all
	 * strongly connected components of that pattern.
	 *
	 * ms is used to halt the frequency calculation if it reaches a
	 * certain maximum, for saving resources.
	 */
	unsigned freq(const Handle& pattern,
	              const HandleSeq& texts,
	              unsigned ms) const;

	/**
	 * Calculate the frequency of the whole pattern, given the
	 * frequency of it's components.
	 */
	unsigned freq(const std::vector<unsigned>& freqs) const;

	/**
	 * Filter in only texts matching the pattern
	 */
	HandleSeq filter_texts(const Handle& pattern,
	                       const HandleSeq& texts) const;

	/**
	 * Check whether a pattern matches a text.
	 */
	bool match(const Handle& pattern, const Handle& text) const;

	/**
	 * Like above but returns the Set of Lists of values associated to
	 * the variables of the pattern. Assumes that pattern is always a
	 * LambdaLink, and not a constant.
	 *
	 * TODO: optimize
	 */
	Handle matched_results(const Handle& pattern, const Handle& text) const;
};

} // ~namespace opencog

#endif /* OPENCOG_MINER_H_ */
