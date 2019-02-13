/*
 * Surprisingness.h
 *
 * Copyright (C) 2019 SingularityNET Foundation
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
#ifndef OPENCOG_SURPRISINGNESS_H_
#define OPENCOG_SURPRISINGNESS_H_

#include "opencog/atoms/base/Handle.h"

namespace opencog
{

/**
 * Collection of tools to calculate pattern surprisingness.
 */

class Surprisingness {
public:
	/**
	 * Calculate the I-Surprisingness as defined in
	 * https://wiki.opencog.org/w/Measuring_Surprisingness of a pattern
	 * composed of the conjunction of components over a given texts. The
	 * argument `partitions` represent all possible ways to partitions the
	 * components to calculate an estimate of the pattern.
	 *
	 * For instance, given
	 *
	 * pattern = (Lambda
	 *             X
	 *             (And
	 *               (Inheritance X (Concept "soda-drinker"))
	 *               (Inheritance X (Concept "male"))))
	 *
	 * partitions = { { (Lambda X (Inheritance X (Concept "soda-drinker"))),
	 *                  (Lambda X (Inheritance X (Concept "male"))) } }
	 *
	 * here since there are only 2 components, there is only one way to
	 * partition it.
	 *
	 * return calculate its I-surprisingness. The components are passed
	 * in order to take into account their support (which should
	 * already be stored in them), and texts in passed in order to
	 * obtain the universe count.
	 *
	 * Normalize determines whether the surprisingness is normalized
	 * according to the pattern frequency.
	 *
	 * Although mathmetically speaking partitions are sets of sets,
	 * they are encoded as lists of lists for performance reasons.
	 */
	static double ISurprisingness(const Handle& pattern,
	                              const HandleSeqSeq& partitions,
	                              const HandleSet& texts,
	                              bool normalize=true);

	/**
	 * Like above but partitions are represented by List/Set links of
	 * List/Set links instead of C++ HandleSetSet.
	 */
	static double ISurprisingness(const Handle& pattern,
	                              const Handle& partitions,
	                              const HandleSet& texts,
	                              bool normalize=true);

	/**
	 * Like above but contain verbatim port of Shujing's code.
	 */
	static double ISurprisingness_old(const Handle& pattern,
	                                  const HandleSeqSeq& partitions,
	                                  const HandleSet& texts,
	                                  bool normalize=true);
	static double ISurprisingness_old(const Handle& pattern,
	                                  const Handle& partitions,
	                                  const HandleSet& texts,
	                                  bool normalize=true);


	/**
	 * Return (Node "*-I-SurprisingnessValueKey-*")
	 */
	static Handle ISurprisingness_key();

	/**
	 * Retrieve the I-Surprisingness value of the given pattern
	 * associated to (Node "*-I-SurprisingnessValueKey-*").
	 */
	static double get_ISurprisingness(const Handle& pattern);

	/**
	 * Return the distance between a value and an interval
	 *
	 * That is if the value, v, is higher than the upper bound, u, then it
	 * returns the distance between u and v. If v is than the lower bound
	 * l, then it returns the distance between l and v. Otherwise it
	 * returns 0.
	 */
	static double dst_from_interval(double l, double u, double v);
};

} // ~namespace opencog

#endif /* OPENCOG_SURPRISINGNESS_H_ */
