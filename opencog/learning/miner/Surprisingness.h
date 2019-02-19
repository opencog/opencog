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

#include <opencog/atoms/base/Handle.h>
#include <opencog/util/empty_string.h>

namespace opencog
{

/**
 * Collection of tools to calculate pattern surprisingness.
 */

typedef std::vector<HandleSeqSeq> HandleSeqSeqSeq;

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
	static double ISurprisingness_old(const Handle& pattern,
	                                  const HandleSeqSeq& partitions,
	                                  const HandleSet& texts,
	                                  bool normalize=true);

	/**
	 * Like above but partitions are represented by List/Set links of
	 * List/Set links instead of C++ HandleSetSet.
	 */
	static double ISurprisingness_old(const Handle& pattern,
	                                  const Handle& partitions,
	                                  const HandleSet& texts,
	                                  bool normalize=true);

	/**
	 * Similar to ISurprisingness_old but takes into account variable
	 * linkage.
	 *
	 * For instance the probability estimate of
	 *
	 * Lambda
	 *   X Y Z
	 *   And
	 *     Inheritance X Y
	 *     Inheritance Z C)
	 *
	 * is the product of probability p1 of
	 *
	 * Lambda
	 *   X Y
	 *   Inheritance X Y
	 *
	 * with probability p2 of
	 *
	 * Lambda
	 *   Z
	 *   Inheritance Z C
	 *
	 * This works fine because the two conjuncts are independent,
	 * however, the probability estimate of
	 *
	 * Lambda
	 *   X Y
	 *   And
	 *     Inheritance X Y
	 *     Inheritance Y C
	 *
	 * isn't merely p1*p2 because they have one variable in common Y.
	 *
	 * To address that we use the fact that the above pattern is
	 * equivalent to
	 *
	 * Lambda
	 *   X Y Z
	 *   And
	 *     Inheritance X Y
	 *     Inheritance Z C
	 *     Equal Y Z
	 *
	 * Then the probability estimate is p1*p2*p3, where p3 is the
	 * probability that the value of Y is equal to the value of Z.
	 *
	 * To calculate p3 accurately one would need to produce the
	 * distribution over all values that Y can take and the
	 * distribution of over all values that Z can take, then calculate
	 * the inner product of the 2 distributions. If that is too
	 * expensive, an option would be to assume a uniform distribution
	 * over all possible values of Y and Z. Getting the exact number of
	 * possible values can be costly so instead one may estimate it
	 * with the pattern support, assuming that all variables within a
	 * conjunct are independent. So for instance in conjunct
	 *
	 * Lambda
	 *   X Y
	 *   Inheritance X Y
	 *
	 * with support S, then assuming complete independence between X
	 * and Y an estimate of the number of values of Y would be the
	 * square root of S. If the pattern has 3 variables, then it would
	 * be the cube root, and so on.
	 *
	 * In the end the formula to estimate p3 is
	 *
	 * 1/nth_root(S1, C1) * 1/nth_root(S2, C2)
	 * = pow(S1, -1/C1) * pow(S2, -1/C2)
	 *
	 * where S1 and S2 are respectively the supports of the first
	 * pattern and second pattern, and C1 and C2 are respectively the
	 * number of variables of the first and second pattern.
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

	/**
	 * Given a handle h and a sequence of sequences of handles, insert
	 * h in front of each subsequence, duplicating each sequence with
	 * its augmented subsequence. For instance
	 *
	 * h = D
	 * hss = [[A],[B,C]]
	 *
	 * returns
	 *
	 * [[[D,A],[B,C]],[[A],[D,B,C]],[[A],[B,C],[D]]]
	 */
	static HandleSeqSeqSeq combinatorial_insert(const Handle& h,
	                                            const HandleSeqSeq& hss);
	static HandleSeqSeqSeq combinatorial_insert(const Handle& h,
	                                            HandleSeqSeq::const_iterator from,
	                                            HandleSeqSeq::const_iterator to);

	/**
	 * Given a HandleSeq hs, produce all partitions of hs. For instance
	 * if hs is the following
	 *
	 * c = [A,B,C]
	 *
	 * return
	 *
	 * [[[A,B,C]],[[B,C],[A]],[[A,C],[B]],[[C],[A,B]],[[C],[B],[A]]]
	 */
	static HandleSeqSeqSeq partitions(const HandleSeq& hs);
	static HandleSeqSeqSeq partitions(HandleSeq::const_iterator from,
	                                  HandleSeq::const_iterator to);
};

std::string oc_to_string(const HandleSeqSeqSeq& hsss,
                         const std::string& indent=empty_string);
	
} // ~namespace opencog

#endif /* OPENCOG_SURPRISINGNESS_H_ */
