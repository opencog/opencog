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

#include <opencog/util/empty_string.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/core/LambdaLink.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{

/**
 * Collection of tools to calculate pattern surprisingness.
 */

typedef std::vector<HandleSeqSeq> HandleSeqSeqSeq;
typedef Counter<HandleSeq, unsigned> HandleSeqUCounter;

class Surprisingness {
public:
	/**
	 * Calculate the I-Surprisingness as defined in
	 * https://wiki.opencog.org/w/Measuring_Surprisingness of a pattern
	 * composed of the conjunction of components over a given
	 * texts. Partitions are directly defined within that function.
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
	static double isurp_old(const Handle& pattern,
	                        const HandleSet& texts,
	                        bool normalize=true);

	/**
	 * Similar to isurp_old but takes into account joint variables.
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
	 * Then the probability estimate is p1*p2*p3, where
	 *
	 * p3 = P(Y=Z)
	 *
	 * is the probability that the value of Y is equal to the value of
	 * Z.
	 *
	 * To calculate p3 accurately one would need to produce the
	 * distribution over all values that Y can take and the
	 * distribution of over all values that Z can take, then calculate
	 * the inner product of the 2 distributions.
	 *
	 * That is probability too expensive to begin with so instead we
	 * assume all values are distributed evenly. Let's assume the same
	 * variable X appears in n difference clauses, and each clause has
	 * support S1 to Sn. Let us first estimate the number of values
	 * that X can take in the ith clause. Assume the ith clause has Ci
	 * variables, if these variables are completely independent then
	 * the number of values of each of them is the Ci-th root of its
	 * support Si
	 *
	 * Vi=root(Si, Ci)
	 *
	 * such that the final support can be obtained by multiplying the
	 * number of values of all variables.
	 *
	 * Then, without loss of generality, let's assume that V1 is the
	 * lowest number of values that X can take (thus in the first
	 * clause), then the probability that X takes the same values in
	 * all clauses is
	 *
	 * Prod_{i=2}^n 1/Vi
	 *
	 * because for each value of the first clause, the probability that
	 * the other values are equal to it is Prod_{i=2}^n 1/Vi, assuming
	 * that all values V2 to Vn contain the values in V1. In practice I
	 * have no idea how true that is, though since the pattern where
	 * surprisingness is being measured has a minimum positive support,
	 * we know all values in Vi, with i=1 to n, have at least one value
	 * in common.
	 */
	static double isurp(const Handle& pattern,
	                    const HandleSet& texts,
	                    bool normalize=true);

	/**
	 * Return (Node "*-I-SurprisingnessValueKey-*")
	 */
	static Handle isurp_key();

	/**
	 * Retrieve the I-Surprisingness value of the given pattern
	 * associated to (Node "*-I-SurprisingnessValueKey-*").
	 */
	static double get_isurp_value(const Handle& pattern);

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
	 * [[[A],[C],[B]],
	 *  [[C,A],[B]],
	 *  [[C],[B,A]],
	 *  [[A],[C,B]],
	 *  [[C,B,A]]]
	 */
	static HandleSeqSeqSeq partitions(const HandleSeq& hs);
	static HandleSeqSeqSeq partitions(HandleSeq::const_iterator from,
	                                  HandleSeq::const_iterator to);

	/**
	 * Like partitions but takes a pattern. Also the partition block
	 * corresponding to the full set has been removed (since it is
	 * already the block corresponding to the full pattern). For
	 * instance
	 *
	 * pattern = Lambda
	 *             And
	 *               A
	 *               B
	 *               C
	 *
	 * return
	 *
	 * [[[A],[C],[B]],
	 *  [[C,A],[B]],
	 *  [[C],[B,A]],
	 *  [[A],[C,B]]]
	 */
	static HandleSeqSeqSeq partitions(const Handle& pattern);

	/**
	 * Convert a partition block [A,B] into a pattern like
	 *
	 * Lambda
	 *   And
	 *     B
	 *     C
	 *
	 * and insert it in as.
	 */
	static Handle add_pattern(const HandleSeq& block, AtomSpace& as);

	/**
	 * Like add_pattern but doesn't add the pattern in any atomspace,
	 * only remains in RAM.
	 */
	static LambdaLinkPtr mk_lambda(const HandleSeq& block);
	static Handle mk_pattern(const HandleSeq& block);

	/**
	 * Turn a partition into a sequence of subpatterns. Add then in the
	 * provided atomspace to enable memoization of their supports.
	 */
	static HandleSeq add_subpatterns(const HandleSeqSeq& partition,
	                                 const Handle& pattern,
	                                 AtomSpace& as);

	/**
	 * Return the set of variables that appear in more than one block
	 *
	 * For instance
	 *
	 * pattern
	 * =
	 * Lambda
	 *   X Y Z
	 *   Inheritance X Y
	 *   Inheritance Y Z
	 *
	 * partition
	 * =
	 * { {Inheritance X Y},
	 *   {Inheritance Y Z} }
	 *
	 * returns
	 *
	 * [Y]
	 *
	 * because it appears in two blocks.
 	 */
	static HandleSeq joint_variables(const Handle& pattern,
	                                 const HandleSeqSeq& partition);

	/**
	 * Return the the number values associated to a given variable in a
	 * block (subpatterns) w.r.t. to texts database.
	 */
	static unsigned value_count(const HandleSeq& block,
	                            const Handle& var,
	                            const HandleSet& texts);

	/**
	 * Return the probability distribution over value of var in the
	 * given subpattern/block against a given database.
	 */
	static HandleCounter value_distribution(const HandleSeq& block,
	                                        const Handle& var,
	                                        const HandleSet& texts);

	/**
	 * Perform the inner product of a collection of distributions.
	 *
	 * For instance
	 *
	 * dists
	 * =
	 * { {A->0.5, B->0.5},
	 *   {B->0.4, C->0.3, D->0.3} }
	 *
	 * returns
	 *
	 * 0.5*0        // A
	 * + 0.5*0.4    // B
	 * + 0*0.3      // C
	 * + 0*0.3      // D
	 * = 0.2
	 */
	static double inner_product(const std::vector<HandleCounter>& dists);

	/**
	 * Calculate the empiric probability of a pattern according to a
	 * database texts.
	 */
	static double emp_prob(const Handle& pattern, const HandleSet& texts);

	/**
	 * Calculate probability estimate of a pattern given a partition,
	 * assuming all blocks are independent, but takes into account the
	 * joint variables.
	 *
	 * An atomspace is provided to memoize the support of subpatterns.
	 */
	static double ji_prob(const HandleSeqSeq& partition,
	                      const Handle& pattern,
	                      const HandleSet& texts);

	/**
	 * Tell whether 2 blocks/subpatterns are equivalent with respect to
	 * a given variable. Basically, whether both block are semantically
	 * equivalent and var is in the same position in both of them.
	 *
	 * For instance
	 *
	 * l_blk = { Inh X Y }
	 * r_blk = { Inh Z Y }
	 *
	 * are equivalent w.r.t Y because are both are semantically
	 * equivalent (up to an alpha-conversion) and Y is used in the same
	 * place in both blocks.
	 */
	static bool is_equivalent(const HandleSeq& l_blk,
	                          const HandleSeq& r_blk,
	                          const Handle& var);

	static HandleSeqUCounter::const_iterator find_equivalent(
		const HandleSeqUCounter& partition_c,
		const HandleSeq& block,
		const Handle& var);
	static HandleSeqUCounter::iterator find_equivalent(
		HandleSeqUCounter& partition_c,
		const HandleSeq& block,
		const Handle& var);

	/**
	 * Given subpatterns linked by a variable, count how many
	 * subpatterns are equivalent with respect to this variable.
	 *
	 * For instance given patterns
	 *
	 * A = Inh X Y
	 * B = Inh Y Z
	 * C = Inh W Y
	 *
	 * A and C are equivalent with respect to Y, because all values
	 * associated to Y in A and the same associated to Y in C, however
	 * B is independent (occupies another block) because values
	 * associated to Y in B are different than the values associated to
	 * Y in A or C.
	 *
	 * Thus for this example it would return
	 *
	 * {A:2, C:1}
	 *
	 * TODO: this should be replaced by a structure considering not
	 * only equivalence but also implication as well.
	 */
	static HandleSeqUCounter group_eq(const HandleSeqSeq& partition,
	                                  const Handle& var);

	/**
	 * For each joint variable of pattern (variable that appears in
	 * more than one partition block) calculate the probability
	 * estimate of being assigned the same value across all block.
	 */
	static double eq_prob(const HandleSeqSeq& partition,
	                      const Handle& pattern,
	                      const HandleSet& texts);
};

/**
 * Given a partition, that is a sequence of blocks, where each
 * block is a sequence of handles, return
 */
std::string oc_to_string(const HandleSeqSeqSeq& hsss,
                         const std::string& indent=empty_string);
	
} // ~namespace opencog

#endif /* OPENCOG_SURPRISINGNESS_H_ */
