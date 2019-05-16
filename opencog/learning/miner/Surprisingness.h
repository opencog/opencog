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
#include <opencog/unify/Unify.h>

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
	                        const HandleSeq& texts,
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
	 *     Inheritance Z C
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
	 * The problem is that, first, calculating such probability
	 * distribution is expensive, and second, the resulting estimate is
	 * too accurate and thus most pattern are measured as unsurprising
	 * due to the inner product capturing the interactions at the point
	 * of contact of the variable, between the components.  Instead an
	 * estimate relying on the counts alone of values associated to
	 * given variables in given components is derived.
	 *
	 * Let's assume the same variable X appears in n difference
	 * components. Let's call these variable appearences X1 to Xn. So
	 * the goal is to estimate P(X1=...=Xi= ...=Xn).  Let's denote
	 * V(Xi) the set of values that Xi can take when its corresponding
	 * component is matched against the database/texts alone, without
	 * any interaction of the other components. Thus |V(Xi)| is the
	 * number values that Xi takes in that standalone component.
	 *
	 * Let's now take into account the syntactic specialization
	 * relationships between each component relative to a given
	 * variable. Formally component A is syntactic specialization of
	 * component B relative to variable X, if B is a subtree of A where
	 * all variables but X have been stripped out. Conversely we say
	 * that B is a syntactic abstraction of A relative to variable
	 * X. Obviously here the variables of interest will be joint
	 * variables between A and B. The idea of establishing a
	 * variable-relative-specialization between components is to
	 * guaranty that the number of possible values that can be chosen
	 * so that the two variable occurences equate is bounded by the
	 * number of values of the variable occurence of the more abstract
	 * component. If no such relationship exists, then the number of
	 * possible values is bounded by the size of the database/texts,
	 * which is usually higher than the actually value, and thus often
	 * a poor basis for an estimate. By performing a purely syntactic
	 * analysis the estimate can be greatly enhenced. Since the
	 * analysis is purely syntactic it does not diminish the measure of
	 * surprisingness of the pattern relative to the database. I.e. it
	 * adequatly discounts in the surprisingness measure the
	 * surprisingness of the pattern alone.
	 *
	 * ## Examples
	 *
	 * ### First Example
	 *
	 * pattern
	 * =
	 * Lambda
	 *   X Y
	 *   And
	 *     Inheritance X Y
	 *     Inheritance F Y
	 *     Inheritance G X
	 *
	 * Let's consider a partition of 3 components/blocks, each clause
	 * is a block.
	 *
	 * A = Inheritance X Y
	 * B = Inheritance F Y
	 * C = Inheritance G X
	 *
	 * All variables of this partition are joint (F and G are
	 * constants). However relative to X components A and C are
	 * independent, while relative to Y component B is a specialization
	 * of component A.
	 *
	 * Let's rewrite the component variables by explicitly showing
	 * variable occurences in components
	 *
	 * A = Inheritance X1 Y1
	 * B = Inheritance F Y2
	 * C = Inheritance G X2
	 *
	 * The specialization relationship between A and B relative to Y
	 * allows us to infer that V(Y2) is a subset of V(Y1). Thus the
	 * number of possible values that Y2 can take is bounded by
	 * |V(Y1)|.
	 *
	 * Let's calculate P(X1=X2) and P(Y1=Y2) for this pattern.
	 *
	 * P(X1=X2) = 1/|U| * 1/|U| * |U|
	 *          = 1/|U|
	 *
	 * the first 1/|U| is because each value of X1 can be any value of
	 * the universe U. The second 1/|U| is because, and since A and C
	 * are independent relative to X, each value of X2 can also be any
	 * value of U. Then we multiple by |U| because the equality may
	 * occur for each possible value of X1 or X2, so the probabilities
	 * add up. Now for P(Y1=Y2)
	 *
	 * P(Y1=Y2) = 1/|U| * 1/|V(Y1)| * |U|
	 *          = 1/|V(Y1)|
	 *
	 * 1/|U| is because Y1 in the more abstract component A can take
	 * any value of U. For any value of Y1 however, Y2 can only take a
	 * value of V(Y1) since B is a specialization of A relative to
	 * Y. Then we multiple by |U| to add up all probabilities for each
	 * values of the more abstract component A.
	 *
	 * ### Second Example
	 *
	 * pattern
	 * =
	 * Lambda
	 *   X Y Z
	 *   And
	 *     Inheritance X Y
	 *     Inheritance Z Y
	 *
	 * Assuming a partition of the 2 components
	 *
	 * A = Inheritance X Y
	 * B = Inheritance Z Y
	 *
	 * Thus after explicitly showing variable occurences
	 *
	 * A = Inheritance X Y1
	 * B = Inheritance Z Y2
	 *
	 * P(Y1=Y2) = 1/|U| * 1/|V(Y1)| * |U|
	 *          = 1/|V(Y1)|
	 *
	 * Here A and B are actually equivalent relative to Y, meaning the
	 * specialization relationship must not be strict.
	 *
	 * ### Third Example
	 *
	 * pattern
	 * =
	 * Lambda
	 *   X Y Z W
	 *   And
	 *     List X Y X
	 *     List Z W X
	 *
	 * Assuming a partition of the 2 components
	 *
	 * A = List X Y X
	 * B = List Z W X
	 *
	 * Thus after explicitly showing variable occurences
	 *
	 * A = List X1 Y X1
	 * B = List Z W X2
	 *
	 * P(X1=X2) = 1/|U| * 1/|V(X2)| * |U| = 1/|V(X2)|
	 *
	 * It doesn't matter that X1 appears twice in A, the number values
	 * it can take is still bounded by its most specialized abstraction
	 * relative to X, that is B.
	 *
	 * ## General formula
	 *
	 * Let X be a variable, with n variable occurences X1, ..., Xn in
	 * the respective components C1, ..., Cn. Without loss of
	 * generality let's assume that C1, ..., Cn are ordered such that
	 * if Ci is strictly more abstract than Cj relative to X, then
	 * i<j. This admits many orders as it doesn't impose restrictions
	 * on components that are equivalent or independent.  But it
	 * doesn't matter as it is only used to avoid cycles in the
	 * calculation of the probability estimate. Then the general
	 * formula is
	 *
	 * P(X1=...=Xn) = Prod_{j=2}^n 1/M(Xj)
	 *
	 * where M(Xj) is either
	 *
	 * 1. |V(Xi)|, the count of Xi in the most specialized component Ci
	 * that is either more abstract than or equivalent to Cj relative
	 * to X and such that i<j.
	 *
	 * 2. |U|, if no such more abstract or equivalent component exists
	 * for Xj.
	 *
	 * A proof sketch of why it is a good estimate of P(X1=...=Xn)
	 * (under independence assumption of the data) is that the
	 * syntactic specialization relationship provides a prior to
	 * discard distributions (i.e. set their prior probabilities to 0)
	 * of values of variable occurences Xi using subset relationships
	 * between V(Xi) and V(Xj).
	 *
	 * One last remark: The count |V(Xi)| can be exact or approximated.
	 * Of course the estimate will be better if the count is exact.
	 * Since such count is only consider component by component, and
	 * interactions are never used to obtain that count, having an
	 * exact count does not invalidate the surprisingness measure.
	 * However it can be computationally costly, so an option is to
	 * approximate it.  One possible approximation under independence
	 * assumptions is as follows.  Assume the component has N
	 * variables, if these variables are completely independent then
	 * the number of values of each of them is the N-th root of the
	 * component support S
	 *
	 * |V(Xi)| ~= Nth-root(S)
	 *
	 * such that the final support can be obtained by multiplying the
	 * number of values of all variables of that component.
	 *
	 * As of today the code calculates the exact count (thus is rather
	 * slow). We have not experimented with approximated counts yet.
	 */
	static double isurp(const Handle& pattern,
	                    const HandleSeq& texts,
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
	static HandleSeqSeqSeq partitions_without_pattern(const Handle& pattern);

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
	                            const HandleSeq& texts);

	/**
	 * Return the probability distribution over value of var in the
	 * given subpattern/block against a given database.
	 */
	static HandleCounter value_distribution(const HandleSeq& block,
	                                        const Handle& var,
	                                        const HandleSeq& texts);

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
	 * Calculate the universe count of the pattern over the given texts
	 */
	static double universe_count(const Handle& pattern, const HandleSeq& texts);

	/**
	 * Given a pattern, a corpus and a probability, calculate the
	 * support of that pattern.
	 */
	static double prob_to_support(const Handle& pattern,
	                              const HandleSeq& texts,
	                              double prob);

	/**
	 * Calculate the empiric probability of a pattern according to a
	 * database texts.
	 */
	static double emp_prob(const Handle& pattern, const HandleSeq& texts);

	/**
	 * Like emp_prob with memoization.
	 */
	static double emp_prob_mem(const Handle& pattern,
	                           const HandleSeq& texts);

	/**
	 * Like emp_prob but subsample the texts to have subsize (if texts
	 * size is greater than subsize).
	 *
	 * TODO: memoizing support is current disabled.
	 */
	static double emp_prob_subsmp(const Handle& pattern,
	                              const HandleSeq& texts,
	                              unsigned subsize=UINT_MAX);

	/**
	 * Randomly subsample texts so that the resulting texts has size
	 * subsize.
	 */
	static HandleSeq subsmp(const HandleSeq& texts, unsigned subsize);

	/**
	 * Like emp_prob but uses bootstrapping for more efficiency. nbs is
	 * the number of subsamplings taking place, and subsize is the size
	 * of each subsample.
	 *
	 * TODO: memoizing support is current disabled.
	 */
	static double emp_prob_bs(const Handle& pattern,
	                          const HandleSeq& texts,
	                          unsigned n_resample,
	                          unsigned subsize);

	/**
	 * Calculate the empirical probability of the given pattern,
	 * possibly boostrapping if necessary. The heuristic to determine
	 * whether the booststrapping should take place, and how, is
	 * calculated based on the pattern, the texts size and the
	 * probability estimate of the pattern.
	 *
	 * pbs stands for possibly boostrapping.
	 */
	static double emp_prob_pbs(const Handle& pattern,
	                           const HandleSeq& texts,
	                           double prob_estimate);

	/**
	 * Like emp_prob_pbs with memoization.
	 */
	static double emp_prob_pbs_mem(const Handle& pattern,
	                               const HandleSeq& texts,
	                               double prob_estimate);

	/**
	 * Determine the number of samples and the subsample size given a
	 * text corpus. The goal here to subsample so that the support does
	 * not exceed the corpus size. The upper bound of the support grows
	 * exponentially with the number conjuncts and polynomially (with
	 * maximum degree the number of conjuncts) with the size of the
	 * corpus. We try first to estimate how fast the support grows
	 * using the support estimate obtained by ji_prob to find what
	 * corpus size should be so that the support is roughly equal to
	 * the corpus size (because we can assume that the user at least
	 * tolerates an amount of resources in space and time that is
	 * already allocated for the corpus itself).
	 *
	 * The heuristic estimating the support from the corpus size, ts,
	 * and the number of conjuncts, nc, is
	 *
	 * f(ts, nc) = alpha * ts^nc
	 *
	 * This is an incredibly bad estimate, but it is the one we're
	 * using for now.
	 *
	 * We want to find the new corpus size nts that verifies the
	 * following equation
	 *
	 * f(nts, nc) = ts
	 *
	 * Thus
	 *
	 * alpha * nts^nc = ts
	 * nts = (ts/alpha)^{1/nc}
	 *
	 * where alpha is calculated as follows
	 *
	 * alpha = support_estimate / ts^nc
	 */
	static unsigned subsmp_size(const Handle& pattern,
	                            const HandleSeq& texts,
	                            double support_estimate);

	/**
	 * Calculate probability estimate of a pattern given a partition,
	 * assuming all blocks are independent, but takes into account the
	 * joint variables (ji in ji_prob stands for joint-independent).
	 *
	 * An atomspace is provided to memoize the support of subpatterns.
	 */
	static double ji_prob(const HandleSeqSeq& partition,
	                      const Handle& pattern,
	                      const HandleSeq& texts);

	/**
	 * Return true iff the given variable has the same position (same
	 * index) in the variable declarations of both patterns.
	 */
	static bool has_same_index(const Handle& l_pat,
	                           const Handle& r_pat,
	                           const Handle& var);

	/**
	 * Tell whether 2 blocks/subpatterns are equivalent relatie to a
	 * given variable. Basically, whether both block are semantically
	 * equivalent and var is in the same position in both of them.
	 *
	 * For instance
	 *
	 * l_blk = { Inh X Y }
	 * r_blk = { Inh Z Y }
	 *
	 * are equivalent relative to Y because are both are semantically
	 * equivalent (up to an alpha-conversion) and Y is used in the same
	 * place in both blocks.
	 */
	static bool is_equivalent(const HandleSeq& l_blk,
	                          const HandleSeq& r_blk,
	                          const Handle& var);

	/**
	 * List above but takes scope links instead of blocks (whether each
	 * scope link has the conjunction of clauses of its block as body).
	 */
	static bool is_equivalent(const Handle& l_pat,
	                          const Handle& r_pat,
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
	 * Tell whether the left block/subpattern is is syntactically more
	 * abstract than the right block/subpattern relative to a given
	 * variable.
	 *
	 * For instance
	 *
	 * l_blk = { List X Y Z }
	 * r_blk = { List W A Z }
	 *
	 * l_blk is more abstract than r_blk relative to Z because the
	 * matching values of Z in l_blk is a subset of the matching values
	 * of Z in l_blk.
	 */
	static bool is_syntax_more_abstract(const HandleSeq& l_blk,
	                                    const HandleSeq& r_blk,
	                                    const Handle& var);

	/**
	 * List above but takes scope links instead of blocks (whether each
	 * scope link has the conjunction of clauses of its block as body).
	 *
	 * TODO: for now, this code relies on unification. However it can
	 * certainly be optimized by not relying on unification and being
	 * re-implemented instead, and perhaps it could then be merged to
	 * the unification code.
	 */
	static bool is_syntax_more_abstract(const Handle& l_pat,
	                                    const Handle& r_pat,
	                                    const Handle& var);

	/**
	 * Like is_syntax_more_abstract but takes into account a bit of
	 * semantics as well (though none that requires data), in
	 * particular it handles conjunctions of clauses such that l_pat is
	 * more abstract if there exists a partition lp of l_pat (meaning a
	 * partition of conjunctions of clauses in l_pat) such that there
	 * exists a subset rs of r_pat (meaning a subset of clauses of
	 * r_pat) such that rs is a syntactic specialization, relative to
	 * var, of each block lb of lp.
	 *
	 * For instance
	 *
	 * l_pat = Lambda
	 *           X Y Z
	 *           And
	 *             Inheritance
	 *               X
	 *               Z
	 *             Inheritance
	 *               Y
	 *               Z
	 *
	 * r_pat = Lambda
	 *           Z
	 *           Inheritance
	 *             A
	 *             Z
	 *
	 * var = Z
	 *
	 * l_pat is indeed an abstraction of r_pat because there exists a
	 * partition lp = { {Inheritance X Z}, {Inheritance Y Z} } such
	 * that the subset { Inheritance A Z} is a syntactic specialization
	 * of each block of lp.
	 */
	static bool is_more_abstract(const Handle& l_pat,
	                             const Handle& r_pat,
	                             const Handle& var);

	/**
	 * Like above but consider list of clauses instead of patterns.
	 */
	static bool is_more_abstract(const HandleSeq& l_blk,
	                             const HandleSeq& r_blk,
	                             const Handle& var);

	/**
	 * Like powerset but return a sequence of sequences instead of set
	 * of sets. Discard the empty sequence.
	 */
	static HandleSeqSeq powerseq_without_empty(const HandleSeq& blk);

	/**
	 * Return true iff l_blk is strictly more abstract than r_blk
	 * relative to var. That is more abstract and not equivalent.
	 */
	static bool is_strictly_more_abstract(const HandleSeq& l_blk,
	                                      const HandleSeq& r_blk,
	                                      const Handle& var);

	/**
	 * Return true iff var_val is a pair with the first element a
	 * variable in vars, and the second element a value (non-variable).
	 */
	static bool is_value(const Unify::HandleCHandleMap::value_type& var_val,
	                     const Variables& vars);

	/**
	 * Sort the partition such that if block A is strictly more
	 * abstract than block B relative var, then A occurs before B.
	 */
	static void rank_by_abstraction(HandleSeqSeq& partition, const Handle& var);

	/**
	 * Copy all subpatterns/blocks where var appears. Also remove all
	 * parts of the subpatterns that are not strongly connected with to
	 * it relative to var.
	 *
	 * So for instance
	 *
	 * partition = { { Inheritance X Y, Inheritance Z A},
	 *               { Inheritance X B, Inheritance Z Y} }
	 *
	 * var = Y
	 *
	 * returns
	 *
	 * { {Inheritance Z Y } }
	 *
	 * because
	 *
	 * 1. Y only appears in the second block
	 *
	 * 2. within that block
	 *
	 *    Inheritance X B
	 *
	 *    is not strongly connected to the component where Y appears.
	 *
	 * Ignoring non-strongly connected components allows to speed up
	 * Surprisingness::value_count as well as covering more cases in
	 * is_more_abstract.
	 */
	static HandleSeqSeq connected_subpatterns_with_var(const HandleSeqSeq& partition,
	                                                   const Handle& var);
	static HandleSeq connected_subpattern_with_var(const HandleSeq& blk,
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
	 * estimate of being assigned the same value across all blocks.
	 */
	static double eq_prob(const HandleSeqSeq& partition,
	                      const Handle& pattern,
	                      const HandleSeq& texts);

	/**
	 * Alternate implementation of eq_prob. Takes into syntactical
	 * abstraction between blocks in order to better estimate variable
	 * occurance equality (see the comment above isurp).
	 */
	static double eq_prob_alt(const HandleSeqSeq& partition,
	                          const Handle& pattern,
	                          const HandleSeq& texts);

	/**
	 * Key of the empirical probability value
	 */
	static const Handle& emp_prob_key();

	/**
	 * Get/set the empirical probability of the given pattern.
	 */
	static TruthValuePtr get_emp_prob(const Handle& pattern);
	static void set_emp_prob(const Handle& pattern, double emp_prob);
};

/**
 * Given a partition, that is a sequence of blocks, where each
 * block is a sequence of handles, return
 */
std::string oc_to_string(const HandleSeqSeqSeq& hsss,
                         const std::string& indent=empty_string);

} // ~namespace opencog

#endif /* OPENCOG_SURPRISINGNESS_H_ */
