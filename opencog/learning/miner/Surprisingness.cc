/*
 * Surprisingness.cc
 *
 * Copyright (C) 2019 OpenCog Foundation
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

#include "Surprisingness.h"

#include "MinerUtils.h"

#include <opencog/util/algorithm.h>
#include <opencog/util/empty_string.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atoms/core/FindUtils.h>
#include <opencog/atoms/core/LambdaLink.h>

#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/range/numeric.hpp>
#include <boost/math/special_functions/binomial.hpp>

#include <cmath>
#include <functional>
#include <limits>

namespace opencog {

double Surprisingness::isurp_old(const Handle& pattern,
                                 const HandleSet& texts,
                                 bool normalize)
{
	// Strictly speaking it should be the power but we use binomial for
	// backward compatibility.
	unsigned int n = texts.size(), k = MinerUtils::n_conjuncts(pattern);
	double total_count = boost::math::binomial_coefficient<double>(n, k);

	// Function calculating the probability of a pattern
	auto prob = [&](const Handle& pattern) {
		double sup = MinerUtils::calc_support(pattern, texts, (unsigned)total_count);
		return sup / total_count;
	};
	auto blk_prob = [&](const HandleSeq& block) {
		return prob(add_pattern(block, *pattern->getAtomSpace()));
	};

	// Calculate the probability of pattern
	double pattern_prob = prob(pattern);

	// Calculate the probability estimate of each partition based on
	// independent assumption of between each partition block.
	auto iprob = [&](const HandleSeqSeq& partition) {
		return boost::accumulate(partition | boost::adaptors::transformed(blk_prob),
		                         1.0, std::multiplies<double>());
	};
	HandleSeqSeqSeq prtns = partitions_without_pattern(pattern);
	std::vector<double> estimates(prtns.size());
	boost::transform(prtns, estimates.begin(), iprob);
	auto p = std::minmax_element(estimates.begin(), estimates.end());
	double emin = *p.first, emax = *p.second;

	// Calculate the I-Surprisingness, normalized if requested.
	double dst = dst_from_interval(emin, emax, pattern_prob);
	return normalize ? dst / pattern_prob : dst;
}

double Surprisingness::isurp(const Handle& pattern,
                             const HandleSet& texts,
                             bool normalize)
{
	// Calculate the empiric probability of pattern
	double emp = emp_prob(pattern, texts);

	// Calculate the probability estimate of each partition based on
	// independent assumption of between each partition block, taking
	// into account the linkage probability.
	std::vector<double> estimates;
	for (const HandleSeqSeq& partition : partitions_without_pattern(pattern)) {
		double jip = ji_prob(partition, pattern, texts);
		estimates.push_back(jip);
		// logger().debug() << "jip = " << jip;
		// double ratio = jip / emp;
		// logger().debug() << "ratio = " << ratio;
	}
	auto mmp = std::minmax_element(estimates.begin(), estimates.end());
	double emin = *mmp.first, emax = *mmp.second;
	// logger().debug() << "emin = " << emin << ", emax = " << emax
	//                  << ", emp = " << emp;

	// Calculate the I-Surprisingness, normalized if requested.
	double dst = dst_from_interval(emin, emax, emp);
	// logger().debug() << "dst = " << dst
	//                  << ", normalize = " << normalize
	//                  << ", ndst = " << dst / emp;

	return normalize ? dst / emp : dst;
}

Handle Surprisingness::isurp_key()
{
	static Handle isurp_key = createNode(NODE, "*-I-SurprisingnessValueKey-*");
	return isurp_key;
}

double Surprisingness::get_isurp_value(const Handle& pattern)
{
	return FloatValueCast(pattern->getValue(isurp_key()))->value().front();
}

double Surprisingness::dst_from_interval(double l, double u, double v)
{
	return (u < v ? v - u : (v < l ? l - v : 0.0));
}

HandleSeqSeqSeq Surprisingness::combinatorial_insert(const Handle& h,
                                                     const HandleSeqSeq& hss)
{
	return combinatorial_insert(h, hss.begin(), hss.end());
}

HandleSeqSeqSeq Surprisingness::combinatorial_insert(const Handle& h,
                                                     HandleSeqSeq::const_iterator from,
                                                     HandleSeqSeq::const_iterator to)
{
	// Base case
	if (from == to)
		return {{{h}}};

	// Recursive case
	HandleSeq head = *from;       // Copy because will get modified
	HandleSeqSeqSeq rst;
	for (auto x : combinatorial_insert(h, ++from, to)) {
		x.push_back(head);
		rst.push_back(x);
	}
	head.push_back(h);
	HandleSeqSeq fst(from, to);
	fst.push_back(head);
	rst.push_back(fst);
	return rst; 
}

HandleSeqSeqSeq Surprisingness::partitions(const HandleSeq& hs)
{
	return partitions(hs.begin(), hs.end());
}

HandleSeqSeqSeq Surprisingness::partitions(HandleSeq::const_iterator from,
                                           HandleSeq::const_iterator to)
{
	// Base case
	if (from == to)
		return {{}};

	// Recursive case
	Handle head = *from;
	HandleSeqSeqSeq res;
	for (const HandleSeqSeq& partition : partitions(++from, to)) {
		HandleSeqSeqSeq subparts = combinatorial_insert(head, partition);
		res.insert(res.end(), subparts.begin(), subparts.end());
	}
	return res;
}

HandleSeqSeqSeq Surprisingness::partitions_without_pattern(const Handle& pattern)
{
	HandleSeqSeqSeq prtns = partitions(MinerUtils::get_clauses(pattern));
	prtns.resize(prtns.size() - 1);
	// prtns.resize(1); // comment this output to only consider
   //                  // singleton blocks (convenient for debugging)
	return prtns;
}

Handle Surprisingness::add_pattern(const HandleSeq& block, AtomSpace& as)
{
	return as.add_link(LAMBDA_LINK,
	                   block.size() == 1 ? block.front()
	                   : as.add_link(AND_LINK, block));
}

HandleSeq Surprisingness::add_subpatterns(const HandleSeqSeq& partition,
                                          const Handle& pattern,
                                          AtomSpace& as)
{
	HandleSeq subpatterns(partition.size());
	boost::transform(partition, subpatterns.begin(), [&](const HandleSeq& blk) {
			return add_pattern(blk, as); });
	return subpatterns;
}

HandleSeq Surprisingness::joint_variables(const Handle& pattern,
                                          const HandleSeqSeq& partition)
{
	HandleUCounter var_count;

	for (const Handle& var : MinerUtils::get_variables(pattern).varset)
		for (const HandleSeq& blk : partition)
			if (is_free_in_any_tree(blk, var))
				++var_count[var];

	HandleSeq jvs;
	for (const auto& vc : var_count)
		if (1 < vc.second)
			jvs.push_back(vc.first);

	return jvs;
}

unsigned Surprisingness::value_count(const HandleSeq& block,
                                     const Handle& var,
                                     const HandleSet& texts)
{
	Valuations vs(MinerUtils::mk_pattern_without_vardecl(block), texts);
	HandleUCounter values = vs.values(var);
	return values.keys().size();
}

HandleCounter Surprisingness::value_distribution(const HandleSeq& block,
                                                 const Handle& var,
                                                 const HandleSet& texts)
{
	Valuations vs(MinerUtils::mk_pattern_without_vardecl(block), texts);
	HandleUCounter values = vs.values(var);
	HandleCounter dist;
	double total = values.total_count();
	for (const auto& v : values)
		dist[v.first] = v.second / total;
	return dist;
}

double Surprisingness::inner_product(const std::vector<HandleCounter>& dists)
{
	// Find the common intersection of values
	HandleSet cvals = dists[0].keys();
	for (size_t i = 1; i < dists.size(); i++)
		cvals = opencog::set_intersection(cvals, dists[i].keys());

	// Calculate the inner product of all distributions across the
	// common values
	double p = 0.0;
	for (const Handle& v : cvals) {
		double inner = 1.0;
		for (const HandleCounter& dist : dists)
			inner *= dist.at(v);
		p += inner;
	}
	return p;
}

double Surprisingness::emp_prob(const Handle& pattern, const HandleSet& texts)
{
	double ucount = pow((double)texts.size(), MinerUtils::n_conjuncts(pattern));
	unsigned ms = (unsigned)std::min((double)UINT_MAX, ucount);
	double sup = MinerUtils::calc_support(pattern, texts, ms);
	// logger().debug() << "emp_prob(" << oc_to_string(pattern) << ") = " << "sup = " << sup << ", ucount = " << ucount << ", res = " << sup/ucount;
	return sup / ucount;
}

double Surprisingness::ji_prob(const HandleSeqSeq& partition,
                               const Handle& pattern,
                               const HandleSet& texts)
{
	// Generate subpatterns from blocks (add them in the atomspace to
	// memoize support calculation)
	HandleSeq subpatterns = add_subpatterns(partition, pattern,
	                                        *pattern->getAtomSpace());

	// Calculate the product of the probability over subpatterns
	// without considering joint variables
	double p = 1.0;
	for (const Handle& subpattern : subpatterns)
		p *= emp_prob(subpattern, texts);
	// logger().debug() << "ji_prob subpattern emp product = " << p;

	// Calculate the probability that all joint variables take the same
	// value
	double eq_p = eq_prob(partition, pattern, texts);
	// logger().debug() << "ji_prob P(equal) = " << eq_p;
	p *= eq_p;

	return p;
}

bool Surprisingness::has_same_index(const Handle& l_pat,
                                    const Handle& r_pat,
                                    const Handle& var)
{
	const Variables& lv = LambdaLinkCast(l_pat)->get_variables();
	const Variables& rv = LambdaLinkCast(r_pat)->get_variables();
	auto lv_it = lv.index.find(var);
	auto rv_it = rv.index.find(var);
	return lv_it != lv.index.end() and rv_it != lv.index.end()
		and lv_it->second == rv_it->second;
}

bool Surprisingness::is_equivalent(const HandleSeq& l_blk,
                                   const HandleSeq& r_blk,
                                   const Handle& var)
{
	Handle l_pat = MinerUtils::mk_pattern_without_vardecl(l_blk);
	Handle r_pat = MinerUtils::mk_pattern_without_vardecl(r_blk);
	return is_equivalent(l_pat, r_pat, var);
}

bool Surprisingness::is_equivalent(const Handle& l_pat,
                                   const Handle& r_pat,
                                   const Handle& var)
{
	return content_eq(l_pat, r_pat) and has_same_index(l_pat, r_pat, var);
}

HandleSeqUCounter::const_iterator Surprisingness::find_equivalent(
	const HandleSeqUCounter& partition_c,
	const HandleSeq& block,
	const Handle& var)
{
	auto it = partition_c.begin();
	for (; it != partition_c.end(); it++)
		if (is_equivalent(it->first, block, var))
			return it;
	return it;
}

HandleSeqUCounter::iterator Surprisingness::find_equivalent(
	HandleSeqUCounter& partition_c,
	const HandleSeq& block,
	const Handle& var)
{
	auto it = partition_c.begin();
	for (; it != partition_c.end(); it++)
		if (is_equivalent(it->first, block, var))
			return it;
	return it;
}

bool Surprisingness::is_syntax_more_abstract(const HandleSeq& l_blk,
                                             const HandleSeq& r_blk,
                                             const Handle& var)
{
	Handle l_pat = MinerUtils::mk_pattern_without_vardecl(l_blk);
	Handle r_pat = MinerUtils::mk_pattern_without_vardecl(r_blk);
	return is_syntax_more_abstract(l_pat, r_pat, var);
}

bool Surprisingness::is_syntax_more_abstract(const Handle& l_pat,
                                             const Handle& r_pat,
                                             const Handle& var)
{
	Variables l_vars = MinerUtils::get_variables(l_pat);
	Variables r_vars = MinerUtils::get_variables(r_pat);
	Handle l_body = MinerUtils::get_body(l_pat);
	Handle r_body = MinerUtils::get_body(r_pat);

	// Let's first make sure that var is both in l_pat and r_pat
	if (not l_vars.is_in_varset(var) or not r_vars.is_in_varset(var))
		return false;

	// Remove var from l_vars and r_vars to be considered as value
	// rather than variable.
	l_vars.erase(var);
	r_vars.erase(var);

	// Find all mappings from variables (except var) to terms
	Unify unify(l_body, r_body, l_vars, r_vars);
	// logger().debug() << "unify" << std::endl
	//                  << "l_body:" << std::endl << oc_to_string(l_body)
	//                  << "r_body:" << std::endl << oc_to_string(r_body)
	//                  << "l_vars:" << std::endl << oc_to_string(l_vars)
	//                  << "r_vars:" << std::endl << oc_to_string(r_vars);
	Unify::SolutionSet sol = unify();

	// If it is not satisfiable, l_pat is not an abstraction
	//
	// TODO: case of nary conjunctions additional care is needed
	if (not sol.is_satisfiable())
		return false;

	Unify::TypedSubstitutions tsub = unify.typed_substitutions(sol, r_body);

	// Check that for all mappings no variable in r_vars maps to a
	// value (non-variable).
	for (const auto& var2val_map : tsub)
		for (const auto& var_val : var2val_map.first)
			if (is_value(var_val, r_vars))
				return false;
	return true;
}

bool Surprisingness::is_more_abstract(const Handle& l_pat,
                                      const Handle& r_pat,
                                      const Handle& var)
{
	HandleSeq l_clauses = MinerUtils::get_clauses(l_pat);
	HandleSeq r_clauses = MinerUtils::get_clauses(r_pat);
	HandleSeq l_scs = connected_subpattern_with_var(l_clauses, var);
	HandleSeq r_scs = connected_subpattern_with_var(r_clauses, var);
	return is_more_abstract(l_scs, r_scs, var);
}

bool Surprisingness::is_more_abstract(const HandleSeq& l_blk,
                                      const HandleSeq& r_blk,
                                      const Handle& var)
{
	using namespace boost::algorithm;
	HandleSeqSeq rps = powerseq_without_empty(r_blk);
	return any_of(partitions(l_blk), [&](const HandleSeqSeq& lp) {
			return any_of(rps, [&](const HandleSeq& rs) {
					return all_of(lp, [&](const HandleSeq& lb) {
							return is_syntax_more_abstract(lb, rs, var);
						});
				});
		});
}

HandleSeqSeq Surprisingness::powerseq_without_empty(const HandleSeq& blk)
{
	HandleSetSet pset = powerset(HandleSet(blk.begin(), blk.end()));
	HandleSeqSeq pseq;
	for (const HandleSet& set : pset)
		if (not set.empty())
			pseq.push_back(HandleSeq(set.begin(), set.end()));
	return pseq;
}

bool Surprisingness::is_strictly_more_abstract(const HandleSeq& l_blk,
                                               const HandleSeq& r_blk,
                                               const Handle& var)
{
	return not is_equivalent(l_blk, r_blk, var)
		and is_more_abstract(l_blk, r_blk, var);
}

bool Surprisingness::is_value(const Unify::HandleCHandleMap::value_type& var_val,
                              const Variables& vars)
{
	return vars.is_in_varset(var_val.first)
		and not var_val.second.is_free_variable();
}

void Surprisingness::rank_by_abstraction(HandleSeqSeq& partition, const Handle& var)
{
	boost::sort(partition,
	            [&](const HandleSeq& l_blk, const HandleSeq& r_blk) {
		            // sort operates on strict weak order so is
		            // compatible with is_strictly_more_abstract
		            return is_strictly_more_abstract(l_blk, r_blk, var);
	            });
}

HandleSeqSeq Surprisingness::connected_subpatterns_with_var(
	const HandleSeqSeq& partition,
	const Handle& var)
{
	HandleSeqSeq var_partition;
	for (const HandleSeq& blk : partition) {
		HandleSeq sc_blk = connected_subpattern_with_var(blk, var);
		if (not sc_blk.empty()) {
			var_partition.push_back(sc_blk);
		}
	}
	return var_partition;
}

HandleSeq Surprisingness::connected_subpattern_with_var(const HandleSeq& blk,
                                                        const Handle& var)
{
	if (not is_free_in_any_tree(blk, var))
		return {};

	HandleSeqSeq sccs = MinerUtils::get_components(blk);
	for (const HandleSeq& scc : sccs)
		if (is_free_in_any_tree(scc, var))
			return scc;
	return {};
}

HandleSeqUCounter Surprisingness::group_eq(const HandleSeqSeq& partition,
                                           const Handle& var)
{
	HandleSeqUCounter partition_c;
	for (const HandleSeq& blk : partition) {
		if (is_free_in_any_tree(blk, var)) {
			auto it = find_equivalent(partition_c, blk, var);
			if (it == partition_c.end()) {
				partition_c[blk] = 1;
			} else {
				it->second++;
			}
		}
	}
	return partition_c;
}

double Surprisingness::eq_prob(const HandleSeqSeq& partition,
                               const Handle& pattern,
                               const HandleSet& texts)
{
	// logger().debug() << "Surprisingness::eq_prob_alt partition:"
	//                  << std::endl << oc_to_string(partition)
	//                  << ", pattern:" << std::endl << oc_to_string(pattern)
	//                  << ", texts.size() = " << texts.size();

	double p = 1.0;
	// Calculate the probability of a variable taking the same value
	// across all blocks/subpatterns where that variable appears.
	for (const Handle& var : joint_variables(pattern, partition)) {
		// logger().debug() << "var = " << oc_to_string(var);

		// Select all strongly connected subpatterns containing var
		HandleSeqSeq var_partition = connected_subpatterns_with_var(partition, var);
		// logger().debug() << "var_partition = " << oc_to_string(var_partition);

		// For each variable, sort the partition so that abstract
		// blocks, relative to var, appear first.
		rank_by_abstraction(var_partition, var);
		// logger().debug() << "var_partition (after ranking by abstraction) = "
		//                  << oc_to_string(var_partition);

		// For each block j_blk, but the first, look for the most
		// specialized block that is more abstract or equivalent to that
		// block relative to var, i_blk, and use the fact that var
		// cannot take more values than its count in i_blk. If no such
		// i_blk block exists, then use uc=|U| as count.
		for (int j = 1; j < (int)var_partition.size(); j++) {
			// logger().debug() << "var_partition[" << j << "]:" << std::endl
			//                  << oc_to_string(var_partition[j]);

			// Since abstraction relation is transitive, one can just go
			// backward from j_blk and pick up the first i_blk that is
			// either equivalent or more abstract, it will be the most
			// specialized abstraction.
			int i = j-1;
			while (0 <= i)
				if (is_more_abstract(var_partition[i], var_partition[j], var))
					break;
				else i--;

			// if (0 <= i)
			// 	logger().debug() << "var_partition[" << i
			// 	                 << "] (most specialized abstraction):" << std::endl
			// 	                 << oc_to_string(var_partition[j]);
			// else
			// 	logger().debug() << "no abstraction (i = " << i << ")";

			double c = texts.size();
			if (0 <= i)
				c = value_count(var_partition[i], var, texts);
			// logger().debug() << "c = " << c;
			p /= c;
		}
	}
	return p;
}

std::string oc_to_string(const HandleSeqSeqSeq& hsss, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << hsss.size() << std::endl;
	size_t i = 0;
	for (const HandleSeqSeq& hss : hsss) {
		ss << indent << "atoms sets[" << i << "]:" << std::endl
		   << oc_to_string(hss, indent + oc_to_string_indent);
		i++;
	}
	return ss.str();
}

} // namespace opencog
