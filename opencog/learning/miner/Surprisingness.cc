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
	HandleSeqSeqSeq prtns = partitions(pattern);
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
	for (const HandleSeqSeq& partition : partitions(pattern)) {
		double jip = ji_prob(partition, pattern, texts);
		estimates.push_back(jip);
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

HandleSeqSeqSeq Surprisingness::partitions(const Handle& pattern)
{
	HandleSeqSeqSeq prtns = partitions(MinerUtils::get_clauses(pattern));
	prtns.resize(prtns.size() - 1);
	// prtns.resize(1); // comment this output to only consider
	                    // singleton blocks (convenient for debugging)
	return prtns;
}

Handle Surprisingness::add_pattern(const HandleSeq& block, AtomSpace& as)
{
	return as.add_link(LAMBDA_LINK, as.add_link(AND_LINK, block));
}

LambdaLinkPtr Surprisingness::mk_lambda(const HandleSeq& block)
{
	return createLambdaLink(HandleSeq{createLink(block, AND_LINK)});
}

Handle Surprisingness::mk_pattern(const HandleSeq& block)
{
	return Handle(mk_lambda(block));
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
	Valuations vs(mk_pattern(block), texts);
	HandleUCounter values = vs.values(var);
	return values.keys().size();
}

HandleCounter Surprisingness::value_distribution(const HandleSeq& block,
                                                 const Handle& var,
                                                 const HandleSet& texts)
{
	Valuations vs(mk_pattern(block), texts);
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
	double sup = MinerUtils::calc_support(pattern, texts, (unsigned)ucount);
	// logger().debug() << "prob(" << oc_to_string(pattern) << ") = " << "sup = " << sup << ", ucount = " << ucount << ", res = " << sup/ucount;
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

bool Surprisingness::is_equivalent(const HandleSeq& l_blk,
                                   const HandleSeq& r_blk,
                                   const Handle& var)
{
	Handle l_pat = mk_pattern(l_blk);
	Handle r_pat = mk_pattern(r_blk);
	if (content_eq(l_pat, r_pat)) {
		const Variables& lv = LambdaLinkCast(l_pat)->get_variables();
		const Variables& rv = LambdaLinkCast(r_pat)->get_variables();
		auto lvit = lv.index.find(var);
		auto rvit = rv.index.find(var);
		return lvit != lv.index.end() and rvit != lv.index.end()
			and lvit->second == rvit->second;
	}
	return false;
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
	double p = 1.0;
	// Calculate the probability of a variable taking the same value
	// across all blocks/subpatterns where that variable appears.
	for (const Handle& var : joint_variables(pattern, partition)) {
		// logger().debug() << "var = " << oc_to_string(var);

		// Group subpatterns in equivalence blocks/subpatterns
		// w.r.t. var. For each subpattern divide p by the number of
		// values var can take (i.e. |texts| since each subpattern is
		// independent), then within each subpattern, divide as many
		// times as they are equivalent subpatterns by the number of
		// values var actually takes, since all subpatterns are
		// equivalent this restrict the universes where these
		// subpatterns are only over these existing values.
		double uc = texts.size();
		for (const auto& blk_c : group_eq(partition, var)) {
			// Take care of the independent part
			p /= uc;
			// logger().debug() << "divide p by " << uc
			//                  << " for block:" << std::endl
			//                  << oc_to_string(blk_c.first);
			// Take care of the dependent (equivalent for now) part
			if (1 < blk_c.second) {
				double c = value_count(blk_c.first, var, texts);
				p /= std::pow(c, blk_c.second - 1.0);
				// logger().debug() << "divide p by " << std::pow(c, blk_c.second - 1.0)
				//                  << " for " << blk_c.second - 1.0
				//                  << " equivalent block(s)";
			}
		}

		// Finally multiple by the maximum number of values to normalize
		// p, because P(equal) happens for each value.
		p *= uc;
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
