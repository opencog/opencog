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

double Surprisingness::ISurprisingness_old(const Handle& pattern,
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

double Surprisingness::ISurprisingness(const Handle& pattern,
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
		double jip = jiprob(partition, pattern, texts);
		estimates.push_back(jip);
		double ratio = jip / emp;
		logger().debug() << "ratio = " << ratio;
	}
	auto mmp = std::minmax_element(estimates.begin(), estimates.end());
	double emin = *mmp.first, emax = *mmp.second;
	logger().debug() << "emin = " << emin << ", emax = " << emax
	                 << ", emp = " << emp;

	// Calculate the I-Surprisingness, normalized if requested.
	double dst = dst_from_interval(emin, emax, emp);
	logger().debug() << "dst = " << dst
	                 << ", normalize = " << normalize
	                 << ", ndst = " << dst / emp;

	return normalize ? dst / emp : dst;
}

Handle Surprisingness::ISurprisingness_key()
{
	static Handle isurp_key = createNode(NODE, "*-I-SurprisingnessValueKey-*");
	return isurp_key;
}

double Surprisingness::get_ISurprisingness(const Handle& pattern)
{
	return FloatValueCast(pattern->getValue(ISurprisingness_key()))->value().front();
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
	// prtns.resize(prtns.size() - 1);
	prtns.resize(1);
	return prtns;
}

Handle Surprisingness::add_pattern(const HandleSeq& block, AtomSpace& as)
{
	return as.add_link(LAMBDA_LINK, as.add_link(AND_LINK, block));
}

HandleSeq Surprisingness::joint_variables(const Handle& pattern)
{
	HandleUCounter var_count;

	for (const Handle& var : MinerUtils::get_variables(pattern).varset)
		for (const Handle& clause : MinerUtils::get_clauses(pattern))
			if (is_free_in_tree(clause, var))
				++var_count[var];

	HandleSeq jvs;
	for (const auto vc : var_count)
		if (1 < vc.second)
			jvs.push_back(vc.first);

	return jvs;
}

HandleCounter Surprisingness::value_distribution(const HandleSeq& block,
                                                 const Handle& var,
                                                 const HandleSet& texts)
{
	Handle ll = Handle(createLambdaLink(HandleSeq{createLink(block, AND_LINK)}));
	Valuations vs(ll, texts);
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
	logger().debug() << "prob(" << oc_to_string(pattern) << ") = " << "sup = " << sup << ", ucount = " << ucount << ", res = " << sup/ucount;
	return sup / ucount;
}

double Surprisingness::jiprob(const HandleSeqSeq& partition,
                              const Handle& pattern,
                              const HandleSet& texts)
{
	// Generate subpatterns from blocks
	HandleSeq subpatterns(partition.size());
	AtomSpace& as = *pattern->getAtomSpace();
	boost::transform(partition, subpatterns.begin(), [&](const HandleSeq& blk) {
			return add_pattern(blk, as); });

	// Calculate the product of the probability across subpatterns
	double p = 1.0;
	for (const Handle& subpattern : subpatterns)
		p *= emp_prob(subpattern, texts);

	logger().debug() << "p = " << p;

	// Calculate the probability of a variable taking the same value
	// across all blocks/subpatterns where that variable appears.
	for (const Handle& var : joint_variables(pattern)) {
		logger().debug() << "var = " << oc_to_string(var);

		// Value probability distributions of var for each block
		// where it appears.
		std::vector<HandleCounter> dists;
		for (const HandleSeq& blk : partition)
			if (is_free_in_any_tree(blk, var))
				dists.push_back(value_distribution(blk, var, texts));

		// Calculate the probability that any value of var is the
		// same for all blocks where var appears.
		double pe = inner_product(dists);

		logger().debug() << "P(Equal) = " << pe;

		p *= pe;
	}

	logger().debug() << "iprob(" << oc_to_string(partition) << ") = " << p;

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
