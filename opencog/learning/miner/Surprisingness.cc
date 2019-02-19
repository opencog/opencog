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

#include "Surprisingness.h"

#include "MinerUtils.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>

#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/numeric.hpp>
#include <boost/math/special_functions/binomial.hpp>

#include <cmath>
#include <functional>

namespace opencog {

double Surprisingness::ISurprisingness_old(const Handle& pattern,
                                           const HandleSeqSeq& partitions,
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

	// Calculate the probability of pattern
	double pattern_prob = prob(pattern);

	// Calculate the probability estimate of each partition based on
	// independent assumption of between each partition block.
	auto iprob = [&](const HandleSeq& partition) {
		return boost::accumulate(partition | boost::adaptors::transformed(prob),
		                         1.0, std::multiplies<double>());
	};
	std::vector<double> estimates(partitions.size());
	boost::transform(partitions, estimates.begin(), iprob);
	auto p = std::minmax_element(estimates.begin(), estimates.end());
	double emin = *p.first, emax = *p.second;

	// Calculate the I-Surprisingness, normalized if requested.
	double dst = dst_from_interval(emin, emax, pattern_prob);
	return normalize ? dst / pattern_prob : dst;
}

double Surprisingness::ISurprisingness_old(const Handle& patterns,
                                           const Handle& partitions,
                                           const HandleSet& texts,
                                           bool normalize)
{
	HandleSeqSeq partitions_ss(partitions->get_arity());
	boost::transform(partitions->getOutgoingSet(), partitions_ss.begin(),
	                 [&](const Handle& partition) {
		                 return partition->getOutgoingSet();
	                 });
	return ISurprisingness_old(patterns, partitions_ss, texts, normalize);
}

double Surprisingness::ISurprisingness(const Handle& pattern,
                                       const HandleSeqSeq& partitions,
                                       const HandleSet& texts,
                                       bool normalize)
{
	// Function calculating the probability of a pattern
	auto prob = [&](const Handle& pattern) {
		double ucount = pow((double)texts.size(), MinerUtils::n_conjuncts(pattern));
		double sup = MinerUtils::calc_support(pattern, texts, (unsigned)ucount);
		logger().debug() << "prob(" << oc_to_string(pattern) << ") = " << "sup = " << sup << ", ucount = " << ucount << ", res = " << sup/ucount;
		return sup / ucount;
	};

	// Calculate the probability of pattern
	double pattern_prob = prob(pattern);

	// Calculate the probability estimate of each partition based on
	// independent assumption of between each partition block, taking
	// into account the linkage probability (probability that 2
	// variables have the same value).
	auto iprob = [&](const HandleSeq& partition) {
		// Calculate the product of the probability of each conjuncts
		double p = boost::accumulate(partition | boost::adaptors::transformed(prob),
		                             1.0, std::multiplies<double>());

		// Calculate the linkage probability

		// TODO

		return p;
	};

	std::vector<double> estimates(partitions.size());
	boost::transform(partitions, estimates.begin(), iprob);
	auto p = std::minmax_element(estimates.begin(), estimates.end());
	double emin = *p.first, emax = *p.second;
	logger().debug() << "emin = " << emin << ", emax = " << emax;

	// Calculate the I-Surprisingness, normalized if requested.
	double dst = dst_from_interval(emin, emax, pattern_prob);
	return normalize ? dst / pattern_prob : dst;
}

double Surprisingness::ISurprisingness(const Handle& patterns,
                                       const Handle& partitions,
                                       const HandleSet& texts,
                                       bool normalize)
{
	HandleSeqSeq partitions_ss(partitions->get_arity());
	boost::transform(partitions->getOutgoingSet(), partitions_ss.begin(),
	                 [&](const Handle& partition) {
		                 return partition->getOutgoingSet();
	                 });
	return ISurprisingness(patterns, partitions_ss, texts, normalize);
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

std::string oc_to_string(const HandleSeqSeqSeq& hsss, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << hsss.size() << std::endl;
	size_t i = 0;
	for (const HandleSeqSeq& hss : hsss) {
		ss << indent << "atoms sets[" << i << "]:" << std::endl \
		   << oc_to_string(hss, indent + OC_TO_STRING_INDENT); \
		i++; \
	} \
	return ss.str(); \

}

} // namespace opencog
