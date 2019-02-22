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
#include <opencog/atoms/core/FindUtils.h>

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
	HandleSeqSeqSeq prtns = partitions_no_set(MinerUtils::get_clauses(pattern));
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
	const Variables& vars = MinerUtils::get_variables(pattern);
	auto iprob = [&](const HandleSeqSeq& partition) {
		// Generate subpatterns from blocks
		HandleSeq subpatterns(partition.size());
		AtomSpace& pat_as = *pattern->getAtomSpace();
		boost::transform(partition, subpatterns.begin(), [&](const HandleSeq& blk) {
				return add_pattern(blk, pat_as); });

		// Calculate the product of the probability across subpatterns
		double p = boost::accumulate(subpatterns | boost::adaptors::transformed(prob),
		                             1.0, std::multiplies<double>());

		logger().debug() << "p = " << p;

		// Calculate the probability of a variable taking the same value
		// across all blocks/subpatterns where that variable appears.
		// TODO: only consider linked variables
		for (const Handle& var : vars.varseq) {
			logger().debug() << "var = " << oc_to_string(var);

			// Calculate the support estimate of var in each block
			std::vector<double> sup_ests(partition.size());
			for (size_t i = 0; i < sup_ests.size(); i++) {
				double sup_est = 0.0;
				if (is_free_in_any_tree(partition[i], var)) {
					size_t vs = MinerUtils::get_variables(subpatterns[i]).varseq.size();
					double sup = MinerUtils::get_support(subpatterns[i]);
					sup_est = std::pow(sup, 1.0/vs);
				}
				sup_ests[i] = sup_est;
				logger().debug() << "sup_ests[" << i << "] = " << sup_est;
			}

			// Calculate the probability that var takes the same value
			// across all subpatterns
			double pe = 1.0;
			double min_s = std::numeric_limits<double>::max();
			for (double s : sup_ests) {
				if (0 < s) {
					pe /= s;
					min_s = std::min(min_s, s);
				}
			}
			pe *= min_s;

			logger().debug() << "P(Equal) = " << pe;

			p *= pe;
		}

		logger().debug() << "iprob(" << oc_to_string(partition) << ") = " << p;

		logger().debug() << "ratio = " << p / pattern_prob;

		return p;
	};

	HandleSeqSeqSeq prtns = partitions_no_set(MinerUtils::get_clauses(pattern));
	std::vector<double> estimates(prtns.size());
	boost::transform(prtns, estimates.begin(), iprob);
	auto p = std::minmax_element(estimates.begin(), estimates.end());
	double emin = *p.first, emax = *p.second;
	logger().debug() << "emin = " << emin << ", emax = " << emax
	                 << ", pattern_prob = " << pattern_prob;

	// Calculate the I-Surprisingness, normalized if requested.
	double dst = dst_from_interval(emin, emax, pattern_prob);
	logger().debug() << "dst = " << dst
	                 << ", normalize = " << normalize
	                 << ", ndst = " << dst / pattern_prob;

	return normalize ? dst / pattern_prob : dst;
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

HandleSeqSeqSeq Surprisingness::partitions_no_set(const HandleSeq& hs)
{
	HandleSeqSeqSeq prtns = partitions(hs);
	// prtns.resize(prtns.size() - 1);
	prtns.resize(1);
	return prtns;
}

Handle Surprisingness::add_pattern(const HandleSeq& block, AtomSpace& as)
{
	return as.add_link(LAMBDA_LINK, as.add_link(AND_LINK, block));
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
