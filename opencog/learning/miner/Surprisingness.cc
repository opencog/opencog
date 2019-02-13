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

double Surprisingness::ISurprisingness(const Handle& pattern,
                                       const HandleSeqSeq& partitions,
                                       const HandleSet& texts,
                                       bool normalize)
{
	// Function calculating the probability of a pattern
	auto prob = [&](const Handle& pattern) {
		double ucount = pow((double)texts.size(), MinerUtils::n_conjuncts(pattern));
		double sup = MinerUtils::calc_support(pattern, texts, (unsigned)ucount);
		return sup / ucount;
	};

	// Calculate the probability of pattern
	double pattern_prob = prob(pattern);

	// Calculate the probability estimate of each partition based on
	// independent assumption of between each partition block, and
	// derive the min and max probability estimates.
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
	// independent assumption of between each partition block, and
	// derive the min and max probability estimates.
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

} // namespace opencog
