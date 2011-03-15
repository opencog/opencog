/** feature_scorer.h --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
 *
 * Author: Nil Geisweiller <nilg@laptop>
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


#ifndef _OPENCOG_FEATURE_SCORER_H
#define _OPENCOG_FEATURE_SCORER_H

#include <opencog/util/numeric.h>

namespace opencog {

/**
 * Compute the MutualInformation
 *
 * H(Y;X1, ..., Xn) = H(X1, ..., Xn) + H(Y) - H(X1, ..., Xn, Y)
 *
 * Slightly more expensive than Conditional Entropy but more relevant
 * across problems.
 */
template<typename IT, typename OT, typename FeatureSet>
struct MutualInformation : public std::unary_function<FeatureSet, double> {

    MutualInformation(const IT& it, const OT& ot) 
        : _it(it), _ot(ot) {}

    double operator()(const FeatureSet& features) const {
        return mutualInformation(_it, _ot, features);
    }

protected:
    const IT& _it;
    const OT& _ot;
};

/**
 * Feature set scorer; Mutual Information, Confidence and Speed
 * prior. The formula is as follows
 *
 * MI(fs) * confidence * speedPrior
 *
 * where confidence = N/(N+confi*|fs|), the confidence of MI (this is
 * a heuristic, in order to measure the true confidence one could
 * compute several MI based on a subsample the dataset and estimate
 * the confidence based on the distribution of MI obtained)
 *
 * speedPrior = max(1, R/exp(cpi*|fs|)), a larger feature means more
 * computational power for the learning algo. So even if the
 * confidence is quite high (because the number of samples in the data
 * set is high) we still don't want to bias the search toward small
 * feature sets.
 */
template<typename IT, typename OT, typename FeatureSet>
struct MICSScorer : public std::unary_function<FeatureSet, double> {

    MICSScorer(const IT& it, const OT& ot,
               double cpi = 1, double confi = 0, double resources = 10000)
        : _it(it), _ot(ot), _cpi(cpi), _confi(confi), _resources(resources) {}

    /**
     * The feature set is represented by an eda::instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    double operator()(const FeatureSet& fs) const {
        double MI = mutualInformation(_it, _ot, fs);
        double confidence = _it.size()/(_it.size() + _confi*fs.size());
        double speedPrior = std::min(1.0, _resources/exp(_cpi*fs.size()));
        return MI * confidence * speedPrior;
    }

    const IT& _it;
    const OT& _ot;
    double _cpi; // complexity penalty intensity
    double _confi; //  confidence intensity
    double _resources; // resources of the learning algo that will take
                       // in input the feature set
};

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SCORER_H
