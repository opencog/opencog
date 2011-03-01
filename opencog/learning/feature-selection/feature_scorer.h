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

#include <functional>

#include <opencog/util/foreach.h>
#include <opencog/util/numeric.h>

namespace opencog {

/**
 * Computes the conditional entropy
 *
 * H(Y|X1, ..., Xn) = H(Y, X1, ..., Xn) - H(X1, ..., Xn)
 *
 * of the output Y given a set of features X1, ..., Xn. IT (a matrix)
 * is the type of the input table, OT (a vector) is the type of the
 * output table. It returns 1 - H(Y|X1, ..., Xn) to meet the
 * assumption that feature scorers return a value in [0, 1], where 0
 * is the lower score and 1 is the higher score.
 *
 * In the context of one single problem maximizing mutual information
 * amounts to minimizing conditional entropy, as explained below:
 *
 * H(X;Y) = H(X,Y)-H(X|Y) - H(Y|X)
 *        = H(X,Y)-H(X,Y) + H(Y) - H(X,Y) + H(X)
 *        = H(Y) + H(X) - H(X,Y)
 *
 * H(Y|X) = H(Y,X) - H(X)
 *
 * But since H(Y) is constant maximizing H(X;Y) is equivalent to
 * minimizing H(Y|X).
 *
 * However in the context of measuring the usefulness of X across
 * several problems mutual information migh be better that conditional
 * entropy. See MutualInformation below.
 */
template<typename IT, typename OT, typename FeatureSet>
struct ConditionalEntropy : public std::unary_function<FeatureSet, double> {

    ConditionalEntropy(const IT& it, const OT& ot) 
        : _it(it), _ot(ot) {}

    double operator()(const FeatureSet& features) const {
        // compute H(X1, ..., Xn) an H(X1, ..., Xn, Y) at once
        std::pair<double, double> jioh = jointIOTEntropies(_it, _ot, features);
        // return 1 - H(Y|X1, ..., Xn)
        double ce = jioh.second - jioh.first;
        return 1 - ce;
    }

protected:
    const IT& _it;
    const OT& _ot;
};

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
        // compute H(X1, ..., Xn) and H(X1, ..., Xn, Y) at once
        std::pair<double, double> jioh = jointIOTEntropies(_it, _ot, features);
        // compute H(Y)
        double hy = OTEntropy(_ot);
        // return H(Y;X1, ..., Xn)
        return jioh.first + hy - jioh.second;
    }

protected:
    const IT& _it;
    const OT& _ot;
};


} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SCORER_H
