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



} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SCORER_H
