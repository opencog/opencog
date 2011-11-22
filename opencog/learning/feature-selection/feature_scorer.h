/** feature_scorer.h --- 
 *
 * Copyright (C) 2010 OpenCog Foundation
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
 */
template<typename CTable, typename FeatureSet>
struct MutualInformation : public std::unary_function<FeatureSet, double> {

    MutualInformation(const CTable& ctable) 
        : _ctable(ctable) {}

    double operator()(const FeatureSet& features) const {
        return mutualInformation(_ctable, features);
    }

protected:
    const CTable& _ctable;
};

/**
 * Feature set scorer combining Mutual Information and Confidence. The
 * formula is as follows
 *
 * MI(fs) * confidence
 *
 * where confidence = N/(N+confi*|fs|), the confidence of MI (this is
 * a heuristic, in order to measure the true confidence one could
 * compute several MI based on subsamples the dataset and estimate the
 * confidence based on the distribution of MI obtained)
 */
template<typename IT, typename OT, typename FeatureSet>
struct MICScorer : public std::unary_function<FeatureSet, double> {

    MICScorer(const IT& it, const OT& ot,
              double cpi = 1, double confi = 0, double resources = 10000)
        : _it(it), _ot(ot), _confi(confi) {}

    /**
     * The feature set is represented by an eda::instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    double operator()(const FeatureSet& fs) const {
        double MI = mutualInformation(_it, _ot, fs);
        double confidence = _it.size()/(_it.size() + _confi*fs.size());
        return MI * confidence;
    }

    const IT& _it;
    const OT& _ot;
    double _confi; //  confidence intensity
};

/// like above but using Table instead of input and output table
template<typename Table, typename FeatureSet>
struct MICScorerTable : public std::unary_function<FeatureSet, double> {

    MICScorerTable(const Table& table, double confi = 0)
        : _table(table), _ctable(_table.compress()), _confi(confi) {}

    /**
     * The feature set is represented by an eda::instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    double operator()(const FeatureSet& fs) const {
        double MI = mutualInformation(_ctable, fs);
        double confidence = _table.size()/(_table.size() + _confi*fs.size());
        return MI * confidence;
    }

    const Table& _table;
    typename Table::CTable _ctable;
    double _confi; //  confidence intensity
};

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SCORER_H
