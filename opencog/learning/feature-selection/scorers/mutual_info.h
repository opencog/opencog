/** scorers/mutual_info.h ---
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


#ifndef _OPENCOG_FEATURE_SCORERS_MI_H
#define _OPENCOG_FEATURE_SCORERS_MI_H

#include <opencog/util/numeric.h>
#include <opencog/comboreduct/table/table.h>
#include "fs_scorer_base.h"

namespace opencog {

using namespace combo;

/**
 * Compute the mutual information MI(Y;X1, ..., Xn)
 *
 * where
 *    MI(Y;X1, ..., Xn) = H(X1, ..., Xn) + H(Y) - H(X1, ..., Xn, Y)
 *
 * and H(...) are the marginal entropies.
 *
 * The actual work is done in the combo table code.
 */
template<typename FeatureSet>
struct MutualInformation : public std::unary_function<FeatureSet, double>
{

    MutualInformation(const CTable& ctable)
        : _ctable(ctable) {}

    double operator()(const FeatureSet& features) const
    {
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
 * where confidence = N / (N + exp(-confi*|fs|)), the confidence of MI.
 * This is a heuristic.  In order to measure the true confidence, one could
 * compute several MI based on subsamples the dataset and estimate the
 * confidence based on the distribution of MI obtained.  (???)
 *
 * (???) I don't get it, for two reasons: 1) feature selection works
 * independently of the overall scale of teh scorers; its only the relative
 * rankings that matter.  2) In what sense is this a "confidence"?  That is,
 * if we look at the MI of a subset of the dataset, we expect the distribution
 * to be identical to the MI, measured on the whole set.  That is, the MI
 * is independent of the sample size.  (or its supposed to be, perhaps there
 * is a bug in our MI code???)
 */
template<typename FeatureSet>
struct MICScorer : public std::unary_function<FeatureSet, double>
{
    MICScorer(const ITable& it, const OTable& ot,
              double confi = 100)
        : _it(it), _ot(ot), _confi(confi) {}

    /**
     * The feature set is represented by an eda::instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    double operator()(const FeatureSet& fs) const
    {
        double MI = mutualInformation(_it, _ot, fs);
        // double confidence = _it.size()/(_it.size() + _confi*fs.size());
        double confidence = _it.size()/(_it.size() + exp(-_confi*fs.size()));
        return MI * confidence;
    }

    const ITable& _it;
    const OTable& _ot;
    double _confi; //  confidence intensity
};

/// like above but using CTable instead of input and output table
template<typename FeatureSet>
struct MICScorerCTable : public fs_scorer_base<FeatureSet>
{
    typedef fs_scorer_base<FeatureSet> super;

    // ctor
    MICScorerCTable(const CTable& ctable, double confi = 100)
        : super(ctable, confi) {}

    /**
     * The feature set is represented by an eda::instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    double operator()(const FeatureSet& fs) const
    {
        double MI = mutualInformation(super::_ctable, fs);
        // double confidence = usize / (usize + confi*fs.size());
        double confidence = super::confidence(fs.size());
        logger().fine("MICScorerCTable MI = %g, confidence = %g",
                      MI, confidence);
        return MI * confidence;
    }
};

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SCORERS_MI_H
