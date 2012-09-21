/** moses_based_scorer.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_MOSES_BASED_SCORER_H
#define _OPENCOG_MOSES_BASED_SCORER_H

#include <opencog/util/numeric.h>

#include <opencog/learning/moses/representation/field_set.h>
#include <opencog/learning/moses/eda/eda.h>
#include <opencog/learning/moses/moses/types.h>
#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/comboreduct/combo/common_def.h>

namespace opencog {

using namespace moses;
using namespace combo;

/**
 * translate an instance into a feature set. Each feature is
 * represented by its index (the left most one is 0).
 */
std::set<arity_t> get_feature_set(const field_set& fields,
                                  const instance& inst);

/**
 * Wrapper to use moses scoring precision (see
 * opencog/learning/moses/moses/scoring.h).
 *
 * That wrapper uses the method best_possible_score() given a certain
 * feature set. And therefore attempts to maximize the best possible
 * score one would get (w.r.t. some fitness function) given the
 * feature set being evaluated.
 */
template<typename FeatureSet>
struct pre_scorer : public unary_function<FeatureSet, double> {
    pre_scorer(const CTable& ctable,
               float penalty = 1.0f,
               float min_activation = 0.5f,
               float max_activation = 1.0f,
               bool positive = true)
        : _ctable(ctable), _penalty(penalty),
          _min_activation(min_activation), _max_activation(max_activation),
          _positive(positive) {}

    double operator()(const FeatureSet& fs) const {
        // filter the ctable
        CTable filtered_ctable = _ctable.filtered(fs);
        // create the scorer
        precision_bscore sc(filtered_ctable, _penalty,
                            _min_activation, _max_activation, _positive);
        return boost::accumulate(sc.best_possible_bscore(), 0.0);
    }
protected:
    const CTable& _ctable;
    float _penalty, _min_activation, _max_activation;
    bool _positive;
};

/**
 * Wrapper to use a feature set scorer with MOSES's optimization
 * algorithms operating on a deme. Each deme is a binary string where
 * each bit represents whether a feature is selected or not.
 */
template<typename FSScorer>
struct deme_based_scorer : public iscorer_base
{
    deme_based_scorer(const FSScorer& fs_scorer, const field_set& fields)
        : _fs_scorer(fs_scorer), _fields(fields) {}

    /**
     * The feature set is represented by an instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    composite_score operator()(const instance& inst) const
    {
        std::set<arity_t> fs = get_feature_set(_fields, inst);
        composite_score csc(_fs_scorer(fs), fs.size(), 0);
        // Logger
        if (logger().isFineEnabled()) {
            logger().fine()
               << "moses_based_scorer - Evaluate instance: " 
               << _fields.stream(inst) << " " << csc;
        }
        // ~Logger
        return csc;
    }

    const FSScorer& _fs_scorer;
    const field_set& _fields;
};

} // ~namespace opencog

#endif // _OPENCOG_MOSES_BASED_SCORER_H
