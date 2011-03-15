/** moses_based_scorer.h --- 
 *
 * Copyright (C) 2011 Nil Geisweiller
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

#include <opencog/learning/moses/eda/field_set.h>
#include <opencog/learning/moses/eda/eda.h>
#include <opencog/learning/moses/moses/types.h>

namespace opencog {

using namespace moses;

/**
 * translate an instance into a feature set.
 */
std::set<arity_t> get_feature_set(const eda::field_set& fields,
                                  const eda::instance& inst) {
    std::set<arity_t> fs;
    eda::field_set::const_bit_iterator bit = fields.begin_bits(inst);
    for(arity_t i = 0; bit != fields.end_bits(inst); bit++, i++)
        if(*bit)
            fs.insert(i);
    return fs;
}

/**
 * Scorer to be used with MOSES' optimizers for feature set combining
 * Mutual Information, Confidence and Speed prior. The formula is as
 * follows
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
template<typename IT, typename OT>
struct MIORScorer : public unary_function<eda::instance, composite_score> {

    MIORScorer(const IT& _it, const OT& _ot, const eda::field_set& _fields,
               double _cpi = 1, double _confi = 0, double _resources = 10000)
        : it(_it), ot(_ot), fields(_fields), cpi(_cpi),
          confi(_confi), resources(_resources) {}

    /**
     * The feature set is represented by an eda::instance encoding a
     * field of booleans. Each boolean represents whether its
     * corresponding feature is in the feature set of not.
     */
    composite_score operator()(const eda::instance& inst) const {
        std::set<arity_t> fs = get_feature_set(fields, inst);
        complexity_t c = fields.count(inst);
        double MI = mutualInformation(it, ot, fs);
        double confidence = it.size()/(it.size() + confi*c);
        double speedPrior = std::min(1.0, resources/exp(cpi*c));
        composite_score csc(MI * confidence * speedPrior, c);
        // Logger
        if(logger().getLevel() >= Logger::FINE) {
            stringstream ss;
            ss << "MIORScorer - Evaluate instance: " 
               << fields.stream(inst) << " " << csc
               << ", confidence = " << confidence
               << ", speedPrior = " << speedPrior << std::endl;
            logger().fine(ss.str());
        }
        // ~Logger
        return csc;
    }

    const IT& it;
    const OT& ot;
    const eda::field_set& fields;
    double cpi; // complexity penalty intensity
    double confi; //  confidence intensity
    double resources; // resources of the learning algo that will take
                      // in input the feature set
};

} // ~namespace opencog

#endif // _OPENCOG_MOSES_BASED_SCORER_H
