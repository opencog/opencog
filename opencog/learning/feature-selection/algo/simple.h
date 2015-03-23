/** 
 * simple.h ---
 *
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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


#ifndef _OPENCOG_FEATURE_SELECTION_SIMPLE_ALGO_H
#define _OPENCOG_FEATURE_SELECTION_SIMPLE_ALGO_H

#include <mutex>

#include <opencog/util/numeric.h>
#include <opencog/util/oc_omp.h>

#include "../main/feature-selection.h"  // needed for feature_set, feature_selection_parameters

namespace opencog {

feature_set_pop simple_select_feature_sets(const CTable& ctable,
                                           const feature_selection_parameters& fs_params);

// Return whether the scored feature set x is greater than y.
// Important note: This attempts to randomize the comparison, when the
// scores are equal.  This is critical for a good mix of training results.
// Second important note: this assumes that the feature sets are singletons.
// This is to provide a minor performance boost.
template<typename FeatureSet>
struct ScoredFeatureSetGreater
{
    typedef std::pair<double, FeatureSet> ScoredFeatureSet;
    ScoredFeatureSetGreater()
    {
        seed = randGen().randint();
    }

    bool operator()(const ScoredFeatureSet& x, const ScoredFeatureSet& y) const
    {
        if (x.first > y.first) return true;
        if (x.first < y.first) return false;

        // The scored feature sets are *always* singletons! 
        // viz FeatureSet will always be std::set<arity_t> with
        // only one element in the set. So no loop required!
        arity_t ix = *x.second.begin();
        arity_t ox = ix ^ seed;

        arity_t iy = *y.second.begin();
        arity_t oy = iy ^ seed;
        return ox > oy;
    }

    typedef ScoredFeatureSet first_argument_type;
    typedef ScoredFeatureSet second_argument_type;
    typedef bool result_type;
private:
    arity_t seed;
};

/**
 * Returns a set S of features following the algo:
 * 1) Compute the pair-wise mutual information between the target and 
 *    each feature in the input set.
 * 2) If return the highest num_desired of these.  If use_exp_distrib
 *    is true, then a random exponential distribution is used to pick
 *    the highest of these; otherwise only the very best as choosen.
 *    The exponential distrib is nice, as it rounds down the sharp
 *    cornder that the sharp cutoff otherise imposes.
 *
 * Clearly, this is a very simple, very fast algrotihm.
 *
 * @param features       The initial set of features to be selected from
 * @param scorer         The function to score a set of features.
 * @param threshold      The threshold to select a set of feature
 *
 * @return               The set of selected features
 */
template<typename Scorer, typename FeatureSet>
FeatureSet simple_selection(const FeatureSet& features,
                            const Scorer& scorer,
                            size_t num_desired,
                            bool use_exp_distrib,
                            double threshold)
{
    typedef std::pair<double, FeatureSet> ScoredFeatureSet;
    // std::greater<>: First, sort by score, then sort by lexicographic order.
    // std::set<ScoredFeatureSet, std::greater<ScoredFeatureSet> > sorted_flist;
    std::set<ScoredFeatureSet, ScoredFeatureSetGreater<FeatureSet> > sorted_flist;

    // Build vector of singleton feature sets.
    std::vector<FeatureSet> singletons; 
    for (auto feat : features)
        singletons.push_back(FeatureSet({feat}));

    // Compute score of all singletons and insert to sorted_flist
    // those above threshold.  If not exponentially distributed, then
    // we don't have to sort all of them; we only have to
    // sort the top num_desired of these.  i.e. just push_back the
    // the scored features onto std::vector and then use
    // std::partial_sort to extract the top scores.  This could improve
    // performance... TODO try this, if this is actually a bottleneck.
    std::mutex sfl_mutex;       // mutex for sorted_flist
    OMP_ALGO::for_each(singletons.begin(), singletons.end(),
                       [&](const FeatureSet& singleton) {
                           double sc = scorer(singleton);
                           if (threshold <= sc) {
                               std::unique_lock<std::mutex> lock(sfl_mutex);
                               sorted_flist.insert({sc, singleton});
                           }
                       });

    // Select num_desired best features from sorted_flist as final
    // feature set. 
    FeatureSet final;
    if (use_exp_distrib)
    {
        // Exponential distribution with mean of num_desired features.
        // Actually, we cheat slightly, and ask for more, so that we
        // don't get too few...
        double x = 1.0 - 1.0 / ((double) num_desired + 1);
        double xn = 1.0;
        for (auto pr = sorted_flist.begin(); pr != sorted_flist.end(); pr++) {
            if (randGen().randdouble() < xn)
            {
                final.insert(*pr->second.begin());
                num_desired --;
                if (num_desired <= 0) break;
            }
            xn *= x;
        }
    } else {
        // stair-step distribution: keep the top num_desired only.
        //  XXX or use partial_sort, as mentioned above...
        for (auto pr = sorted_flist.begin(); pr != sorted_flist.end(); pr++) {
            final.insert(*pr->second.begin());
            num_desired --;
            if (num_desired <= 0) break;
        }
    }

    return final;
}


} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTION_SIMPLE_ALGO_H
