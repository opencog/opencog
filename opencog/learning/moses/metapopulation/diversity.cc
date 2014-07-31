/** diversity.cc ---
 *
 * Copyright (C) 2010 Novemente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Authors: Nil Geisweiller, Moshe Looks, Linas Vepstas
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

#include <math.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#include <opencog/util/oc_omp.h>

#include "metapopulation.h"

namespace opencog {
namespace moses {

using namespace std;
using namespace combo;

void metapopulation::set_diversity()
{
    logger().debug("Compute diversity penalties of the metapopulation");

    scored_combo_tree_ptr_set pool; // new metapopulation

    // structure to remember a partially aggredated distorted
    // diversity penalties between the candidates and the ones in
    // the pool (to avoid recomputing them)
    typedef scored_combo_tree_ptr_set_it psi;
    typedef std::pair<psi, dp_t> bsct_dp_pair;
    std::vector<bsct_dp_pair> tmp;
    psi last = _scored_trees.end();
    for (psi bsct_it = _scored_trees.begin(); bsct_it != last; ++bsct_it)
        tmp.push_back(bsct_dp_pair(bsct_it, 0.0));

    // pointer to the last candidate moved from tmp to pool
    const scored_combo_tree* last_ptr(nullptr);

    // // debug
    // std::atomic<unsigned> dp_count(0); // count the number of
    //                                    // calls of _cached_dst
    // unsigned lhits = _cached_dst.hits.load(),
    //     lmisses = _cached_dst.misses.load();
    // // ~debug

    // if diversity_exponent is negative or null then the generalized
    // mean is replaced by the max
    bool dp_max = _params.diversity.exponent <= 0.0;

    // Update the diversity penalty of the candidate according to its
    // diversity distance to the pool
    auto update_diversity_penalty = [&](bsct_dp_pair& v) {

        if (!pool.empty()) { // only do something if the pool is
                             // not empty (WARNING: this assumes
                             // that all diversity penalties are
                             // initially zero)

            scored_combo_tree& bsct = *v.first;
            OC_ASSERT(bsct.get_bscore().size(),
                      "Behavioral score is needed for diversity!");

            // compute diversity penalty between bs and the last
            // element of the pool
            dp_t last_dst = this->_cached_dst(&bsct, last_ptr);
            OC_ASSERT(last_dst >= 0.0, "The distance cannot be negative."
                      "There could be a bug or you are not using a true distance. "
                      "For instance the Tanimoto distance may be negative "
                      "when some of its components are negative. "
                      "If that is the case you might want to switch to the angular "
                      "distance.");
            dp_t last_dp = _params.diversity.dst2dp(last_dst),
            last_ddp = dp_max ? last_dp : pow(last_dp, _params.diversity.exponent);

            // // debug
            // ++dp_count;
            // // ~debug

            // add it to v.second and compute the aggregated
            // diversity penalty
            dp_t adp;
            if (dp_max) {
                v.second = std::max(v.second, last_ddp);
                adp = v.second;
            } else {
                v.second += last_ddp;
                unsigned N = _params.diversity.normalize ? pool.size() : 1;
                adp = this->aggregated_dps(v.second, N);
            }

            // update v.first
            if (_params.diversity.dst2dp_type == _params.diversity.pthpower)
                bsct.get_composite_score().multiply_diversity = true;
            bsct.get_composite_score().set_diversity_penalty(adp);

            if (logger().isFineEnabled()) {
                stringstream ss;
                ss << "Diversity for candidate: " << bsct.get_tree()
                   << ", last_dst = " << last_dst
                   << ", last_dp = " << last_dp
                   << ", last_ddp = " << last_ddp
                   << ", adp = " << adp;
                logger().fine(ss.str());
            }
        }
    };

    while (tmp.size()) {

        // // debug
        // logger().fine("Pool =");
        // for (scored_combo_tree& pbs_tr : pool) {
        //     stringstream ss;
        //     ostream_scored_combo_tree(ss, pbs_tr, true, true, true);
        //     logger().fine(ss.str());
        // }
        // // ~debug

        // update all diversity penalties of tmp
        OMP_ALGO::for_each(tmp.begin(), tmp.end(), update_diversity_penalty);

        // take the max score, insert in the pool and remove from tmp

        // Define less function to compare bsct_dp_pair; comparison is
        // based on the composite score ...
        sct_score_greater bsct_gt;
        auto gt = [&](const bsct_dp_pair& l, const bsct_dp_pair& r) {
            return bsct_gt(*l.first, *r.first);
        };
        // note although we do use min_element it returns the
        // candidate with the best penalized score because we use
        // a greater_than function instead of a less_than function
        auto mit = OMP_ALGO::min_element(tmp.begin(), tmp.end(), gt);
        // remember the last one added in the pool to calculate its
        // distance with the other in tmp
        last_ptr = &*mit->first;
        // move the last candidate to the pool and remove it from tmp
        pool.transfer(mit->first, _scored_trees);
        tmp.erase(mit);
    }

    // // debug
    // logger().debug() << "Number of dst evals = " << dp_count;
    // logger().debug() << "Is lock free? " << dp_count.is_lock_free();
    // logger().debug() << "Number of hits = "
    //                  << _cached_dst.hits.load() - lhits;
    // logger().debug() << "Number of misses = "
    //                  << _cached_dst.misses.load() - lmisses;
    // logger().debug() << "Total number of hits = " << _cached_dst.hits;
    // logger().debug() << "Total number of misses = " << _cached_dst.misses;
    // // ~debug

    // Replace the existing metapopulation with the new one.
    _scored_trees.swap(pool);

    if (logger().isFineEnabled()) {
        stringstream ss;
        ss << "Metapopulation after setting diversity:" << std::endl;
        ostream_metapop(ss);
        logger().fine() << ss.str();
    }
}


/**
 * Gather statistics about the diversity of the n best candidates
 * (if n is negative then all candidates are included)
 */
metapopulation::diversity_stats
metapopulation::gather_diversity_stats(int n)
{
    if (n < 0)
        return _cached_dst.gather_stats();
    else {
        namespace ba = boost::accumulators;
        typedef ba::accumulator_set<double,
                                 ba::stats<ba::tag::count,
                                           ba::tag::mean,
                                           ba::tag::variance,
                                           ba::tag::min,
                                           ba::tag::max>> accumulator_t;

        // compute the statistics
        accumulator_t acc;
        auto from_i = _scored_trees.cbegin(),
            to = std::next(_scored_trees.cbegin(), std::min(n, (int)size()));
        for (; from_i != to; ++from_i) {
            for (auto from_j = _scored_trees.cbegin(); from_j != from_i; ++from_j) {
#ifdef ENABLE_DST_CACHE
                cached_dst::ptr_pair cts = {&*from_j, &*from_i};
                auto it = _cached_dst.cache.find(cts);
                OC_ASSERT(it != _cached_dst.cache.cend(),
                          "Candidate isn't in the cache that must be a bug");
                acc(it->second);
#else
                acc(_cached_dst(&*from_j, &*from_i));
#endif
            }
        }

        // gather stats
        diversity_stats ds;
        ds.count = ba::count(acc);
        ds.mean = ba::mean(acc);
        ds.std = sqrt(ba::variance(acc));
        ds.min = ba::min(acc);
        ds.max = ba::max(acc);

        return ds;
    }
}

diversity_parameters::dp_t
metapopulation::cached_dst::operator()(const scored_combo_tree* cl,
                                       const scored_combo_tree* cr)
{
#ifdef ENABLE_DST_CACHE
    ptr_pair cts = {cl, cr};
    // hit
    {
        shared_lock lock(mutex);
        auto it = cache.find(cts);
        if (it != cache.end()) {
            ++hits;
            return it->second;
        }
    }
    // miss
    dp_t dst = _dparams.dst(cl->get_bscore(), cr->get_bscore());

    // // debug
    // logger().fine("&cl = %p, &cr = %p, dst = %f", cl, cr, dst);
    // // ~debug

    ++misses;
    {
        unique_lock lock(mutex);
        return cache[cts] = dst;
    }
#else
    return _dparams.dst(cl->get_bscore(), cr->get_bscore());
#endif
}

/**
 * Remove all keys containing any element of ptr_seq
 */
void metapopulation::cached_dst::erase_ptr_seq(std::vector<scored_combo_tree*> ptr_seq)
{
#ifdef ENABLE_DST_CACHE
    for (Cache::iterator it = cache.begin(); it != cache.end();) {
        if (!is_disjoint(ptr_seq, it->first))
            it = cache.erase(it);
        else
            ++it;
    }
#endif
}

/**
 * Gather some statistics about the diversity of the
 * population, such as mean, std, min, max of the distances.
 */
metapopulation::diversity_stats
metapopulation::cached_dst::gather_stats() const
{
    namespace ba = boost::accumulators;
    typedef ba::accumulator_set<double,
                                ba::stats<ba::tag::count,
                                          ba::tag::mean,
                                          ba::tag::variance,
                                          ba::tag::min,
                                          ba::tag::max>> accumulator_t;

    // compute the statistics
    accumulator_t acc;
    for (const auto& v : cache) acc(v.second);

    // gather stats
    diversity_stats ds;
    ds.count = ba::count(acc);
    ds.mean = ba::mean(acc);
    ds.std = sqrt(ba::variance(acc));
    ds.min = ba::min(acc);
    ds.max = ba::max(acc);

    return ds;
}


} // ~namespace moses
} // ~namespace opencog

