/*
 * opencog/learning/moses/scoring/ss_bscore.cc
 *
 * Copyright (C) 2014 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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

#include "ss_bscore.h"

#include <boost/range/irange.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#include <opencog/util/random.h>

namespace opencog { namespace moses {

///////////////
// ss_bscore //
///////////////


ss_bscore::ss_bscore(const bscore_base& bscorer, unsigned n_subsamples,
                     float low_dev_pressure, bool by_time)
    : _bscorer(bscorer), _n_subsamples(n_subsamples),
      _low_dev_pressure(low_dev_pressure), _by_time(by_time) {
    _timestamps = _bscorer.get_ctable().get_timestamps();
}

behavioral_score ss_bscore::operator()(const combo_tree& tr) const {
    if (_n_subsamples == 0 or _low_dev_pressure == 0.0)
        return _bscorer(tr);

    using namespace boost::accumulators;
    typedef accumulator_set<float,
                            stats<boost::accumulators::tag::count,
                                  boost::accumulators::tag::mean,
                                  boost::accumulators::tag::variance>> stat_acc_t;
    stat_acc_t sc_var_acc;

    if (_by_time) {
        // For each subsample-deme hold set of timestamps to remove
        std::vector<std::set<TTable::value_type>> ignore_timestamps_per_ss_deme(_n_subsamples);

        std::set<TTable::value_type> timestamps(_timestamps);
        unsigned tt_size = timestamps.size();

        // // Debug
        // ostreamContainer(logger().fine() << "timestamps = ", timestamps);
        // logger().fine() << "tt_size = " << tt_size;
        // // ~Debug

        for (unsigned i = 0; i < tt_size; ++i) {
            TTable::value_type time = randset_erase(timestamps);
            ignore_timestamps_per_ss_deme[i % _n_subsamples].insert(time);
        }

        // Compute the score for all subsamples and compute the variance
        for (unsigned i = 0; i < _n_subsamples; ++i) {
            _bscorer.ignore_rows_at_times(ignore_timestamps_per_ss_deme[i]);
            behavioral_score psc = _bscorer(tr);
            sc_var_acc(boost::accumulate(psc, 0.0));
        }        
    } else {
        // For each fitness hold set of row indexes to remove
        std::vector<std::set<unsigned>> ignore_row_idxs_per_fitness;

        unsigned usize = _bscorer.get_ctable_usize();
        // Create a vector [0, usize)
        auto rng = boost::irange(0U, usize);
        std::vector<unsigned> row_idxs(rng.begin(), rng.end());

        // Randomly shuffle the order
        auto shr = [&](ptrdiff_t i) { return randGen().randint(i); };
        random_shuffle(row_idxs.begin(), row_idxs.end(), shr);

        // Divide the vector into n_ss_demes equal parts
        unsigned seg_size = usize / _n_subsamples;

        for (auto it = row_idxs.cbegin();
             it != row_idxs.cbegin() + _n_subsamples * seg_size;
             it += seg_size)
            ignore_row_idxs_per_fitness.emplace_back(it, it + seg_size);

        // Compute the score for all subsamples and compute the variance
        for (unsigned i = 0; i < _n_subsamples; ++i) {
            _bscorer.ignore_rows(ignore_row_idxs_per_fitness[i]);
            behavioral_score psc = _bscorer(tr);
            sc_var_acc(boost::accumulate(psc, 0.0));
        }
    }

    // Recompute the bscore over all data points
    logger().fine() << "Recompute over all data points";
    _bscorer.ignore_rows(std::set<unsigned>());
    behavioral_score psc = _bscorer(tr);

    // Compute the low deviation penalty. No variance corresponds to
    // no penalty, the penalty grows with the variance
    score_t sc_var = boost::accumulators::variance(sc_var_acc),
        low_dev_penalty = - _low_dev_pressure * sqrt(sc_var);

    logger().fine() << "sc_var = " << sc_var
                    << ", low_dev_penalty = " << low_dev_penalty;

    // Add the low variance penalty
    psc.push_back(low_dev_penalty);
    return psc;
}

behavioral_score ss_bscore::best_possible_bscore() const {
    return _bscorer.best_possible_bscore();
}

score_t ss_bscore::min_improv() const {
    return _bscorer.min_improv();
}

void ss_bscore::ignore_cols(const std::set<arity_t>& idxs) const {
    _bscorer.ignore_cols(idxs);
}

void ss_bscore::ignore_rows(const std::set<unsigned>&) const {
    OC_ASSERT(false, "Not implemented yet");
}

void ss_bscore::ignore_rows_at_times(const std::set<TTable::value_type>&) const {
    OC_ASSERT(false, "Not implemented yet");
}

unsigned ss_bscore::get_ctable_usize() const {
    return _bscorer.get_ctable_usize();
}

const CTable& ss_bscore::get_ctable() const {
    return _bscorer.get_ctable();
}

} // ~namespace moses
} // ~namespace opencog
