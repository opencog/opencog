/*
 * opencog/learning/moses/scoring/scoring_base.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012,2013 Poulin Holdings LLC
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Moshe Looks, Nil Geisweiller, Linas Vepstas
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

#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/irange.hpp>

#include <opencog/comboreduct/table/table_io.h>
#include <opencog/util/oc_assert.h>
#include "scoring_base.h"

namespace opencog { namespace moses {

// Note that this function returns a POSITIVE number, since p < 0.5
score_t discrete_complexity_coef(unsigned alphabet_size, double p)
{
    return -log((double)alphabet_size) / log(p/(1-p));
}

        // Note that this returns a POSITIVE number.
score_t contin_complexity_coef(unsigned alphabet_size, double stdev)
{
    return log(alphabet_size) * 2 * sq(stdev);
}

void bscore_base::set_complexity_coef(unsigned alphabet_size, float p)
{
    // Both p==0.0 and p==0.5 are singularities in the forumla.
    // See the explanation in the comment above ctruth_table_bscore.
    _complexity_coef = 0.0;
    if (p > 0.0f && p < 0.5f)
        _complexity_coef = discrete_complexity_coef(alphabet_size, p);

    logger().info() << "BScore noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/_complexity_coef;
}

void bscore_base::set_complexity_coef(score_t complexity_ratio)
{
    _complexity_coef = 0.0;
    if (complexity_ratio > 0.0)
        _complexity_coef = 1.0 / complexity_ratio;

    logger().info() << "BScore complexity ratio = " << 1.0/_complexity_coef;
}

behavioral_score
bscore_base::operator()(const scored_combo_tree_set& ensemble) const
{
    OC_ASSERT(false, "Ensemble scoring not implemented for bscorer %s",
        typeid(*this).name());
    return behavioral_score();
}

behavioral_score
bscore_base::worst_possible_bscore() const
{
    // Can't assert; this will fail during ensemble setup.
    // OC_ASSERT(false, "Worst possible score not implemented for bscorer %s",
    //     typeid(*this).name());
    return behavioral_score();
}

/**
 * Compute the average (weighted) complexity of all the trees in the
 * ensemble.  XXX this is probably wrong, we should probably do something
 * like add the logarithm of the number of trees to the complexity, or 
 * I dunno .. something.  Unclear how the theory should even work for this
 * case.
 */
complexity_t bscore_base::get_complexity(const scored_combo_tree_set& ensemble) const
{
    if (0 == ensemble.size()) return 0.0;

    double cpxy = 0.0;
    double norm = 0.0;
    for (const scored_combo_tree& sct : ensemble) {
        double w = sct.get_weight();
        cpxy += w * tree_complexity(sct.get_tree());
        norm += w;
    }

    // XXX FIXME complexity_t should be a double not an int ...
    return (complexity_t) floor (cpxy / norm + 0.5);
}

score_t
bscore_base::get_error(const behavioral_score&) const
{
    OC_ASSERT(false, "score error not implemented for bscorer %s",
        typeid(*this).name());
    return -1.0;
}

score_t bscore_base::sum_bscore(const behavioral_score& bs) const
{
    // Don't use weights if not boosting.
    if (not _return_weighted_score or _size == 0 or _weights.size() == 0)
        return boost::accumulate(bs, 0.0);

    size_t i=0;
    score_t res = 0.0;
    for (; i<_size; i++) {
        res += _weights[i] * bs[i];
    }

    // Any extra penalties tacked onto the end of the bscore get added
    // without any weights.  For example, the "pre" scoer tacks these
    // on, so that the minimum activation can be hit.
    for (; i<bs.size(); i++) {
        res += bs[i];
    }
    return res;
}

void bscore_base::reset_weights()
{
    if (_return_weighted_score)
        _weights = std::vector<double>(_size, 1.0);
    else
        _weights = std::vector<double>();
}


////////////////////////
// bscore_ctable_base //
////////////////////////

bscore_ctable_base::bscore_ctable_base(const CTable& ctable)
    : _orig_ctable(ctable), _wrk_ctable(_orig_ctable),
      _all_rows_wrk_ctable(_wrk_ctable),
      _ctable_usize(_orig_ctable.uncompressed_size())
{
    _size = ctable.size();
    recompute_weight();
}

void bscore_ctable_base::recompute_weight() const
{
    // Sum of all of the weights in the working table.
    // Note that these weights can be fractional!
    _ctable_weight = 0.0;
    for (const CTable::value_type& vct : _wrk_ctable) {
        const CTable::counter_t& cnt = vct.second;

        _ctable_weight += cnt.total_count();
    }
}

void bscore_ctable_base::ignore_cols(const std::set<arity_t>& idxs) const
{
    if (logger().isDebugEnabled())
    {
        std::stringstream ss;
        ss << "Compress CTable for optimization by ignoring features: ";
        ostreamContainer(ss, idxs, ",");
        logger().debug(ss.str());
    }

    // Get permitted idxs.
    auto irng = boost::irange(0, _orig_ctable.get_arity());
    std::set<arity_t> all_idxs(irng.begin(), irng.end());
    std::set<arity_t> permitted_idxs = opencog::set_difference(all_idxs, idxs);

    // Filter orig_table with permitted idxs.
    _wrk_ctable = _orig_ctable.filtered_preserve_idxs(permitted_idxs);
    recompute_weight();

    // for debugging, keep that around till we fix best_possible_bscore
    // fully_filtered_ctable = _orig_ctable.filtered(permitted_idxs);

    logger().debug("Original CTable size = %u", _orig_ctable.size());
    logger().debug("Working CTable size = %u", _wrk_ctable.size());

    if (logger().isFineEnabled()) {
        std::stringstream ss;
        ss << "_wrk_ctable =" << std::endl;
        ostreamCTable(ss, _wrk_ctable);
        logger().fine(ss.str());
        // for debugging, keep that around till we fix best_possible_bscore
        //     std::stringstream ss2;
        //     ss2 << "fully_filtered_ctable =" << std::endl;
        //     ostreamCTable(ss2, fully_filtered_ctable);
        //     logger().fine(ss2.str());
    }

    // Copy the working ctable in a temporary ctable that keeps track
    // of all rows (so ignore_rows can be applied several times)
    _all_rows_wrk_ctable = _wrk_ctable;
}

void bscore_ctable_base::ignore_rows(const std::set<unsigned>& idxs) const
{
    _wrk_ctable = _all_rows_wrk_ctable; // to include all rows in _wrk_ctable

    // if (logger().isFineEnabled())
    //     logger().fine() << "Remove " << idxs.size() << " uncompressed rows from "
    //                     << "_wrk_ctable of compressed size " << _wrk_ctable.size()
    //                     << ", uncompressed size = " << _wrk_ctable.uncompressed_size();
    _wrk_ctable.remove_rows(idxs);
    _ctable_usize = _wrk_ctable.uncompressed_size();
    _size = _wrk_ctable.size();
    recompute_weight();

    // if (logger().isFineEnabled())
    //     logger().fine() << "New _wrk_ctable compressed size = " << _wrk_ctable.size()
    //                     << ", uncompressed size = " << _ctable_usize;
}

void bscore_ctable_base::ignore_rows_at_times(const std::set<TTable::value_type>& timestamps) const
{
    // logger().fine() << "bscore_ctable_base::ignore_rows_at_times";
    // ostreamContainer(logger().fine() << "timestamps = ", timestamps);

    _wrk_ctable = _all_rows_wrk_ctable; // to include all rows in _wrk_ctable

    // if (logger().isFineEnabled())
    //     logger().fine() << "Remove " << timestamps.size() << " dates from "
    //                     << "_wrk_ctable of compressed size " << _wrk_ctable.size()
    //                     << ", uncompressed size = " << _wrk_ctable.uncompressed_size();

    _wrk_ctable.remove_rows_at_times(timestamps);
    _ctable_usize = _wrk_ctable.uncompressed_size();
    _size = _wrk_ctable.size();
    recompute_weight();

    // if (logger().isFineEnabled())
    //     logger().fine() << "New _wrk_ctable compressed size = " << _wrk_ctable.size()
    //                     << ", uncompressed size = " << _ctable_usize;

}

unsigned bscore_ctable_base::get_ctable_usize() const
{
    return _ctable_usize;
}

const CTable& bscore_ctable_base::get_ctable() const
{
    return _orig_ctable;
}

} // ~namespace moses
} // ~namespace opencog
