/*
 * opencog/learning/moses/scoring/scoring_base.h
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
#ifndef _MOSES_SCORING_BASE_H
#define _MOSES_SCORING_BASE_H

#include "../moses/types.h"
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/table/table.h>

namespace opencog { namespace moses {

using combo::combo_tree;
using combo::arity_t;
using combo::CTable;
using combo::TTable;

/// Used to define the complexity scoring component given that p is the
/// probability of having an observation being wrong (see the comment
/// regarding ctruth_table_bscore for more information).
///
/// Note: this returns NEGATIVE values.
score_t discrete_complexity_coef(unsigned alphabet_size, double p);

/// Used to define the complexity scoring component given that stdev is
/// the standard deviation of the noise of the we're trying to predict
/// output (see the comment regarding contin_bscore for more information).
///
/// Note: this returns NEGATIVE values.
score_t contin_complexity_coef(unsigned alphabet_size, double stdev);

/// Abstract base class for behavioral scoring.
/// A behavioral score is a vector of scores, one per sample of a dataset.
struct bscore_base : public std::unary_function<combo_tree, behavioral_score>
{
    bscore_base() : _complexity_coef(0.0), _size(0) {};
    virtual ~bscore_base() {};

    /// Return the behavioral score for the combo_tree
    virtual behavioral_score operator()(const combo_tree&) const = 0;

    /// Return the behavioral score for the ensemble
    virtual behavioral_score operator()(const scored_combo_tree_set&) const;

    /// Return the size (length) of the behavioral_score that operator()
    /// above would return.
    virtual size_t size() const { return _size; }

    /// Return the best possible bscore achievable with this fitness
    /// function. This is useful for stopping MOSES when the best
    /// possible score has been reached.
    virtual behavioral_score best_possible_bscore() const = 0;

    /// Return the smallest change in the score which can be considered
    /// to be an improvement over the previous score. This is useful for
    /// avoiding local maxima which have a very flat top. That is, where
    /// all combo trees in the same local maximum have almost exactly
    /// the same score, and so scores improve by very small amounts
    /// during the search.  In such cases, one can save a lot of CPU
    /// time by terminating the search when the imrpovements are smaller
    /// than the min_improv(). Returns 0.0 by default.
    virtual score_t min_improv() const { return 0.0; }

    /// Indicate a set of features that should be ignored during scoring,
    /// The features are indicated as indexes, starting from 0.
    ///
    /// The primary intended use of this function is to improve
    /// performance by avoiding evaluation of the ignored features.
    /// At this time, the only users of this method are the table-based
    /// scorers.  By ignoring most columns, the table can typically be
    /// significantly compressed, thus reducing evaluation time.
    ///
    /// It is important that the combo trees to be scored do not use
    /// any of the ignored indexes, as otherwise, a faulty scoring will
    /// result.  Thus, the typical use case is to remove all columns
    /// that do not appear in a knob-decorated combo tree.  The resulting
    /// table is then safe to use during instance scoring, because no
    /// instance could ever reference one of the ignored columns.
    ///
    /// Note that the best_possible_score may depend on the set of
    /// ignored features. Thus, the best_possible_score() method should
    /// be called only after this method.
    ///
    /// This method may be called multiple times; with each call, the
    /// previously-ignored features will first be restored, before the
    /// new index set is ignored.  Thus, calling this with the empty set
    /// will have the effect of restoring all columns that were previously
    /// ignored.
    virtual void ignore_cols(const std::set<arity_t>&) const {}

    /// In case one wants to evaluate the fitness on a subset of the
    /// data, one can provide a set of row indexes to ignore.
    ///
    /// This method may be called multiple times. With each call, the
    /// previously ignored rows will be restored, before the
    /// newly-specified rows are removed.  Thus, calling this with the
    /// empty set has the effect of restoring all ignored rows.
    virtual void ignore_rows(const std::set<unsigned>&) const {}

    // Like ignore_rows but consider timestamps instead of indexes
    virtual void ignore_rows_at_times(const std::set<TTable::value_type>&) const {}

    // Return the uncompressed size of the CTable
    virtual unsigned get_ctable_usize() const {
        OC_ASSERT(false, "You must implement me in the derived class");
        return 0U;
    }

    // Return the original CTable
    virtual const CTable& get_ctable() const {
        static const CTable empty_ctable;
        OC_ASSERT(false, "You must implement me in the derived class");
        return empty_ctable;
    }

    /// Get the appropriate complexity measure for the indicated combo
    /// tree. By default, this is the tree complexity, although it may
    /// depend on the scorer.
    virtual complexity_t get_complexity(const combo_tree& tr) const
    {
        return tree_complexity(tr);
    }
    virtual complexity_t get_complexity(const scored_combo_tree_set&) const;

    /// Return the complexity coefficient.  This is used to obtain the
    /// complexity penalty for the score, which is meant to be computed
    /// as penalty = get_complexity_coef() * get_complexity(tree);
    /// This is done in two steps like this, because different scorers
    /// use a different scale, and so the complexity needs to be rescaled.
    /// Furthermore, different scorers also have a different notion of
    /// complexity. This is the opportunity to make adjustments for each
    /// case.
    virtual score_t get_complexity_coef() const { return _complexity_coef; }

    /// Store a complexity coefficient with the scorerer.  This is
    /// done to work around the fact that different kinds of scorers
    /// normalize their scores in different ways, and so the way that
    /// the penalties are scaled should differ as well.
    /// Strictly speaking, the user could just specify a different
    /// scale on the moses command line, but perhaps this inflicts too
    /// much effort on the user.  Thus, we maintain a "suggested" scaling
    /// here.
    virtual void set_complexity_coef(score_t complexity_ratio);
    virtual void set_complexity_coef(unsigned alphabet_size, float p);

protected:
    score_t _complexity_coef;
    size_t _size;
};

/// Base class for fitness functions that use a ctable. Provides useful
/// table compression
struct bscore_ctable_base : public bscore_base
{
    bscore_ctable_base(const CTable&);

    /// Indicate a set of features that should be ignored during scoring,
    /// The features are indicated as indexes, starting from 0.
    ///
    /// This function is used to improve performance by avoiding
    /// evaluation of the ignored features. By ignoring most columns,
    /// the table can typically be significantly compressed, thus
    /// reducing evaluation time. For tables with tens of thousands of
    /// uncompressed rows, this can provide a significant speedup when
    /// evaluating a large number of instances.
    void ignore_cols(const std::set<arity_t>&) const;

    /// In case one wants to evaluate the fitness on a subset of the
    /// data, one can provide a set of row indexes to ignore.
    void ignore_rows(const std::set<unsigned>&) const;

    // Like ignore_rows but consider timestamps instead of indexes
    void ignore_rows_at_times(const std::set<TTable::value_type>&) const;

    // Return the uncompressed size of the CTable
    unsigned get_ctable_usize() const;

    // Return the original CTable
    const CTable& get_ctable() const;

protected:
    const CTable& _orig_ctable;  // Reference to the original table.

    // The table that is actually used for the evaluation. This is the
    // the compressed table that results after ignore_cols() and
    // ignore_rows() have been applied.  Must be mutable to avoid the
    // const-ness of tables in general.
    mutable CTable _wrk_ctable;

    // A copy of wrk_ctable prior to ignore_rows() being applied.  This
    // allows ignore_rows() to be called multiple times, without forcing
    // a complete recalculation.
    mutable CTable _all_rows_wrk_ctable;

    mutable size_t _ctable_usize;   // uncompressed size of ctable
};

/// Abstract base class for summing behavioral scores.
// XXX TODO FIXME: this should probably be completely removed,
// and replaced by boosting_ascore everywhere, and boosting_ascore
// should probably be renamed to weighted_score or something like
// that ... in particular, simple_ascore should be killed.
struct ascore_base : public std::unary_function<combo_tree, composite_score>
{
    /// Sum up the behavioral score
    virtual score_t operator()(const behavioral_score&) const = 0;

    virtual ~ascore_base(){}
};

/// Simplest ascore, it just totals up the bscore.
struct simple_ascore : ascore_base
{
    /// Sum up the behavioral score
    virtual score_t operator()(const behavioral_score&) const;
};

// helper to log a combo_tree and its behavioral score
static inline void log_candidate_bscore(const combo_tree& tr,
                                        const behavioral_score& bs)
{
    if (logger().isFineEnabled())
        logger().fine() << "Evaluate candidate: " << tr << "\n"
                        << "\tBScored: " << bs;
}

} //~namespace moses
} //~namespace opencog

#endif
