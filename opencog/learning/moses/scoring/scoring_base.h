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
using combo::count_t;
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
    bscore_base() : _return_weighted_score(false), _complexity_coef(0.0), _size(0) {};
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

    /// Return the worst possible bscore achievable with this fitness
    /// function. This is needed during boosting, to ascertain if at
    // least half the answers are correct.
    virtual behavioral_score worst_possible_bscore() const;

    /// Return the smallest change in the score which can be considered
    /// to be an improvement over the previous score. This is useful for
    /// avoiding local maxima which have a very flat top. That is, where
    /// all combo trees in the same local maximum have almost exactly
    /// the same score, and so scores improve by very small amounts
    /// during the search.  In such cases, one can save a lot of CPU
    /// time by terminating the search when the imrpovements are smaller
    /// than the min_improv(). Returns 0.0 by default.
    virtual score_t min_improv() const { return 0.0; }

    /// Return weighted scores instead of flat scores.  The weighted
    /// scores are needed by the boosting algorithms; the unweighted
    /// scores are needed to find out what the "actual" score would be.
    void use_weighted_scores() { _return_weighted_score = true; }

    /// Return the (possbily  weighted) sum of the behavioral score.
    /// If _return_weighted_score is false, then this returns the "flat"
    /// score, a simple sum over all samples:
    /// 
    ///      score = sum_x BScore(x)
    /// 
    /// Otherwise, it returns a weighted sum of the bscore:
    /// 
    ///      score = sum_x weight(x) * BScore(x)
    ///
    /// Each element in the bscore typically corresponds to a sample in
    /// a supervised training set, that is, a row of a table contianing
    /// the training data.  By default, the weight is 1.0 for each entry.
    /// The intended use of the weights is for boosting, so that the
    /// the score for erroneous rows can be magnified, such as in AdaBoost.
    ///
    /// See, for example, http://en.wikipedia.org/wiki/AdaBoost --
    /// However, CAUTION! That wikipedia article currently (as of July
    /// 2014) contains serious, fundamental mistakes in it's desciption
    /// of the boosting algo!
    virtual score_t sum_bscore(const behavioral_score&) const;

    /// Reset the weights to a uniform distribution.
    virtual void reset_weights();

    /// A vector of per-bscore weights, used to tote up the behavioral
    /// score into a single number.
    // XXX TODO should be a std::valarray not a vector.
    virtual void update_weights(const std::vector<double>&);

    /// Return the amount by which the bscore differs from a perfect
    /// score.  This is used by the boosting algorithm to weight the
    /// a scored combo tree.
    ///
    /// The returned value must be normalized so that 0.0 stands for
    /// a perfect score (all answers are the best possible), a value
    /// of 0.5 corresponds to "random guessing", and 1.0 corresponds
    /// to a worst-possible score (all answers are the worst possible.)
    /// This error amount does not have to be a metric or distance
    /// measure, nor does it have to be linear; however, boosting will
    /// probably work better if the error is vaguely metric-like and
    /// quasi-linear.
    ///
    /// See the notes below, for the CTable sccorer, for special
    /// considerations that CTable-based scorers must make.
    virtual score_t get_error(const behavioral_score&) const;

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
    bool _return_weighted_score;
    score_t _complexity_coef;
    mutable size_t _size; // mutable to work around const bugs
    std::vector<double> _weights;
};

/// Base class for fitness functions that use a ctable. Provides useful
/// table compression.
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

    /// Like ignore_rows but consider timestamps instead of indexes
    void ignore_rows_at_times(const std::set<TTable::value_type>&) const;

    /// Return the uncompressed size of the CTable
    unsigned get_ctable_usize() const;

    /// Return the original CTable
    const CTable& get_ctable() const;

    /// Implementing get_error() for CTables-based scorers equires some
    /// special consideration.  First, the length of the behavioral
    /// score is needed, for normalization.  The correct "length" is
    /// kind-of tricky to understand when a table has weighted rows,
    /// or when it is degenerate.  In the degenerate case, no matter
    /// what selection is made, some rows will be wrong.
    ///
    /// We explicitly review these cases here: the table may have
    /// degenerate or non-degenerate rows, and these may be weighted or
    /// non-weighted.  Here, the "weights" are not the boosting weights,
    /// but the user-specified row weights.
    ///
    ///  non-degenerate, non weighted:
    ///       (each row has defacto weight of 1.0)
    ///       best score = 0.0 so  err = score / num rows;
    ///
    ///  non-degenerate, weighted:
    ///       best score = 0.0 so  err = score / weighted num rows;
    ///       since the score is a sum of weighted rows.
    ///
    ///       e.g. two rows with user-specified weights:
    ///            0.1
    ///            2.3
    ///       so if first row is wrong, then err = 0.1/2.4
    ///       and if second row is wrong, err = 2.3/2.4
    ///
    ///  degenerate, non-weighted:
    ///       best score > 0.0   err = (score - best_score) / eff_num_rows;
    ///
    ///       where eff_num_rows = sum_row fabs(up-count - down-count)
    ///       is the "effective" number of rows, as opposing rows
    ///       effectively cancel each-other out.  This is also the
    ///       "worst possible score", what would be returned if every
    ///       row was marked wrong.
    ///
    ///       e.g. table five uncompressed rows:
    ///            up:1  input-a
    ///            dn:2  input-a
    ///            up:2  input-b
    ///       best score is -1 (i.e. is 4-5 where 4 = 2+2).
    ///       so if first row is wrong, then err = (1-1)/5 = 0/3
    ///       so if second row is wrong, then err = (2-1)/5 = 1/3
    ///       so if third & first is wrong, then err = (3-1)/3 = 2/3
    ///       so if third & second is wrong, then err = (4-1)/3 = 3/3
    ///
    /// Thus, the "effective_length" is (minus) the worst possible score.
    ///
    /// The subtraction (score - best_score) needs to be done in the
    /// by the get_error() method, and not somewhere else: that's
    /// because the boost row weighting must be performed on this
    /// difference, so that only the rows that are far away from their
    /// best-possible values get boosted.
    ///
    // score_t get_error(const behavioral_score&) const;
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
    mutable CTable _all_rows_wrk_ctable; // mutable to work around const bugs.

    mutable size_t _ctable_usize;    // uncompressed size of ctable
    mutable count_t _ctable_weight;  // Total weight of all rows in table.

    void recompute_weight() const;   // recompute _ctable_weight
};

// helper to log a combo_tree and its behavioral score
static inline void log_candidate_bscore(const combo_tree& tr,
                                        const behavioral_score& bs)
{
    if (logger().isFineEnabled())
        logger().fine() << "Evaluate candidate: " << tr << "\n"
                        << "\tBScore size=" << bs.size()
                        << " bscore: " << bs;
}

} //~namespace moses
} //~namespace opencog

#endif
