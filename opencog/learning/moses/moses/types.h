/*
 * opencog/learning/moses/moses/types.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Moshe Looks, Linas Vepstas
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
#ifndef _MOSES_TYPES_H
#define _MOSES_TYPES_H

#include <cfloat>
#include <functional>
#include <iomanip>
#include <unordered_set>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/operators.hpp>
#include <boost/ptr_container/ptr_set.hpp>

#include <opencog/util/functional.h>
#include <opencog/util/iostreamContainer.h>

#include <opencog/comboreduct/combo/combo.h>
#include "complexity.h"

namespace opencog { namespace moses {

using combo::vertex;
using boost::indirect_iterator;
using boost::transform_iterator;

/////////////////
// basic types //
/////////////////

// A score_t is defined as float rather than double to save memory and
// limit the number of decimals needed in IO (such as logging) so that
// tools that need its textual representation can stay in sync with
// what's in RAM with less digits. If that precision is not enough,
// please try first to change the internal scoring function types then
// convert the result to score_t before changing its typedef to
// double, since float as the final score type is likely enough.
typedef float score_t;

// Score precision used for logging and printing results. It is set
// to maximum because the logged scores are used by other tools that
// need the precision.  Note: max precision of single-presicion float
// is about 8 decimal places; that for double is about 17 places.
static const int io_score_precision = 18;

static const score_t very_best_score = std::numeric_limits<score_t>::max();
static const score_t very_worst_score = std::numeric_limits<score_t>::lowest();
// use FLT_EPSILON not EPSILON because its float, not double.
static const score_t epsilon_score = FLT_EPSILON;

// But modify the default sort ordering for these objects.
struct composite_score:
        public boost::less_than_comparable<composite_score>,
        public boost::equality_comparable<composite_score>
{
    /// By convention, we expect score to be negative (so that
    /// higher scores==better scores) while cpxy and penalty are both
    /// positive.  This, higher complexity==larger number, and
    /// bigger penalty==bigger number. The penalty is *SUBTRACTED*
    /// from the score during evaluation!
    //
    // Note: we cache the total penalized_score, in order to avoid a
    // subtraction in the comparison operator.
    composite_score(score_t scor, complexity_t cpxy,
                    score_t complexity_penalty_ = 0.0,
                    score_t diversity_penalty_ = 0.0)
        : multiply_diversity(false), score(scor), complexity(cpxy),
          complexity_penalty(complexity_penalty_),
          diversity_penalty(diversity_penalty_)
    {
        update_penalized_score();
    }

    composite_score();    // build the worst score
    composite_score& operator=(const composite_score &r);

    score_t get_score() const { return score; }
    complexity_t get_complexity() const { return complexity; }
    score_t get_penalized_score() const { return penalized_score; }

    // Use this only to over-ride the score, wehn re-scoring.
    void set_score(score_t sc)
    {
        score = sc;
        update_penalized_score();
    }

    /// Sign convention: the penalty is positive, it is subtracted from
    /// the "raw" score to get the penalized score.
    score_t get_complexity_penalty() const { return complexity_penalty; }
    void set_complexity_penalty(score_t penalty)
    {
        complexity_penalty = penalty;
        update_penalized_score();
    }
    score_t get_diversity_penalty() const { return diversity_penalty; }
    void set_diversity_penalty(score_t penalty)
    {
        diversity_penalty = penalty;
        update_penalized_score();
    }
    score_t get_penalty() const
    {
        return complexity_penalty + diversity_penalty;
    }

    /// Compare penalized scores.  That is, we compare score-penalty
    /// on the right to score-penalty on the left. If the 2
    /// score-penalty qre equal then we compare by complexity
    /// (decreasing order, as complexity is positive). We do that in
    /// case no score penalty has been set.
    ///
    /// Additionally we assume that nan is always smaller than
    /// anything (including -inf) except nan
    bool operator<(const composite_score &r) const;

    /// used in test cases -- compare equality to 7 decimal places.
    bool operator==(const composite_score& r) const;

    // EXPERIMENTAL: if multiply_diversity is set to true then the
    // diversity_penalty is multiplied with the raw score instead
    // being subtracted. This makes more sense if the diversity
    // penalty represent a probability. Hmm. Except that scores
    // behave kind-of-like the logarithm of a (solomonoff) probability...
    // so if if the diversity is acting like a probability, we should
    // probably be taking it's log, and adding that.  Certainly,
    // that's exatly how we treat the complexity penalty: its the log
    // of the total number of states the combo tree represents, which
    // is why we add it ...
    bool multiply_diversity;

protected:
    score_t score;
    complexity_t complexity;
    score_t complexity_penalty;
    score_t diversity_penalty;
    score_t penalized_score;

    /// Update penalized_score, i.e. substract the complexity and
    /// diversity penalty from the raw score.
    void update_penalized_score() {
        penalized_score = score - complexity_penalty;
        if (multiply_diversity)
            penalized_score *= diversity_penalty;
        else
            penalized_score -= diversity_penalty;
    }
};

extern const composite_score worst_composite_score;

/// Assign a unique ID for each deme.  This is used to keep track of
/// which deme each candidate comes from. The demeID is stored as a string,
/// formatted as either a single a single integer EXPANSION or as a pair
/// EXPANSION.BREADTH_FIRST.   The first is used if there is only one deme
/// spawned per expansion.  The second form is used if more than one deme
/// is spawned per expansion
///
/// Here, EXPANSION is the number of times that the deme expander has
/// been called thus far. BREADTH_FIRST is the index of the deme created
/// by a single call of the deme expander. The initial metapopulation
/// comes from demeID "0".
//
// XXX wouldn't it be better to store ints here ??
struct demeID_t : public std::string
{
    demeID_t(unsigned expansion = 0 /* default initial deme */);
    demeID_t(unsigned expansion, unsigned breadth_first);
    demeID_t(unsigned expansion, unsigned breadth_first, unsigned ss_deme);
};

/// Behavioral scores record one score per row of input data.
///
/// For a boolean problem, that score will typically be zero or one,
/// signifying that the combo tree got that row correct, or not.
/// The score may differ from zero or one if the table is compressed
/// (has multiple input rows that are identical) or if the table is
/// weighted (different rows have different weights, set by user) or
/// boosted (different weights that are dynamically assigned).
///
/// That is, a behavioral score is always for a particular combot tree,
/// in reference to a particular table of data.  Exactly which tree it
/// is, and which table, is implicit.
//
// TODO this should be a std::valarray not std::vector but I am too
// lazy to make the switch right now.
struct behavioral_score : public std::vector<score_t>
{
    behavioral_score() {}
    behavioral_score(size_t sz) : std::vector<score_t>(sz) {}
    behavioral_score(size_t sz, score_t val) : std::vector<score_t>(sz, val) {}
    behavioral_score(std::initializer_list<score_t> il) : std::vector<score_t>(il) {}

    std::vector<score_t> operator-=(const std::vector<score_t>& rhs)
    {
        size_t sz = rhs.size();
        OC_ASSERT(size() == sz,
            "Error: Incompatible behavioral_score sizes, this=%zu rhs=%zu",
            size(), sz);
        for (size_t i=0; i<sz; i++) {
            (*this)[i] -= rhs[i];
        }
        return *this;
    }
};

static inline behavioral_score operator-(const behavioral_score& lhs,
                                         const behavioral_score& rhs)
{
    size_t sz = rhs.size();
    OC_ASSERT(lhs.size() == sz,
        "Error: Incompatible behavioral_score sizes, lhs=%zu rhs=%zu",
         lhs.size(), sz);
    behavioral_score bs;
    for (size_t i=0; i<sz; i++) {
        bs.push_back(lhs[i] - rhs[i]);
    }
    return bs;
}

/// A single combo tree, together with various score metrics for it.
///
/// Large parts of the system need to track a combo tree, along with
/// various performance metrics associated with that tree.  This
/// provides the place where tree-related information is kept.
///
/// Included is:
/// -- a composite score (total score, plus complexity and diversity
///    penalties)
/// -- a behavioral score (how well the tree did on each row of a table;
///    exactly which table it is is implicit)
/// -- a boosting weight (used to implement the boosting algorithm)
class scored_combo_tree : public boost::equality_comparable<scored_combo_tree>
{
public:
    scored_combo_tree(combo::combo_tree tr,
                      demeID_t id = demeID_t(),
                      composite_score cs = composite_score(),
                      behavioral_score bs = behavioral_score())
        : _tree(tr), _deme_id(id), _cscore(cs), _bscore(bs), _weight(1.0)
    {}

private:
    combo::combo_tree _tree;
    demeID_t _deme_id;
    composite_score _cscore;
    behavioral_score _bscore;
    double _weight;

public:
    const combo::combo_tree& get_tree() const { return _tree; }
    combo::combo_tree& get_tree() { return _tree; }

    const demeID_t get_demeID() const { return _deme_id; }
    demeID_t get_demeID() { return _deme_id; }

    const behavioral_score& get_bscore() const
    {
       return _bscore;
    }
    void set_bscore(const behavioral_score& bs)
    {
       _bscore = bs;
    }
    double get_weight() const
    {
       return _weight;
    }
    void set_weight(double w)
    {
       _weight = w;
    }
    const composite_score& get_composite_score() const
    {
       return _cscore;
    }
    composite_score& get_composite_score()
    {
       return _cscore;
    }

    /* Utility wrappers */
    score_t get_score() const { return _cscore.get_score(); }
    complexity_t get_complexity() const { return _cscore.get_complexity(); }
    score_t get_penalized_score() const { return _cscore.get_penalized_score(); }
    score_t get_complexity_penalty() const { return _cscore.get_complexity_penalty(); }
    score_t get_diversity_penalty() const { return _cscore.get_diversity_penalty(); }
    score_t get_penalty() const { return _cscore.get_penalty(); }

    bool operator==(const scored_combo_tree& r) const;
};

// =======================================================================
// collections of trees

/**
 * greater_than operator for scored_combo_tree.  The order is determined
 * by the composite score; that is, the composite scores are compared to
 * determine ordering.  If the scores are equal, then are equal, then the
 * tree sizes are compared; if these are equal, the trees are compared
 * lexicographically.  Note that tree equality requires two lexicographic
 * compares :-(
 *
 * FYI, this ordering makes the non-standard assumption that anything
 * is greater than nan.   This is done so as to not pollute the
 * metapopulation or the deme with candidates with undefined scores
 * (as these are usually very bad candidates).
 */
struct sct_score_greater
    : public std::binary_function<scored_combo_tree, scored_combo_tree, bool>
{
    bool operator()(const scored_combo_tree&,
                    const scored_combo_tree&) const;
};

/**
 * greater_than operator for scored_combo_tree.  The order is determined
 * first by tree size, then by tree lexicographic order. Note that tree
 * equality requires two  lexicographic compares :-(
 */
struct sct_tree_greater
    : public std::binary_function<scored_combo_tree, scored_combo_tree, bool>
{
    bool operator()(const scored_combo_tree&,
                    const scored_combo_tree&) const;
};

struct scored_combo_tree_hash
     : public std::unary_function<scored_combo_tree, size_t>
{
    size_t operator()(const scored_combo_tree&) const;
};

struct scored_combo_tree_equal
     : public std::binary_function<scored_combo_tree, scored_combo_tree, bool>
{
    bool operator()(const scored_combo_tree&,
                    const scored_combo_tree&) const;
};

/// scored_combo_tree_hash_set provides an O(1) way of determining if
/// a combo tree is in the set, or not (and getting its score, if it is).
/// Its O(1) in theory. In practice, it can be quite slow, for two
/// reasons: one is that it needs to compute the hash of the tree, and
/// since trees can be big, this will be expensive.  The other problem
/// it that this invokes the copy constructor for insertion.
/// See below for other containers with different properties.
typedef std::unordered_set<scored_combo_tree,
                 scored_combo_tree_hash,
                 // scored_combo_tree_equal> scored_combo_tree_hash_set;
                 scored_combo_tree_equal> scored_combo_tree_set;

/// scored_combo_tree_tset offers a fairly fast, mutable storage for
/// combo trees, based on the combo tree itself, and not how its scored.
/// This has slghtly slower insert time than scored_combo_tree_ptr_set
/// below, which uses scores to order the trees. But its more stable,
/// precisely because it does not use the scores (which thus are allowed
/// to change, depending on the situation).
typedef boost::ptr_set<scored_combo_tree,
                       // sct_tree_greater> scored_combo_tree_set;
                       sct_tree_greater> scored_combo_tree_tset;

/// scored_combo_tree_ptr_set holds scored combo trees, using the
/// composite score to order the elements. Thus, if the trees have
/// all been consistently scored, then we can easily spot two unequal
/// trees simply by looking at their scores. However, if the scorer
/// is changing the way in which it is scoring over time, so that the
/// same tree might be scored differently at tdifferent times, then
/// there is no way to guarantee that a given tree appears only once
/// in the set. You have been warned!
///
typedef boost::ptr_set<scored_combo_tree,
                       sct_score_greater> scored_combo_tree_ptr_set;
typedef scored_combo_tree_ptr_set::iterator scored_combo_tree_ptr_set_it;
typedef scored_combo_tree_ptr_set::const_iterator scored_combo_tree_ptr_set_cit;

// =======================================================================
// ostream functions

std::ostream& ostream_behavioral_score(std::ostream& out, const behavioral_score&);

// Stream out a scored combo tree.
std::ostream& ostream_scored_combo_tree(std::ostream& out,
                                        const scored_combo_tree&,
                                        bool output_score = true,
                                        bool output_cscore = true,
                                        bool output_demeID = true,
                                        bool output_bscore = true);

scored_combo_tree string_to_scored_combo_tree(const std::string& line);

std::istream& istream_scored_combo_trees(std::istream& in,
                                         std::vector<scored_combo_tree>& scts);

inline std::ostream& operator<<(std::ostream& out,
                                const moses::scored_combo_tree& sct)
{
    return moses::ostream_scored_combo_tree(out, sct);
}

inline std::ostream& operator<<(std::ostream& out,
                                const moses::composite_score& ts)
{
    return out << "[score="
               << std::setprecision(moses::io_score_precision)
               << ts.get_score()
               << ", penalized score=" << ts.get_penalized_score()
               << ", complexity=" << ts.get_complexity()
               << ", complexity penalty=" << ts.get_complexity_penalty()
               << ", diversity penalty=" << ts.get_diversity_penalty()
               << "]";
}

inline std::ostream& operator<<(std::ostream& out,
                                const moses::behavioral_score& s)
{
    return moses::ostream_behavioral_score(out, s);
}

} // ~namespace moses
} // ~namespace opencog

#endif
