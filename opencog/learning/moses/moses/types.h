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

// score precision used for logging and outputting results, it is set
// very high because that information may be used by other tools
static const int io_score_precision = 32;

static const score_t very_best_score = std::numeric_limits<score_t>::max();
static const score_t very_worst_score = std::numeric_limits<score_t>::lowest();

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
    // penalty represent a probability
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
typedef std::vector<score_t> behavioral_score;

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
/// -- a boosing vector (used to implement the boosting algorithm)
class scored_combo_tree
{
public:
    scored_combo_tree(combo::combo_tree tr,
                      demeID_t id = demeID_t(),
                      composite_score cs = composite_score(),
                      behavioral_score bs = behavioral_score())
        : _tree(tr), _deme_id(id), _cscore(cs), _bscore(bs)
    {}

private:
    combo::combo_tree _tree;
    demeID_t _deme_id;
    composite_score _cscore;
    behavioral_score _bscore;
    behavioral_score _boost;

public:
    const combo::combo_tree& get_tree(void) const { return _tree; }
    combo::combo_tree& get_tree(void) { return _tree; }

    const demeID_t get_demeID() const { return _deme_id; }
    demeID_t get_demeID() { return _deme_id; }

    const behavioral_score& get_bscore(void) const
    {
       return _bscore;
    }
    const behavioral_score& get_boost(void) const
    {
       return _boost;
    }
    behavioral_score& get_bscore(void)
    {
       return _bscore;
    }
    const composite_score& get_composite_score(void) const
    {
       return _cscore;
    }
    composite_score& get_composite_score(void)
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
};

// =======================================================================
// collections of trees

/**
 * greater_than operator for scored_combo_tree.  The order is
 * determined by the composite score; that is, the composite
 * scores are compared to determin ordering.
 *
 * FYI, this ordering makes the non-standard assumption that anything
 * is greater than nan.   This is done so as to not pollute the
 * metapopulation or the deme with candidates with undefined scores
 * (as these are usually very bad candidates).
 */
struct scored_combo_tree_greater
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

/// scored_combo_tree_set provides an O(1) way of determining if a combo
/// tree is in the set, or not (and getting its score, if it is).
typedef std::unordered_set<scored_combo_tree,
                 scored_combo_tree_hash,
                 scored_combo_tree_equal> scored_combo_tree_set;

typedef boost::ptr_set<scored_combo_tree,
                       scored_combo_tree_greater> scored_combo_tree_ptr_set;
typedef scored_combo_tree_ptr_set::iterator scored_combo_tree_ptr_set_it;
typedef scored_combo_tree_ptr_set::const_iterator scored_combo_tree_ptr_set_cit;

// =======================================================================
// ostream functions
template<typename Out>
Out& ostream_behavioral_score(Out& out, const behavioral_score& bs)
{
    return ostreamContainer(out, bs, " ", "[", "]");
}

/**
 * stream out a candidate along with their scores (optionally
 * complexity and bscore).
 *
 * @param bool output_python if true, output is a python module
 *             instead of a combo program XXX currently broken XXX
 */
static const std::string complexity_prefix_str = "complexity:";
static const std::string complexity_penalty_prefix_str = "complexity penalty:";
static const std::string diversity_penalty_prefix_str = "diversity penalty:";
static const std::string penalized_score_prefix_str = "penalized score:";
static const std::string behavioral_score_prefix_str = "behavioral score:";

template<typename Out>
Out& ostream_scored_combo_tree(Out& out,
                                 const scored_combo_tree& sct,
                                 bool output_score = true,
                                 bool output_penalty = false,
                                 bool output_bscore = false,
                                 bool output_python = false)
{
    const combo::combo_tree& tr = sct.get_tree();
    const composite_score& cs = sct.get_composite_score();
    const behavioral_score& bs = sct.get_bscore();

    if (output_python)
        return ostream_combo_tree_cpbscore_python(out, tr, cs, bs,
                                                  output_score,
                                                  output_penalty,
                                                  output_bscore);

    if (output_score)
        out << std::setprecision(io_score_precision)
            << cs.get_score() << " ";

    out << tr << std::endl;

    if (output_penalty)
        out << complexity_prefix_str << " "
            << cs.get_complexity() << std::endl
            << complexity_penalty_prefix_str << " "
            << cs.get_complexity_penalty() << std::endl
            << diversity_penalty_prefix_str << " "
            << cs.get_diversity_penalty() << std::endl
            << penalized_score_prefix_str << " "
            << cs.get_penalized_score() << std::endl;

    if (output_bscore)
        ostream_behavioral_score(out << behavioral_score_prefix_str << " ",
                                 bs) << std::endl;

    return out;
}

// Stream in scored_combo_tree, use the same format as
// ostream_scored_combo_tree. Note that for now we assume that combo
// tree is always preceeded by the score, it's easier that way.
//
// You may want to set 'in' to send exceptions if something goes wrong
// such as
//
// in.exceptions(ifstream::failbit | ifstream::badbit | ifstream::eofbit);
//
// so a bad parse is detected (maybe istream_scored_combo_tree should
// set it automatically).
//
// TODO: if the istream doesn't end by a bscore then it will
// completely exhaust it.
template<typename In>
scored_combo_tree istream_scored_combo_tree(In& in)
{
    // parse score
    score_t sc;
    in >> sc;

    // parse combo tree
    combo::combo_tree tr;
    in >> tr;

    // parse the rest
    complexity_t cpx = 0;
    score_t cpx_penalty = 0, diversity_penalty = 0, penalized_score = 0;
    behavioral_score bs;
    // whitespace, function split
    auto ssplit = [](const std::string& s) {
        std::vector<std::string> res;
        boost::split(res, s, boost::algorithm::is_space());
        return res;
    };
    std::vector<std::string> complexity_penalty_prefix_str_split =
        ssplit(complexity_penalty_prefix_str);
    std::vector<std::string> diversity_penalty_prefix_str_split =
        ssplit(diversity_penalty_prefix_str);
    std::vector<std::string> penalized_score_prefix_str_split =
        ssplit(penalized_score_prefix_str);
    std::vector<std::string> behavioral_score_prefix_str_split =
        ssplit(behavioral_score_prefix_str);
    while (in.good()) {
        std::string token;
        in >> token;
        if (token == complexity_prefix_str)
            in >> cpx;
        else if (token == complexity_penalty_prefix_str_split[0]) {
            in >> token;        // complexity_penalty_prefix_str_split[1]
            in >> cpx_penalty;
        }
        else if (token == diversity_penalty_prefix_str_split[0]) {
            in >> token;        // diversity_penalty_prefix_str_split[1]
            in >> diversity_penalty;
        }
        else if (token == penalized_score_prefix_str_split[0]) {
            in >> token;        // penalized_score_prefix_str_split[1]
            in >> penalized_score;
        }
        else if (token == behavioral_score_prefix_str_split[0]) {
            in >> token;        // behavioral_score_prefix_str_split[1]
            istreamContainer(in, back_inserter(bs), "[", "]");
            break;              // that way we don't consume 'in' further
        }
    }
    // assign to candidate
    combo::combo_tree tr_test = tr;

    composite_score cs(sc, cpx, cpx_penalty, diversity_penalty);
    return scored_combo_tree(tr, /* default demeID */ 0, cs, bs);
}

/**
 * stream out a candidate along with their scores (optionally
 * complexity and bscore) as a python module
 */
template<typename Out>
Out& ostream_combo_tree_cpbscore_python(Out& out,
                                        const combo::combo_tree& tr,
                                        const composite_score& cs,
                                        const behavioral_score& bs,
                                        bool output_score = true,
                                        bool output_penalty = false,
                                        bool output_bscore = false)
{
    out << "#!/usr/bin/env python" << std::endl
        << "from operator import *" << std::endl
        << std::endl
        << "#These functions allow multiple args instead of lists." << std::endl
        << "def ors(*args):" << std::endl
        << "    return any(args)" << std::endl
        << std::endl
        << "def ands(*args):" << std::endl
        << "    return all(args)" << std::endl
        << std::endl;

    if (output_score) {
        out << "#score: " << std::setprecision(io_score_precision)
            << cs.get_score() << std::endl;
    }
    if (output_penalty) {
        out << " #complexity: " << cs.get_complexity() << std::endl;
        out << " #complexity_penalty: " << cs.get_complexity_penalty() << std::endl;
        out << " #diversity_penalty: " << cs.get_diversity_penalty() << std::endl;
    }

    out << std::endl << "def moses_eval(i):" << std::endl << "    return ";
    ostream_combo_tree(out, tr, combo::fmt::python);
    out << std::endl;

    if (output_bscore) {
        out << std::endl<< "#bscore: " ;
        ostream_behavioral_score(out, bs);
        out << std::endl;
    }
    return out;
}


inline std::ostream& operator<<(std::ostream& out,
                                const moses::composite_score& ts)
{
    return out << "[score="
               << std::setprecision(moses::io_score_precision)
               << ts.get_score()
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
