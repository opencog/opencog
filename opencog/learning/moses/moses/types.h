/*
 * opencog/learning/moses/moses/types.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
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

#include <boost/unordered_map.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/operators.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/ptr_container/ptr_set.hpp>

#include <opencog/util/functional.h>
#include <opencog/util/iostreamContainer.h>

#include <opencog/comboreduct/combo/combo.h>
#include "complexity.h"

namespace opencog { namespace moses {

using std::binary_function;
using combo::vertex;
using boost::indirect_iterator;
using boost::transform_iterator;

/////////////////
// basic types //
/////////////////

// a score_t is defined as float rather than double to save memory and
// limit the number of decimals needed in IO (such as logging) so that
// tools that need its textual representation can stay in sync with
// what's in RAM with less digits. If that precision is not enough,
// please try first to change the internal scoring function types then
// convert the result to score_t before changing its typedef to
// double, float as final score type is likely enough
typedef float score_t;

// score precision used for logging and outputting results, it is set
// very high because that information may be used by other tools
static const int io_score_precision = 32;

static const score_t very_best_score = std::numeric_limits<score_t>::max();

// below we use 1 - best score and not
// std::numeric_limits<score_t>::min, please recall that in the STL
// standard min when applied to a floating type returns the smallest
// possible representable value
static const score_t very_worst_score = score_t(1) - very_best_score;

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
    score_t get_penalty() const { return complexity_penalty + diversity_penalty; }

    /// Compare penalized scores.  That is, we compare score-penalty
    /// on the right to score-penalty on the left. If the 2
    /// score-penalty qre equal then we compare by complexity
    /// (decreasing order, as complexity is positive). We do that in
    /// case no score penalty has been set.
    ///
    /// Additionally we assume that nan is always smaller than
    /// anything (including -inf) except nan
    bool operator<(const composite_score &r) const;

    /// useful for testing (probably not in practice)
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

// In order to keep track from which deme each candidate comes we
// assign a unique ID for each deme.
//
// The demeID is formatted as followed, either
//
// EXPANSION
//
// If only one deme is spawned per expansion
//
// or
//
// EXPANSION.BREADTH_FIRST
//
// If more than one deme is spawned per expansion
//
// Where EXPANSION is the number of times the deme expander has been
// called thus far. And BREADTH_FIRST is the index of the deme created
// by one call of the deme expander. The initial metapopulation comes
// from demeID "0" by convention.
struct demeID_t : public std::string {
    demeID_t(unsigned expansion = 0 /* default initial deme */);
    demeID_t(unsigned expansion, unsigned breadth_first);
};
        
typedef std::vector<score_t> behavioral_score;
typedef std::pair<behavioral_score, score_t> penalized_bscore;
typedef std::pair<penalized_bscore, composite_score> composite_penalized_bscore;
typedef std::pair<composite_penalized_bscore, demeID_t> cpbscore_demeID;
typedef std::pair<combo::combo_tree, cpbscore_demeID> pbscored_combo_tree;

///////////////////////////
// convenience accessors //
///////////////////////////

score_t get_penalized_score(const composite_score& sc);

const combo::combo_tree& get_tree(const pbscored_combo_tree& pbst);
combo::combo_tree& get_tree(pbscored_combo_tree& pbst);

const composite_penalized_bscore& get_composite_penalized_bscore(const pbscored_combo_tree& pbst);
composite_penalized_bscore& get_composite_penalized_bscore(pbscored_combo_tree& pbst);

const cpbscore_demeID& get_cpbscore_demeID(const pbscored_combo_tree& pbst);
cpbscore_demeID& get_cpbscore_demeID(pbscored_combo_tree& pbst);

demeID_t get_demeID(const pbscored_combo_tree& pbst);

// The following function creates an overloading ambiguity in gcc
// 4.6.3 (apparently fixed in gcc 4.7.2)
// const composite_score& get_composite_score(const composite_penalized_bscore& ctbs);
const composite_score& get_composite_score(const pbscored_combo_tree& bsct);
composite_score& get_composite_score(composite_penalized_bscore& ctbs);
composite_score& get_composite_score(pbscored_combo_tree& bsct);

score_t get_penalized_score(const composite_score& sc);
score_t get_penalized_score(const composite_penalized_bscore& cpb);
score_t get_penalized_score(const pbscored_combo_tree& st);

score_t get_score(const composite_score& ts);
score_t get_score(const composite_penalized_bscore& ts);
score_t get_score(const pbscored_combo_tree& bst);

complexity_t get_complexity(const composite_score& ts);
complexity_t get_complexity(const composite_penalized_bscore& ts);
complexity_t get_complexity(const pbscored_combo_tree& bst);

score_t get_complexity_penalty(const composite_score& ts);
score_t get_complexity_penalty(const composite_penalized_bscore& ts);
score_t get_complexity_penalty(const pbscored_combo_tree& bst);

score_t get_diversity_penalty(const composite_score& ts);
score_t get_diversity_penalty(const composite_penalized_bscore& ts);
score_t get_diversity_penalty(const pbscored_combo_tree& bst);

score_t get_penalty(const composite_score& ts);
score_t get_penalty(const composite_penalized_bscore& ts);
score_t get_penalty(const pbscored_combo_tree& bst);

const penalized_bscore& get_pbscore(const composite_penalized_bscore& ts);
const penalized_bscore& get_pbscore(const pbscored_combo_tree& bst);
penalized_bscore& get_pbscore(composite_penalized_bscore& ts);
penalized_bscore& get_pbscore(pbscored_combo_tree& bst);

const behavioral_score& get_bscore(const penalized_bscore& pbs);
const behavioral_score& get_bscore(const composite_penalized_bscore& cbs);
const behavioral_score& get_bscore(const pbscored_combo_tree& bst);

/**
 * greater_than operator for pbscored_combo_tree.  The order is
 * determined by the composite score; that is, the composite
 * scores are compared to determin ordering.
 *
 * FYI, this ordering makes the non-standard assumption that anything
 * is greater than nan.   This is done so as to not pollute the
 * metapopulation or the deme with candidates with undefined scores
 * (as these are usually very bad candidates).
 */
struct pbscored_combo_tree_greater : public binary_function<pbscored_combo_tree,
                                                            pbscored_combo_tree,
                                                            bool>
{
    bool operator()(const pbscored_combo_tree& bs_tr1,
                    const pbscored_combo_tree& bs_tr2) const;
};
typedef std::set<pbscored_combo_tree,
                 pbscored_combo_tree_greater> pbscored_combo_tree_set;
typedef pbscored_combo_tree_set::iterator pbscored_combo_tree_set_it;
typedef pbscored_combo_tree_set::const_iterator pbscored_combo_tree_set_cit;

typedef boost::ptr_set<pbscored_combo_tree,
                       pbscored_combo_tree_greater> pbscored_combo_tree_ptr_set;
typedef pbscored_combo_tree_ptr_set::iterator pbscored_combo_tree_ptr_set_it;
typedef pbscored_combo_tree_ptr_set::const_iterator pbscored_combo_tree_ptr_set_cit;

typedef std::vector<pbscored_combo_tree> pbscored_combo_tree_seq;
typedef pbscored_combo_tree_seq::iterator pbscored_combo_tree_seq_it;
typedef pbscored_combo_tree_seq::const_iterator pbscored_combo_tree_seq_cit;

/// metapop_candidates provides an O(1) way of determining if a combo
/// tree is in the map, or not (and getting its score, if it is).
typedef boost::unordered_map<combo::combo_tree,
                             pbscored_combo_tree::second_type,
                             boost::hash<combo::combo_tree> > metapop_candidates;
typedef metapop_candidates::value_type metapop_candidate;
typedef metapop_candidates::iterator metapop_candidates_it;
typedef metapop_candidates::const_iterator metapop_candidates_cit;

// ostream functions
template<typename Out>
Out& ostream_behavioral_score(Out& out, const behavioral_score& bs)
{
    return ostreamContainer(out, bs, " ", "[", "]");
}

template<typename Out>
Out& ostream_penalized_bscore(Out& out, const penalized_bscore& pbs)
{
    out << pbs.second << " ";
    return ostreamContainer(out, pbs.first, " ", "[", "]");
}

/**
 * stream out a candidate along with their scores (optionally
 * complexity and bscore).
 *
 * @param bool output_python if true, output is a python module instead of a combo program
 */
static const std::string complexity_prefix_str = "complexity:";
static const std::string complexity_penalty_prefix_str = "complexity penalty:";
static const std::string diversity_penalty_prefix_str = "diversity penalty:";
static const std::string penalized_score_prefix_str = "penalized score:";
static const std::string behavioral_score_prefix_str = "behavioral score:";
template<typename Out>
Out& ostream_pbscored_combo_tree(Out& out, const pbscored_combo_tree& cnd,
                                 bool output_score = true,
                                 bool output_penalty = false,
                                 bool output_bscore = false,
                                 bool output_python = false)
{
    return ostream_combo_tree_composite_pbscore(out, get_tree(cnd),
                                                get_composite_penalized_bscore(cnd),
                                                output_score,
                                                output_penalty,
                                                output_bscore,
                                                output_python);
}
template<typename Out>
Out& ostream_combo_tree_composite_pbscore(Out& out,
                                          const combo::combo_tree& tr,
                                          const composite_penalized_bscore& cpb,
                                          bool output_score = true,
                                          bool output_penalty = false,
                                          bool output_bscore = false,
                                          bool output_python = false)
{
    if (output_python)
        return ostream_combo_tree_composite_pbscore_python(out, tr, cpb,
                                                           output_score,
                                                           output_penalty,
                                                           output_bscore);

    if (output_score)
        out << std::setprecision(io_score_precision)
            << get_score(cpb) << " ";
    
    out << tr << std::endl;

    if (output_penalty)
        out << complexity_prefix_str << " "
            << get_complexity(cpb) << std::endl
            << complexity_penalty_prefix_str << " "
            << get_complexity_penalty(cpb) << std::endl
            << diversity_penalty_prefix_str << " "
            << get_diversity_penalty(cpb) << std::endl
            << penalized_score_prefix_str << " "
            << get_penalized_score(cpb) << std::endl;

    if (output_bscore)
        ostream_behavioral_score(out << behavioral_score_prefix_str << " ",
                                 get_bscore(cpb)) << std::endl;

    return out;
}

// Stream in pbscored_combo_tree, use the same format as
// ostream_pbscored_combo_tree. Note that for now we assume that combo
// tree is always preceeded by the score, it's easier that way.
//
// You may want to set 'in' to send exceptions if something goes wrong
// such as
//
// in.exceptions(ifstream::failbit | ifstream::badbit | ifstream::eofbit);
//
// so a bad parse is detected (maybe istream_pbscored_combo_tree should
// set it automatically).
//
// TODO: if the istream doesn't end by a bscore then it will
// completely exhaust it.
template<typename In>
pbscored_combo_tree istream_pbscored_combo_tree(In& in) {

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
    
    penalized_bscore pbs(bs, cpx_penalty);
    composite_score cs(sc, cpx, cpx_penalty, diversity_penalty);
    composite_penalized_bscore cbs(pbs, cs);
    cpbscore_demeID cbs_demeID(cbs, /* default demeID */ 0);
    return pbscored_combo_tree(tr, cbs_demeID);
}                                      

/**
 * stream out a candidate along with their scores (optionally
 * complexity and bscore) as a python module
 */
template<typename Out>
Out& ostream_combo_tree_composite_pbscore_python(Out& out,
                                                 const combo::combo_tree& tr,
                                                 const composite_penalized_bscore& cpb,
                                                 bool output_score = true,
                                                 bool output_penalty = false,
                                                 bool output_bscore = false)
{
    out << std::endl
        << "#!/usr/bin/python" << std::endl
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
            << get_score(cpb) << std::endl;
    }
    if (output_penalty) {
        out << " #complexity: " << get_complexity(cpb) << std::endl;
        out << " #complexity_penalty: " << get_complexity_penalty(cpb) << std::endl;
        out << " #diversity_penalty: " << get_diversity_penalty(cpb) << std::endl;
    }

    out << std::endl << "def moses_eval(i):" << std::endl << "    return ";
    ostream_combo_tree(out, tr, combo::fmt::python);
    out << std::endl;

    if (output_bscore) {
        out << std::endl<< "#bscore: " ;
        ostream_penalized_bscore(out, cpb.first);
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
                                const moses::composite_penalized_bscore& s)
{
    moses::ostream_penalized_bscore(out, s.first);
    out << ", " << s.second;
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const moses::behavioral_score& s)
{
    return moses::ostream_behavioral_score(out, s);
}

inline std::ostream& operator<<(std::ostream& out,
                                const moses::penalized_bscore& s)
{
    return moses::ostream_penalized_bscore(out, s);
}

} // ~namespace moses
} // ~namespace opencog

#endif
