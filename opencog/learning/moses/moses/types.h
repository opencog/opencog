/*
 * opencog/learning/moses/moses/types.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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

#include <opencog/util/functional.h>
#include <opencog/util/foreach.h>
#include <opencog/util/iostreamContainer.h>

#include <opencog/comboreduct/combo/vertex.h>
#include "complexity.h"

namespace opencog { namespace moses {

using std::binary_function;
using combo::vertex;
  
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
// very high because that information is used by other tools
static const int io_score_precision = 32;

static const score_t worst_score =
    -(std::numeric_limits<score_t>::max()-score_t(1));
    // for some weird reasons what is below leads to bugs
    // std::numeric_limits<score_t>::min(),
    // so does this!!!
    // std::numeric_limits<score_t>::min()+1,

typedef std::pair<score_t, complexity_t> composite_score;
typedef tagged_item<combo::combo_tree,
                    composite_score> scored_combo_tree;

typedef std::vector<score_t> behavioral_score;

typedef tagged_item<behavioral_score,
                    composite_score> composite_behavioral_score;
typedef tagged_item<combo::combo_tree,
                    composite_behavioral_score> bscored_combo_tree;

extern const composite_score worst_composite_score;

typedef boost::unordered_map<combo::combo_tree, composite_behavioral_score, 
                             boost::hash<combo::combo_tree> > metapop_candidates;

typedef std::list<bscored_combo_tree> metapop_candidates_list;

//convenience accessors
inline const combo::combo_tree& get_tree(const scored_combo_tree& st) { 
    return st.first; 
}
inline const combo::combo_tree& get_tree(const bscored_combo_tree& bst) { 
    return bst.first;
}

inline const composite_score& get_composite_score(const composite_behavioral_score& ctbs) {
    return ctbs.second;
}
inline const composite_score& get_composite_score(const bscored_combo_tree& bsct) {
    return get_composite_score(bsct.second);
}

inline complexity_t get_complexity(const composite_score& ts) { 
    return ts.second; 
}
inline complexity_t get_complexity(const composite_behavioral_score& ts) { 
    return get_complexity(ts.second);
}
inline complexity_t get_complexity(const bscored_combo_tree& bst) { 
    return get_complexity(bst.second);
}
inline complexity_t get_complexity(const scored_combo_tree& st) { 
    return get_complexity(st.second);
}

inline score_t get_score(const composite_score& ts) { 
    return ts.first;
}
inline score_t get_score(const composite_behavioral_score& ts) { 
    return get_score(ts.second);
}
inline score_t get_score(const bscored_combo_tree& bst) { 
    return get_score(bst.second);
}
inline score_t get_score(const scored_combo_tree& st) { 
    return get_score(st.second);
}

inline const behavioral_score& get_bscore(const composite_behavioral_score& ts) { 
    return ts.first;
}
inline const behavioral_score& get_bscore(const bscored_combo_tree& bst) { 
    return get_bscore(bst.second);
}

// ostream functions
template<typename Out>
Out& ostream_behavioral_score(Out& out, const behavioral_score& bs) {
    return ostreamContainer(out, bs, " ", "[", "]");
}

/**
 * stream out a candidate along with their scores (optionally
 * complexity and bscore).
 */
template<typename Out>
Out& ostream_bscored_combo_tree(Out& out, const bscored_combo_tree& candidate,
                                bool output_score = true,
                                bool output_complexity = false,
                                bool output_bscore = false) {
    if(output_score)
        out << std::setprecision(io_score_precision) 
            << get_score(candidate) << " ";
    if(output_complexity)
        out << get_complexity(candidate) << " ";
    out << get_tree(candidate) << std::endl;
    if(output_bscore) {
        ostream_behavioral_score(out, get_bscore(candidate));
        out << std::endl;
    }
    return out;
}

} // ~namespace moses

inline std::ostream& operator<<(std::ostream& out,
                                const moses::composite_score& ts) {
    return out << "[score=" 
               << std::setprecision(moses::io_score_precision) 
               << ts.first 
               << ", complexity=" << ts.second << "]";
}
inline std::ostream& operator<<(std::ostream& out,
                                const moses::composite_behavioral_score& s) {
    moses::ostream_behavioral_score(out, s.first);
    out << ", " << s.second;
    return out;
}

} // ~namespace opencog

#endif
