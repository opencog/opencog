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

#include <boost/unordered_map.hpp>

#include <opencog/util/functional.h>
#include <opencog/util/foreach.h>
#include <opencog/util/printContainer.h>

#include <opencog/comboreduct/combo/vertex.h>
#include "complexity.h"

namespace moses {
  
    //basic types
    typedef double score_t;
    typedef combo::complexity_t complexity_t;

    typedef std::pair<score_t,complexity_t> combo_tree_score;
    typedef opencog::tagged_item<combo::combo_tree,
                                 combo_tree_score> scored_combo_tree;

    typedef std::vector<score_t> behavioral_score;

    typedef opencog::tagged_item<behavioral_score,
                                 combo_tree_score> combo_tree_behavioral_score;
    typedef opencog::tagged_item<combo::combo_tree,
                                 combo_tree_behavioral_score> behavioral_scored_combo_tree;

    typedef boost::unordered_map<combo::combo_tree, combo_tree_behavioral_score, 
                                 boost::hash<combo::combo_tree> > metapop_candidates;
  
    extern const combo_tree_score worst_possible_score;

    //convenience accessors
    inline const combo::combo_tree& get_tree(const scored_combo_tree& st) { 
        return st.first; 
    }
    inline const combo::combo_tree& get_tree(const behavioral_scored_combo_tree& bst) { 
        return bst.first;
    }

    inline complexity_t get_complexity(const combo_tree_score& ts) { 
        return ts.second; 
    }
    inline complexity_t get_complexity(const combo_tree_behavioral_score& ts) { 
        return get_complexity(ts.second);
    }
    inline complexity_t get_complexity(const behavioral_scored_combo_tree& bst) { 
        return get_complexity(bst.second);
    }
    inline complexity_t get_complexity(const scored_combo_tree& st) { 
        return get_complexity(st.second);
    }

    inline score_t get_score(const combo_tree_score& ts) { 
        return ts.first;
    }
    inline score_t get_score(const combo_tree_behavioral_score& ts) { 
        return get_score(ts.second);
    }
    inline score_t get_score(const behavioral_scored_combo_tree& bst) { 
        return get_score(bst.second);
    }
    inline score_t get_score(const scored_combo_tree& st) { 
        return get_score(st.second);
    }

    template<typename Out>
    Out& ostream_behavioral_score(Out& out, const behavioral_score bs) {
        return opencog::ostreamContainer(out, bs, " ", "[", "]");
    }

} // ~namespace moses

inline std::ostream& operator<<(std::ostream& out,
                                const moses::combo_tree_score& ts) {
    return (out << "[score=" << ts.first << ", complexity=" << -ts.second << "]");
}
inline std::ostream& operator<<(std::ostream& out,
                                const moses::combo_tree_behavioral_score& s) {
    moses::ostream_behavioral_score(out, s.first);
    out << ", " << s.second;
    return out;
}

#endif
