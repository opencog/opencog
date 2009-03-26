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

#include "util/functional.h"
#include "util/foreach.h"

#include "comboreduct/combo/vertex.h"
#include "comboreduct/combo/complexity.h"

namespace moses {
  
  //basic types
  typedef double score_t;
  typedef combo::complexity_t complexity_t;

  typedef std::pair<score_t,complexity_t> tree_score;
  typedef opencog::tagged_item<combo::combo_tree,tree_score> scored_tree;

  typedef std::vector<float> behavioral_score;

  typedef opencog::tagged_item<behavioral_score,tree_score> behavioral_tree_score;
  typedef opencog::tagged_item<combo::combo_tree,
			    behavioral_tree_score> behavioral_scored_tree;
  
  extern const tree_score worst_possible_score;

  //convenience accessors
  inline const combo::combo_tree& get_tree(const scored_tree& st) { 
    return st.first; 
  }
  inline const combo::combo_tree& get_tree(const behavioral_scored_tree& bst) { 
    return bst.first;
  }

  inline complexity_t get_complexity(const tree_score& ts) { 
    return ts.second; 
  }
  inline complexity_t get_complexity(const behavioral_tree_score& ts) { 
    return get_complexity(ts.second);
  }
  inline complexity_t get_complexity(const behavioral_scored_tree& bst) { 
    return get_complexity(bst.second);
  }
  inline complexity_t get_complexity(const scored_tree& st) { 
    return get_complexity(st.second);
  }

  inline score_t get_score(const tree_score& ts) { 
    return ts.first;
  }
  inline score_t get_score(const behavioral_tree_score& ts) { 
    return get_score(ts.second);
  }
  inline score_t get_score(const behavioral_scored_tree& bst) { 
    return get_score(bst.second);
  }
  inline score_t get_score(const scored_tree& st) { 
    return get_score(st.second);
  }

}

inline std::ostream& operator<<(std::ostream& out,const moses::tree_score& ts) {
  return (out << "[score=" << ts.first << ", complexity=" << -ts.second << "]");
}
inline std::ostream& operator<<(std::ostream& out,
				const moses::behavioral_tree_score& s) {
  out << "[ ";
  foreach (float f,s.first)
    out << f << " ";
  out << "], " << s.second;
  return out;
}

#endif
