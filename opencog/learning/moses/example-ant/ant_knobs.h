/*
 * opencog/learning/moses/moses/ant_knobs.h
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
#ifndef _MOSES_ANT_KNOBS_H
#define _MOSES_ANT_KNOBS_H

#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include "knobs.h"

namespace opencog { namespace moses {

using namespace ant_combo;

// Knob specifically designed for the Sante Fe Trail problem.
// Note - children aren't canonized when parents are called.
struct ant_action_subtree_knob : public discrete_knob<4> {
    static const int none    = 0;
    static const int forward = 1;
    static const int rleft   = 2;   // rotate left
    static const int rright  = 3;   // rotate right

    ant_action_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                            combo_tree::iterator subtree)
        : discrete_knob<4>(tr) {

        _default = none;
        _current = _default;

        _loc = _tr.append_child(tgt, id::null_vertex);
        _tr.append_child(_loc, subtree);
    }

    complexity_t complexity_bound() const {
        return tree_complexity(_loc);
    }

    void clear_exemplar() {
        if (in_exemplar())
            turn(0);
        else
            _tr.erase(_loc);
    }

    void turn(int idx) {
        idx = map_idx(idx);
        OC_ASSERT((idx < 4), "Index greater than 3.");

        if (idx == _current) //already set, nothing to
            return;

        if (idx == none) {
            if (_current != none)
                _loc = _tr.insert_above(_loc, id::null_vertex);
        } else {
            if (_current == none)
                _tr.erase_children(_loc);

            switch (idx) {
            case forward:
                *_loc = get_instance(id::move_forward);
                break;

            case rleft:
                *_loc = get_instance(id::turn_left);
                break;

            case rright:
                *_loc = get_instance(id::turn_right);
                break;

            default:
                break;
            }
        }

        _current = idx;
    }

    field_set::disc_spec spec() const {
        return field_set::disc_spec(multiplicity());
    }

    std::string toStr() const {
        stringstream ss;
        ss << "[" << *_loc << " TODO ]";
        return ss.str();
    }
};

} //~namespace moses
} //~namespace opencog

#endif

