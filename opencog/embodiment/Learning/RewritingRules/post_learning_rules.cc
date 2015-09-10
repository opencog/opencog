/*
 * opencog/embodiment/Learning/RewritingRules/post_learning_rules.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#include "post_learning_rules.h"
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

namespace opencog { namespace reduct {

using namespace AvatarCombo;

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

//add a drop action in front of a grab action
void post_learning_drop_before_grab::operator()(combo_tree& tr,
        combo_tree::iterator it) const
{
    if (*it == id::sequential_exec || *it == id::sequential_or
            || *it == id::sequential_and) {
        for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
            if (*sib == get_instance(id::grab)) {
                sib_it pre_sib = tr.previous_sibling(sib);
                if (tr.is_valid(pre_sib)) {
                    if (*pre_sib != get_instance(id::drop))
                        sib = tr.insert(sib, get_instance(id::drop));
                } else sib = tr.insert(sib, get_instance(id::drop));
            }
        }
    }
}

//add action_success at child of any empty and_seq
void post_learning_empty_and_seq::operator()(combo_tree& tr,
        combo_tree::iterator it) const
{
    if (*it == id::sequential_and && it.is_childless()) {
        tr.append_child(it, id::action_success);
    }
}

} // ~namespace reduct
} // ~namespace opencog
