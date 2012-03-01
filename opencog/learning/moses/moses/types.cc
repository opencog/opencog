/*
 * opencog/learning/moses/moses/types.cc
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
#include "types.h"
#include "complexity.h"

namespace opencog { namespace moses {

bool bscored_combo_tree_greater::operator()(const bscored_combo_tree& bs_tr1,
                                            const bscored_combo_tree& bs_tr2) const
{
    composite_score csc1 = get_composite_score(bs_tr1),
        csc2 = get_composite_score(bs_tr2);
    return (csc1 > csc2)
        || (!(csc2 > csc1) &&
            size_tree_order<vertex>()(get_tree(bs_tr1),
                                      get_tree(bs_tr2)));
}

// the empty composite_score ctor returns the worst composite score
const composite_score worst_composite_score = composite_score();

score_t composite_score::weight = 4.0f;

composite_score::composite_score(score_t s, complexity_t c)
    : super(s, c) {}
composite_score::composite_score(const std::pair<score_t, complexity_t> &p)
    : super(p) {}
composite_score::composite_score()
    : super(worst_score, worst_complexity) {}
composite_score& composite_score::operator=(const composite_score &r) {
    first = r.first; second = r.second; return *this;
}
bool composite_score::operator<(const composite_score &r) const {
    score_t lef = weight*first + second;
    score_t rig = weight*r.first + r.second;
    return (lef < rig);
}

} // ~namespace moses
} // ~namespace opencog
