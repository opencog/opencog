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

#include <sstream>

namespace opencog { namespace moses {

using namespace std;
        
bool bscored_combo_tree_greater::operator()(const bscored_combo_tree& bs_tr1,
                                            const bscored_combo_tree& bs_tr2) const
{
    composite_score csc1 = get_composite_score(bs_tr1);
    composite_score csc2 = get_composite_score(bs_tr2);
    bool gt = (csc1 > csc2);
    if (gt) return true;
    bool lt = (csc1 < csc2);
    if (lt) return false;

    // If we are here, then they are equal.  We are desperate to break
    // a tie, because otherwise, the bscored_combo_tree_set will discard
    // anything that compares equal, and we really don't want that.
    score_t sc1 = get_score(csc1);
    score_t sc2 = get_score(csc2);
    gt = (sc1 > sc2);
    if (gt) return true;
    lt = (sc1 < sc2);
    if (lt) return false;

    // Arghh, still tied!  The above already used complexity to break
    // the tie.  We're grasping at straws, here.
    return size_tree_order<vertex>()(get_tree(bs_tr1), get_tree(bs_tr2));
}

// the empty composite_score ctor returns the worst composite score
const composite_score worst_composite_score = composite_score();

composite_score::composite_score()
    : score(very_worst_score), complexity(worst_complexity),
      complexity_penalty(0.0), diversity_penalty(0.0),
      penalized_score(very_worst_score)
{}

composite_score& composite_score::operator=(const composite_score &r)
{
    score = r.score;
    complexity = r.complexity;
    penalized_score = r.penalized_score;
    complexity_penalty = r.complexity_penalty;
    diversity_penalty = r.diversity_penalty;
    return *this;
}

bool composite_score::operator<(const composite_score &r) const
{
    score_t lef = penalized_score;
    score_t rig = r.penalized_score;

    if (isnan(lef))
        return !isnan(rig);
    else
        return (lef < rig)
                // Note: I've tried to see if the addition below
                // increases the performance when there is no
                // complexity penalty and it doesn't, over 100 runs
                // solving 3-parity is actually .5s slower (which
                // probably has no statistical significance given it
                // takes 13s in average). But I don't want to conclude
                // too fast, it could have an impact on other
                // problems.
                or (lef == rig and complexity > r.complexity);
}

bool composite_score::operator==(const composite_score &r) const
{
    return score == r.get_score()
        && complexity == r.get_complexity()
        && complexity_penalty == r.get_complexity_penalty()
        && diversity_penalty == r.get_diversity_penalty()
        && penalized_score == r.get_penalized_score();
}

/// Compute the distance between two vectors, using the p-norm.
score_t lp_distance(const behavioral_score& a, const behavioral_score& b, double p)
{
    // // debug
    // {
    //     stringstream ss;
    //     ostream_behavioral_score(ss, a);
    //     logger().fine("a = %s", ss.str().c_str());
    // }
    // {
    //     stringstream ss;
    //     ostream_behavioral_score(ss, b);
    //     logger().fine("b = %s", ss.str().c_str());
    // }
    // score_t res = p_norm(a, b, p);
    // logger().fine("res = %f", res);
    // return res;
    // // ~debug
    return p_norm(a, b, p);
}

// score_t lp_distance(const penalized_behavioral_score& a, const penalized_behavioral_score& b, double p)
// {
//     return lp_distance(a.first, b.first, p);
// }

// score_t lp_distance(const composite_behavioral_score& a, const composite_behavioral_score& b, double p)
// {
//     return lp_distance(a.first, b.first, p);
// }

// score_t lp_distance(const bscored_combo_tree& a, const bscored_combo_tree& b, double p)
// {
//     return lp_distance(a.second, b.second, p);
// }
        
} // ~namespace moses
} // ~namespace opencog
