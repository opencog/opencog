/*
 * opencog/embodiment/Learning/RewritingRules/RewritingRules.h
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

#ifndef _REWRITING_RULES_H
#define _REWRITING_RULES_H

#include <opencog/comboreduct/reduct/reduct.h>
#include "hillclimbing_action_reduction.h"
#include "hillclimbing_perception_reduction.h"
#include "hillclimbing_full_reduction.h"
#include "post_learning_rewriting.h"

namespace opencog { namespace reduct {

//hillclimbing
inline void hillclimbing_full_reduce(combo_tree& tr, combo_tree::iterator it)
{
    hillclimbing_full_reduction()(tr, it);
}

inline void hillclimbing_full_reduce(combo_tree& tr)
{
    hillclimbing_full_reduction()(tr);
}

inline void hillclimbing_perception_reduce(combo_tree& tr, combo_tree::iterator it)
{
    hillclimbing_perception_reduction()(tr, it);
}

inline void hillclimbing_perception_reduce(combo_tree& tr)
{
    hillclimbing_perception_reduction()(tr);
}

inline void hillclimbing_action_reduce(combo_tree& tr, combo_tree::iterator it)
{
    hillclimbing_action_reduction()(tr, it);
}

inline void hillclimbing_action_reduce(combo_tree& tr)
{
    hillclimbing_action_reduction()(tr);
}

//post_learning
inline void post_learning_rewrite(combo_tree& tr, combo_tree::iterator it)
{
    post_learning_rewriting()(tr, it);
}

inline void post_learning_rewrite(combo_tree& tr)
{
    post_learning_rewriting()(tr);
}


} // ~namespace reduct
} // ~namespace opencog

#endif
