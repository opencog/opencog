/** deme_params.h --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_DEME_PARAMETERS_H
#define _OPENCOG_DEME_PARAMETERS_H

#include "feature_selector.h"
#include "../representation/representation.h"

namespace opencog { namespace moses {

static const operator_set empty_ignore_ops = operator_set();

/**
 * parameters for deme management
 */
struct deme_parameters
{
    deme_parameters(bool _reduce_all = true,
                    const operator_set& _ignore_ops = empty_ignore_ops,
                    const combo_tree_ns_set* _perceptions = NULL,
                    const combo_tree_ns_set* _actions = NULL,
                    const feature_selector* _fstor = NULL) :
        reduce_all(_reduce_all),
        ignore_ops(_ignore_ops),
        perceptions(_perceptions),
        actions(_actions),
        fstor(_fstor),
        linear_contin(true)
        {}

    // The max number of candidates considered to be added to the
    // metapopulation, if negative then all candidates are considered.
    int max_candidates_per_deme;

    // If true then all candidates are reduced before evaluation.
    bool reduce_all;

    // the set of operators to ignore
    operator_set ignore_ops;

    // the set of perceptions of an optional interactive agent
    const combo_tree_ns_set* perceptions;
    // the set of actions of an optional interactive agent
    const combo_tree_ns_set* actions;

    const feature_selector* fstor;

    // Build only linear expressions involving contin features.
    // This can greatly decrease the number of knobs created during
    // representation building, resulting in much smaller field sets,
    // and instances that can be searched more quickly. However, in
    // order to fit the data, linear expressions may not be as good,
    // and thus may require more time overall to find...
    bool linear_contin;

    // Defines how many pairs of literals constituting subtrees op(l1
    // l2) are considered while creating the prototype of an
    // exemplar. It ranges from 0 to 1, 0 means arity positive
    // literals and arity pairs of literals, 1 means arity positive
    // literals and arity*(arity-1) pairs of literals
    float perm_ratio;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_DEME_PARAMETERS_H
