/*
 * @file opencog/planning/Utilities.h
 *
 * Copyright (C) 2016 OpenCog Foundation
 * All Rights Reserved
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

#include <opencog/atomspaceutils/AtomSpaceUtils.h>
#include <opencog/query/BindLinkAPI.h>

#include "Utilities.h"

using namespace opencog;


HandleSeq opencog::fetch_actions(AtomSpace& as)
{
    // Pattern typing
    Handle var_type = as.add_node(TYPE_NODE, "ConceptNode"),
        var_alias = as.add_node(VARIABLE_NODE, "__ACTION_ALIAS__"),
        typed_var = as.add_link(TYPED_VARIABLE_LINK, var_alias, var_type);

    // Pattern structure
    Handle oc_action = as.add_node(CONCEPT_NODE, "opencog: action"),
        member_link = as.add_link(INHERITANCE_LINK, var_alias, oc_action);

    // Pattern
    Handle get_link = as.add_link(GET_LINK, typed_var, member_link);

    // Fetch atoms satisfying pattern
    Handle results = satisfying_set(&as, get_link);
    HandleSeq actions = results->getOutgoingSet();

    // Garbage Collection.
    remove_hypergraph(as, get_link);
    as.remove_atom(results);

    return actions;
}
