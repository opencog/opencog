/*
 * @file opencog/planning/ActionSelector.cc
 * @author Amen Belayneh <amenbelayneh@gmail.com> November 2015
 *
 * Copyright (C) 2015 OpenCog Foundation
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

//#include <algorithm>

#include <opencog/atomspaceutils/AtomSpaceUtils.h>
#include <opencog/rule-engine/UREConfigReader.h>
#include <opencog/query/BindLinkAPI.h>

#include "ActionSelector.h"

using namespace opencog;

/**
 * Default action rulebase name.
 */
const std::string ActionSelector::action_rbs_name = "opencog: action-rbs";

/**
 * ActionSelector Constructor.
 *
 * @param as Atomspace on which planning is being performed
 * @param rbs handle of the atom defining the rulebase which is the set of all
 *            actions to be selected.
 */
ActionSelector::ActionSelector(AtomSpace& as, Handle rbs) : _as(as), _rbs(rbs)
{
    if (Handle::UNDEFINED == rbs)
        throw RuntimeException(TRACE_INFO,
            "[ActionSelector] - invalid action rulebase specified!");

    auto temp_actions(fetch_actions());

    // Construct actions from rules.
    UREConfigReader temp_rbs(_as, rbs);

    // Check if a rule is also an action.
    for(auto& rule : temp_rbs.get_rules()) {
        auto result = std::find(temp_actions.begin(), temp_actions.end(),
                                rule.get_alias());

        if (result != temp_actions.end()) {
            _actions.emplace_back(rule);
        } else {
            throw RuntimeException(TRACE_INFO,
                "[ActionSelector] - invalid action rulebase specified!");
        }
    }
}

ActionSelector::~ActionSelector()
{
}

/**
 * Gets the actions aliases from the atomspace. This function doesn't check
 * if the alias is part of the rulebase, thus is in Private access scope.
 *
 * @return HandleSeq of action aliases.
 */
HandleSeq ActionSelector::fetch_actions()
{
    // Pattern typing
    Handle var_type = _as.add_node(TYPE_NODE, "ConceptNode"),
        var_alias = _as.add_node(VARIABLE_NODE, "__ACTION_ALIAS__"),
        typed_var = _as.add_link(TYPED_VARIABLE_LINK, var_alias, var_type);

    // Pattern structure
    Handle base_rbs = _as.add_node(CONCEPT_NODE, action_rbs_name),
        member_link = _as.add_link(INHERITANCE_LINK, var_alias, base_rbs);

    // Pattern
    Handle get_link = _as.add_link(GET_LINK, typed_var, member_link);

    // Fetch aliases
    Handle outputs = satisfying_set(&_as, get_link);

    // Garbage Collection.
    remove_hypergraph(_as, get_link);

    return LinkCast(outputs)->getOutgoingSet();
}
