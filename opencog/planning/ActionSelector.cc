/*
 * @file opencog/planning/ActionSelector.cc
 *
 * Copyright (C) 2015-2016 OpenCog Foundation
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

#include <opencog/planning/Utilities.h>
#include <opencog/rule-engine/UREConfigReader.h>


#include "ActionSelector.h"

using namespace opencog;

/**
 * Default action rulebase name.
 */

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

    auto temp_actions(fetch_actions(as));

    // Check if a rule is also an action.
    // Just because an atom inherites from (ConceptNode "opencog: action")
    // doesn't make it a valid action-rule. It has to be a valid URE rule also.
    // NOTE: There is no utiility for checking a pattern
    UREConfigReader temp_rbs(_as, rbs);
    for(auto& rule : temp_rbs.get_rules()) {
        auto result = std::find(temp_actions.begin(), temp_actions.end(),
                                rule.get_alias());

        if (result != temp_actions.end()) {
            _actions.emplace_back(rule);
        } else {
            throw RuntimeException(TRACE_INFO,
                "[ActionSelector] - invalid action-rulebase specified!");
        }
    }
}

ActionSelector::~ActionSelector()
{
}
