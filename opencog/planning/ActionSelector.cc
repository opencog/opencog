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

ActionSelector::ActionSelector(AtomSpace& as, Handle rbs) : _as(as), _rbs(rbs)
{
    if (Handle::UNDEFINED == rbs)
        throw RuntimeException(TRACE_INFO,
            "[ActionSelector] - Invalid action rulebase specified!");

    auto temp_actions(fetch_actions(as));

    // Check if a rule is also an action.
    // Just because an atom inherites from (ConceptNode "opencog: action")
    // doesn't make it a valid action-rule. It has to be a valid URE rule also.
    UREConfigReader temp_rbs(_as, rbs);
    for(auto& rule : temp_rbs.get_rules()) {
        auto result = std::find(temp_actions.begin(), temp_actions.end(),
                                rule.get_alias());

        if (result != temp_actions.end()) {
            _actions.emplace_back(rule);
        } else {
            throw RuntimeException(TRACE_INFO,
                "[ActionSelector] - Invalid action-rulebase."
                "This following rule is not an action-rule : %s.",
                rule.get_alias()->toString().c_str());
        }
    }
}

ActionSelector::~ActionSelector()
{
}

HandleSeq ActionSelector::select_by_context()
{
    HandleSeq result;

    for (auto i : _actions) {
        if (i.is_derived_state_satisfiable(_as)) {
            std::cout <<"\n got match";
            result.push_back(i.get_rule().get_alias());
        }
    }

    return result;
}
