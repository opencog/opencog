/*
 * @file opencog/planning/Action.cc
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

#include <opencog/atoms/pattern/PatternLink.h>
#include <opencog/atomutils/FollowLink.h>

#include "Action.h"

using namespace opencog;

const std::string Action::main_action_name = "opencog: action";

/**
 * Main Constructor
 */
Action::Action(Rule a_rule) : _rule(a_rule)
{
    init();
}

void Action::init()
{
    PatternLinkPtr state(PatternLinkCast(_rule.get_handle()));

    if (NULL == state) {
        throw InvalidParamException(TRACE_INFO,
            "[Action::init()] Expecting a PatternLink type or ",
            "sub-type for Rule handle, got %s",
             _rule.get_handle()->toString().c_str());
    }

    FollowLink fl;
    // NOTE:  Assuming only one InheritanceLink
    auto oc_node = fl.follow_binary_link(_rule.get_alias(), INHERITANCE_LINK);
    auto oc_node_ptr = NodeCast(oc_node);

    if (not oc_node_ptr or (main_action_name != oc_node_ptr->getName()))
        throw InvalidParamException(TRACE_INFO,
            "[Action::init()] the rule is not an action type");

    Type implicant_type = state->get_body()->getType();

    // Construct the derived rule. Assuming the body is wrapped in an
    // AndLink or SequentialAndLink. The variable declaration isn't modified
    // because the variables in virtual-links must also be part of the body
    // of the pattern independent of the virtual-links.
    // TODO: If the body is wrapped in an OrLink, ChoiceLink,
    // SequentailOrLink, RandomChoiceLink or their derivatives
    // create a separate rule for each. Will require breaking the variable
    // declaration as the a variable declared must occur in the body.
    if (not (classserver().isA(implicant_type, AND_LINK) or
             classserver().isA(implicant_type, SEQUENTIAL_AND_LINK)))
        throw InvalidParamException(TRACE_INFO,
            "[Action::init()] Expecting a AndLink/SequentialAndLink type as",
            "the implicant, got %s", classserver().getTypeName(implicant_type).c_str());

    auto derived_body = Handle(createLink(implicant_type, state->get_fixed()));

    // Check if variable-declaration has been made before extracting the
    // derived state.
    if (state->get_vardecl()) {
        _derived_state = createLink(SATISFACTION_LINK, HandleSeq {
                            state->get_vardecl(),
                            derived_body});
    } else {
        _derived_state = createLink(SATISFACTION_LINK, derived_body);
    }
}

/**
 * @return The rule associated with an action.
 */
Rule Action::get_rule()
{
    return _rule;
}

/**
 * @return The derived state associated with an action.
 */
LinkPtr Action::get_derived_state()
{
    return _derived_state;
}
