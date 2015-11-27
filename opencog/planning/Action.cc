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
#include "Action.h"

using namespace opencog;

/**
 * Main Constructor
 */
Action::Action(Rule a_rule) : _rule(a_rule)
{
    init();
}

void Action::init()
{

    PatternLinkPtr pattern(PatternLinkCast(_rule.get_handle()));
    if(NULL == pattern) {
        throw InvalidParamException(TRACE_INFO,
			"[Action::init()] Expecting a PatternLink type or ",
            "sub-type for Rule handle, got %s",
             _rule.get_handle()->toString().c_str());
    }

    // Construct the derived rule. Assuming the body is wrapped in an
    // AndLink or SequentialAndLink. The variable declaration isn't modified
    // because the variables in virtual-links must also be part of the body
    // of the pattern independent of the virtual-links.
    // TODO: If the body is wrapped in an OrLink, ChoiceLink,
    // SequentailOrLink, RandomChoiceLink or ....
    // create a separate rule for each. Will require breaking the variable
    // declaration as the a variable declared must occur in the body.
    auto temp_hs = pattern->get_fixed();
    temp_hs.insert(temp_hs.begin(), pattern->get_vardecl());

    _derived_rule = createBindLink(temp_hs);
}

/**
 * @return The rule associated with an action.
 */
Rule Action::get_rule()
{
    return _rule;
}

/**
 * @return The derived rules associated with an action.
 */
BindLinkPtr Action::get_derived_rule()
{
    return _derived_rule;
}
