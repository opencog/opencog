/*
 * @file opencog/planning/Action.cc
 * @author Amen Belayneh <amenbelayneh@gmail.com> August 2015
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
Action::Action(Handle member_l) : Rule(member_l)
{
    init();
}

void Action::init()
{
    /*PatternLinkPtr pattern(PatternLinkCast(rule_handle_));
    _virtual_terms(pattern->get_pattern()._virtual);
    _fixed_terms(pattern->get_pattern()._fixed);
    */
}

Action::~Action()
{
}
