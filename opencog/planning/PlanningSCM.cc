/*
 * @file opencog/planning/PlanningSCM.cc
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

#include <opencog/guile/SchemeSmob.h>
#include <opencog/planning/ActionSelector.h>

#include "PlanningSCM.h"

using namespace opencog;

PlanningSCM::PlanningSCM() : ModuleWrap("opencog planning") {}

void PlanningSCM::init()
{
    // TODO: Let one specify the action selection similar to
    // psi-get-action-rules
    // define_scheme_primitive("cog-filter-actions",
    //     &PlanningSCM::filter_actions, this, "planning");

    define_scheme_primitive("cog-select-actions",
        &PlanningSCM::select_actions, this, "planning");
}

HandleSeq PlanningSCM::select_actions(Handle rbs)
{
    if (Handle::UNDEFINED == rbs)
        throw RuntimeException(TRACE_INFO,
            "PlanningSCM::select_actions - invalid rulebase!");

    AtomSpace *as = SchemeSmob::ss_get_env_as("cog-select-actions");
    ActionSelector selector(*as, rbs);

    return selector.select_by_context();
}

// Create a single static instance.
void opencog_planning_init(void)
{
    static PlanningSCM planning;
    planning.module_init();
}
