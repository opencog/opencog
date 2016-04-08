/*
 * @file opencog/planning/ActionSelector.h
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

 #ifndef _OPENCOG_PLANNING_ACTION_SELECTOR_H
 #define _OPENCOG_PLANNING_ACTION_SELECTOR_H

 #include <vector>

 #include <opencog/planning/Action.h>


namespace opencog
{
/** \addtogroup planning
 * @{
 */

class ActionSelector
{
public:
    /**
     * ActionSelector Constructor.
     *
     * @param as Atomspace on which planning is being performed
     * @param rbs Handle of the atom defining the rulebase which is the set
     *            of all actions to be selected.
     */
    ActionSelector(AtomSpace& as, Handle rbs);
    ~ActionSelector();

    /**
     * Selects the actions by context
     *
     * @returns A vector of Handles of the alias Nodes f the action-rules
     *          that have satisfiable derived states.
     */
    HandleSeq select_by_context();

    /**
     * @returns a vector of actions that form the action-rulebase.
     */
    vector<Action> get_actions();

private:
    AtomSpace& _as;

    vector<Action> _actions;

    // Initial rulebase
    Handle _rbs;
};

/** @}*/
} // namespace opencog

 #endif  // _OPENCOG_PLANNING_ACTION_SELECTOR_H
