/*
 * @file opencog/planning/Action.h
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

#ifndef _OPENCOG_PLANNING_ACTION_H
#define _OPENCOG_PLANNING_ACTION_H

#include <opencog/atoms/pattern/BindLink.h>
#include <opencog/rule-engine/Rule.h>

namespace opencog
 {
/** \addtogroup planning
 * @{
 */

class Action
{
public:
    Action(Rule a_rule);

    /**
     * @return The rule associated with an action.
     */
    Rule get_rule();

    /**
     * @return The derived state associated with an action.
     */
    LinkPtr get_derived_state();

    /**
     * Checks if the action's derived state is satisfiable in the given
     * atomspace.
     *
     * @param as An atomspace that provides the state to be used for checking
     *           if the action's derived context is satisfiable.
     * @return `true` if it is satisfiable and `false` if not.
     */
    bool is_derived_state_satisfiable(AtomSpace& as);

private:
    /**
     * An action is  a URE rule that inherits from this node.
     */
    static const NodePtr main_action_node;

    Rule _rule;


    /**
     * It is a SatisfactionLink containing the body of the BindLink, that
     * forms the action-rule,  after all the VIRTUAL_LINKS are removed.
     * The VIRTUAL_LINKS are removed because they could have side effects that
     * changes the state of the atomspace and/or external environment. Using
     * this atom instead of `_rule` helps in minimizing side-effects during
     * action-selection/planning/simulation.
     */
    LinkPtr _derived_state;

    // Function for helping construct
    void init();
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_PLANNING_ACTION_H
