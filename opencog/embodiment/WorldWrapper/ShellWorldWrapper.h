/*
 * opencog/embodiment/WorldWrapper/ShellWorldWrapper.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _SHELLWORLDWRAPPER_H
#define _SHELLWORLDWRAPPER_H

#include "WorldWrapper.h"

namespace opencog { namespace world {

/**
 * interactive shell world wrapper
 */

class ShellWorldWrapper : public WorldWrapperBase
{

public:

    /**
     * Constructor, destructor
     */
    ShellWorldWrapper();
    ~ShellWorldWrapper();


    /**
     * return true is the current action plan is finished
     * false otherwise or if there is no action plan
     */
    bool isPlanFinished() const;

    /**
     * return true if the plan has failed
     * false otherwise
     * pre-condition : the plan is finished
     */
    bool isPlanFailed() const;

    /**
     * Send a sequence of sequential_and actions [from, to)
     * stdio the user to decide whether the plan has failed or not
     * return true iff the plan is actually executed
     */
    bool sendSequential_and(sib_it from, sib_it to);

    /**
     * evaluate a perception
     */
    combo::vertex evalPerception(pre_it per,
                                 combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    /**
     * evaluate an indefinite object
     */
    combo::vertex evalIndefiniteObject(combo::indefinite_object io,
                                       combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

private:
    bool _isFailed;
    bool _isFinished;
};

} } // namespace opencog::world

#endif
