/*
 * opencog/embodiment/WorldWrapper/WorldWrapper.h
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

#ifndef _WORLDWRAPPERBASE_H
#define _WORLDWRAPPERBASE_H

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/variable_unifier.h>

namespace opencog { namespace world {

/**
 * The WorldWrapperBase class is a layer used between the combo interpreter
 * and the world it is interpreted in.
 */
class WorldWrapperBase
{
public:

    typedef combo::combo_tree::iterator pre_it;
    typedef combo::combo_tree::sibling_iterator sib_it;

    WorldWrapperBase();
    virtual ~WorldWrapperBase();

    /**
     * return true is the current action plan is finished
     * false otherwise or if there is no action plan
     */
    virtual bool isPlanFinished() const = 0;

    /**
     * return true if the plan has failed
     * false otherwise
     * pre-condition : the plan is finished
     */
    virtual bool isPlanFailed() const = 0;

    /**
     * Send a sequence of sequential_and actions [from, to)
     * returns true iff and action plan gets executed
     */
    virtual bool sendSequential_and(sib_it from, sib_it to) = 0;

    /**
     * Send a sequence of sequential_or actions [from, to)
     */
    //virtual bool sendSequential_or(sib_it from, sib_it to);

    /**
     * Send a sequence of sequential_exec actions [from, to)
     */
    //virtual bool sendSequential_exec(sib_it from, sib_it to);

    /**
     * evaluate a perception. vu is the variable_unifier object used to
     * resolve wild_card symbol _*_
     */
    virtual combo::vertex evalPerception(pre_it per,
            combo::variable_unifier& vu = 
            combo::variable_unifier::DEFAULT_VU()) = 0;

    /**
     * evaluate an indefinite object. vu is the variable_unifier object used to
     * resolve wild_card symbol _*_
     */
    virtual combo::vertex evalIndefiniteObject(combo::indefinite_object io,
            combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU()) = 0;

};

} } // namespace opencog::world

#endif
