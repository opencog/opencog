/*
 * opencog/embodiment/Control/PredicateUpdaters/PredicatesUpdater.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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


/**
 * Update all is_X predicates once a XML message has been processed by the
 * PAI component.
 */
#ifndef PREDICATESUPDATER_H_
#define PREDICATESUPDATER_H_

#include "BasicPredicateUpdater.h"
#include <opencog/embodiment/Control/AvatarInterface.h>
#include <opencog/atomspace/AtomSpace.h>

#include <vector>

using namespace opencog;

namespace opencog { namespace oac {

class PredicatesUpdater
{

private:

    /**
     * holds all predicate updaters to be called when the update action
     * takes place
     */
    std::vector<BasicPredicateUpdater *> updaters;
    BasicPredicateUpdater* petPsychePredicatesUpdater;

    AtomSpace &atomSpace;
    std::string petId;

public:

    PredicatesUpdater(AtomSpace &_atomSpace, const std::string &_petId);

    ~PredicatesUpdater();

    /**
     * Update the predicates based on the objects that were
     * created or changed via a PVPMessage processed by the
     * PAI component.
     *
     * @param objects A std::vector containing the handles of
     *     all OBJECT_NODES that were updated
     * @param timestamp The current timestamp in the virtual world.
     */
    void update(std::vector<Handle> objects, unsigned long timestamp);

};// class

} } // namespace opencog::oac

#endif /*PREDICATEUPDATER_H_*/
