/*
 * opencog/embodiment/Control/PredicateUpdaters/IsPickupablePredicateUpdater.h
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

#ifndef ISPICKUPABLEPREDICATEUPDATER_H_
#define ISPICKUPABLEPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace opencog { namespace oac {

class IsPickupablePredicateUpdater : public BasicPredicateUpdater
{

private:

    /**
     * Return true if the object is movable and false otherwise. An object
     * to be pickable must be movable, but the oposite isn't true.
     *
     * @param object The handle of the object
     */
    bool isMovable(Handle Object);

public:
    IsPickupablePredicateUpdater(AtomSpace &atomSpace);
    ~IsPickupablePredicateUpdater();

    void update(Handle object, Handle pet, unsigned long timestamp );

}; // class

} } // namespace opencog::oac

#endif /*ISPICKUPABLEPREDICATEUPDATER_H_*/
