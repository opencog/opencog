/*
 * opencog/embodiment/Control/PredicateUpdaters/IsSmallPredicateUpdater.h
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

#ifndef ISSMALLPREDICATEUPDATER_H_
#define ISSMALLPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace opencog { namespace oac {

class IsSmallPredicateUpdater : public BasicPredicateUpdater
{

protected:

    /**
     * Return the size of the object. For 2D maps the size is considered as
      * the width * length product. For 3D maps the size is considered as
      * the width * length * height product.
      *
      * NOTE: for now only 2D maps are considered
      *
      * @param object. The handle to the object to be evaluated.
     */
    double getSize(Handle object);

public:

    IsSmallPredicateUpdater(AtomSpace & atomSpace);
    ~IsSmallPredicateUpdater();

    /**
     * Update is_small predicate for the given object. Since the
     * predicate relates to the object size relative to the pet
     * size, that is, an object small for a pet could be not
     * small for other ones
     *
     * @param pet The handle of the pet (used to get its size)
     * @param object The handle of the object, used to get it's category
     *        and size.
     */
    void update(Handle object, Handle pet, unsigned long timestamp );

}; // class

} } // namespace opencog::oac

#endif /*ISSMALLPREDICATEUPDATER_H_*/
