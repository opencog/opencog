/*
 * opencog/embodiment/Control/PredicateUpdaters/IsPooPlacePredicateUpdater.h
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

#ifndef ISPOOPLACEPREDICATEUPDATER_H_
#define ISPOOPLACEPREDICATEUPDATER_H_

#include "BasicPredicateUpdater.h"

namespace opencog { namespace oac {

class IsPooPlacePredicateUpdater : public BasicPredicateUpdater
{

public:
    IsPooPlacePredicateUpdater(AtomSpace &atomSpace);
    ~IsPooPlacePredicateUpdater();

    void update(Handle object, Handle pet, unsigned long timestamp );

}; // class;

} } // namespace opencog::oac

#endif /*ISDRINKABLE_H_*/
