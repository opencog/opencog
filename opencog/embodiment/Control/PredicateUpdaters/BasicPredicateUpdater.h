/*
 * opencog/embodiment/Control/PredicateUpdaters/BasicPredicateUpdater.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef BASICPREDICATEUPDATER_H_
#define BASICPREDICATEUPDATER_H_

#include <opencog/util/Logger.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>

using namespace opencog;

namespace opencog { namespace oac {

class BasicPredicateUpdater
{

public:

    /*
     * Constructor and destructor;
     */
    BasicPredicateUpdater(AtomSpace& _atomSpace) : atomSpace(_atomSpace) {}

    virtual  ~BasicPredicateUpdater() {}

    /**
     * Update all the predicates of given objects.  
     *
     * The default behavior of this function, especially for is_X predicates, is
     * simply invoking the 'update' function below for each object. In many 
     * circumstances, that is exactly what we want. Then there's no need to
     * override this function. We only have to override the function below.  
     *
     * However, if we want to consider all the objects all at once, such as during 
     * calculating spatial relations, we should override this function. 
     */
    virtual void update(std::vector<Handle> & objects, Handle pet, unsigned long timestamp);

    /**
     * Update is_X predicate based on the object in question. 
     *
     * The pet handle may be used to update predicates relative to the pet.
     *
     * @param object The handle of the object
     * @param pet The handle of the pet
     */
    virtual void update(Handle object, Handle pet, unsigned long timestamp); 

    /**
     * Return true if there is already a is_X predicate created for the given
     * object handle.
     *
     * @param object The handle of the object
     * @param predicateName A string representation of the predicate
     */
    bool isUpdated(Handle object, std::string predicateName);

protected:

    AtomSpace & atomSpace;

    /**
     * Return the handle of all of the EvalLink for the predicate in
     * question with the given object. If there is no such predicate
     * then Handle::UNDEFINED is returned.
     *
     * @param object The handle of the object
     * @param predicateName A string representation of the predicate
     */
    Handle getPredHandle(Handle object, std::string predicateName);

    /**
     * Get the handle of the OBJECT_NODE (or subtype) for the given name.
     *
     * @return Handle::UNDEFINED if no handle is found, the exactly handle if
     * just one handle is found or the first handle if more than on handle
     * is found
     */
    Handle getHandle(std::string objName);

}; // class

} } // namespace opencog::oac

#endif
