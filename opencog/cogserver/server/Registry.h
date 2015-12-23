/*
 * opencog/cogserver/server/Registry.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_REGISTRY_H
#define _OPENCOG_REGISTRY_H

#include <string>
#include <list>

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>

#include <opencog/cogserver/server/Factory.h>
#include <opencog/cogserver/server/RequestClassInfo.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class Request;
class CogServer;

/**
 * This template implements a simplified Factory registry, following some of the
 * guidelines provided by the book 'Modern C++ Design' by Andrei Alexandrescu.
 */
template< typename _BaseType >
class Registry
{

protected:

    typedef typename std::map<const std::string, const
        AbstractFactory<_BaseType>*> FactoryMap;
    typedef typename std::map<const std::string, const
        AbstractFactory<_BaseType>*>::const_iterator FactoryMapConstIterator;
    typedef typename std::map<const std::string, const
        AbstractFactory<_BaseType>*>::iterator FactoryMapIterator;
    typedef typename std::map<const std::string, const
        AbstractFactory<_BaseType>*>::value_type FactoryMapValueType;
    FactoryMap factories;

public:

    Registry() {}
    virtual ~Registry() {}

    /** Registers a factory identified by 'id' */
    virtual bool register_(const std::string& id, AbstractFactory<_BaseType>
            const* factory)
    {
        logger().debug("registering %s \"%s\"",
                demangle(typeid(_BaseType).name()).c_str(), id.c_str());
        return factories.insert(FactoryMapValueType(id, factory)).second;
    }

    /** Unregisters the factory identified by 'id' */
    virtual bool unregister(const std::string& id)
    {
        logger().debug("Unregistering %s \"%s\"", 
            demangle(typeid(_BaseType).name()).c_str(), id.c_str());
        return factories.erase(id) == 1;
    }

    /** Creates a new instance using the factory identified by 'id' */
    virtual _BaseType* create(CogServer& cs, const std::string& id)
    {
        FactoryMapConstIterator it = factories.find(id);
        if (it == factories.end()) {
            // If it wasn't found, then the user probably made a 
            // simple typo at the command line. 
            logger().debug("Unknown %s command: %s", 
               demangle(typeid(_BaseType).name()).c_str(), id.c_str());
            return NULL;
        }
        // invoke the creation function
        logger().debug("Creating %s instance with \"%s\"", 
             demangle(typeid(_BaseType).name()).c_str(), id.c_str());
        return it->second->create(cs);
    }

    /** Returns the metadata associated with the factory identified by 'id' */
    const ClassInfo& classinfo(const std::string& id) const
    {
        // XXX It is common for cogserver code to upcast, without any 
        // runtime checking, (i.e. without using dynamic_cast) to class 
        // RequestClassInfo, and then dereference the "help" string. 
        // This occurs, for example, when the user types "help asdf" at
        // the command shell, and the "asdf" command does not exist. 
        // The unchecked cast and access causes a crash. The correct fix 
        // would be to not cast all over the place. But that's a lot of
        // work, since & not * is used all over the place. So, for now,
        // this fix will do the trick.  XXX
        //
        static RequestClassInfo emptyClassInfo;
        emptyClassInfo.help = "Error: No such command";
        FactoryMapConstIterator it = factories.find(id);
        if (it == factories.end()) {
            // Not found
            // Log only as 'info', not 'error' since the most likely
            // cause is user-error, i.e. mis-typed something at the
            // terminal. And even 'info' is maybe too strong for that.
            logger().info("unknown %s id: %s",
                    demangle(typeid(Request).name()).c_str(), id.c_str());
            return emptyClassInfo;
        }
        // Invoke the description function
        logger().debug("returning %s classinfo with id \"%s\"",
                demangle(typeid(Request).name()).c_str(), id.c_str());
        return it->second->info();
    }

    /** Returns a list with all the ids from the registered factories */
    virtual std::list<const char*> all(void) const
    {
        logger().debug("listing all %ss",
                demangle(typeid(_BaseType).name()).c_str());
        std::list<const char*> l;
        FactoryMapConstIterator it;
        for (it = factories.begin(); it != factories.end(); ++it) {
            l.push_back(it->first.c_str());
        }
        return l;
    }
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_REGISTRY_H
