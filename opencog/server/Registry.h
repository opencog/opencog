/*
 * opencog/server/Registry.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

/* Simple implementation of a factory registry */

#ifndef _OPENCOG_REGISTRY_H
#define _OPENCOG_REGISTRY_H

#include <string>
#include <list>

#include <opencog/server/Factory.h>
#include <opencog/server/Request.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>
#include <opencog/util/exceptions.h>

namespace opencog
{

template< typename _BaseType >
class Registry
{

protected:

    typedef typename std::map<const std::string, const AbstractFactory<_BaseType>*> FactoryMap;
    typedef typename std::map<const std::string, const AbstractFactory<_BaseType>*>::const_iterator FactoryMapConstIterator;
    typedef typename std::map<const std::string, const AbstractFactory<_BaseType>*>::iterator FactoryMapIterator;
    typedef typename std::map<const std::string, const AbstractFactory<_BaseType>*>::value_type FactoryMapValueType;
    FactoryMap factories;

public:

    Registry() {}
    virtual ~Registry() {}

    virtual bool register_(const std::string& id, AbstractFactory<_BaseType> const* factory)
    {
        logger().debug("registering %s \"%s\"", demangle(typeid(_BaseType).name()).c_str(), id.c_str());
        return factories.insert(FactoryMapValueType(id, factory)).second;
    }

    virtual bool unregister(const std::string& id)
    {
        logger().debug("unregistering %s \"%s\"", demangle(typeid(_BaseType).name()).c_str(), id.c_str());
        return factories.erase(id) == 1;
    }

    virtual _BaseType* create(const std::string& id)
    {
        FactoryMapConstIterator it = factories.find(id);
        if (it == factories.end()) {
            // not found
            logger().error("unknown %s id: %s", demangle(typeid(_BaseType).name()).c_str(), id.c_str());
            return NULL;
        }
        // invoke the creation function
        logger().debug("creating %s instance with \"%s\"", demangle(typeid(_BaseType).name()).c_str(), id.c_str());
        return it->second->create();
    }

    const ClassInfo& classinfo(const std::string& id) const
    {
        static ClassInfo emptyClassInfo;
        FactoryMapConstIterator it = factories.find(id);
        if (it == factories.end()) {
            // not found
            logger().error("unknown %s id: %s", demangle(typeid(Request).name()).c_str(), id.c_str());
            return emptyClassInfo;
        }
        // invoke the description function
        logger().debug("returning %s classinfo with id \"%s\"", demangle(typeid(Request).name()).c_str(), id.c_str());
        return it->second->info();
    }

    virtual std::list<const char*> all(void) const
    {
        logger().debug("listing all %ss", demangle(typeid(_BaseType).name()).c_str());
        std::list<const char*> l;
        FactoryMapConstIterator it;
        for (it = factories.begin(); it != factories.end(); ++it) {
            l.push_back(it->first.c_str());
        }
        return l;
    }
};

} // namespace opencog

#endif // _OPENCOG_REGISTRY_H
