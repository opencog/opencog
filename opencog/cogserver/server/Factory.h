/*
 * opencog/cogserver/server/Factory.h
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

/* simple implementation of a AbstractFactory + Factory pattern */

#ifndef _OPENCOG_FACTORY_H
#define _OPENCOG_FACTORY_H

#include <string>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class CogServer;

/** Defines the base class metadata information. For the base class, this
 * consists of the class' id only. */
struct ClassInfo
{
    std::string id;

    ClassInfo() {};
    ClassInfo(const char* s) : id(s) {};
    ClassInfo(const std::string& s) : id(s) {};
};
 
/** Defines an abstract factory template, following Alexandrescu's pattern from
 * 'Modern C++ Design' */
template< typename _BaseType >
class AbstractFactory
{
public:
    explicit AbstractFactory() {};
    virtual ~AbstractFactory() {}
    virtual _BaseType* create(CogServer&) const = 0;
    virtual const ClassInfo& info() const = 0;
}; 

/** Defines a factory template, following Alexandrescu's pattern from 'Modern
 * C++ Design' */
template< typename _Type, typename _BaseType >
class Factory : public AbstractFactory<_BaseType>
{
public:
    explicit Factory() : AbstractFactory<_BaseType>() {}
    virtual ~Factory() {}
    virtual _BaseType* create(CogServer& cs) const { return new _Type(cs); }
    virtual const ClassInfo& info() const { return _Type::info(); }
}; 

/** Defines a single factory template to allow insert the same agent
 * multiple times in the Cogserver schedule */
template< typename _Type, typename _BaseType >
class SingletonFactory : public Factory<_Type, _BaseType>
{
public:
    explicit SingletonFactory() : Factory<_Type, _BaseType>() {}
    virtual ~SingletonFactory() {}
    virtual _BaseType* create(CogServer& cs) const {
        static _BaseType* inst =  new _Type(cs);
        return inst;
    }
};


/** @}*/
} // namespace opencog

#endif // _OPENCOG_FACTORY_H
