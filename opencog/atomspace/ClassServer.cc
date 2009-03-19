/*
 * opencog/atomspace/ClassServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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

#include "ClassServer.h"

#include <exception>
#include <functional>
#include <tr1/functional>
#include <algorithm>

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>

#include "atom_types.definitions"

using namespace opencog;
using namespace std::tr1::placeholders;

// ClassServer's class variables
Type ClassServer::nTypes = 0;
bool ClassServer::initialized = false;

std::vector< std::vector<bool> >                  ClassServer::inheritanceMap;
std::tr1::unordered_map<std::string, Type>        ClassServer::name2CodeMap;
std::tr1::unordered_map<Type, const std::string*> ClassServer::code2NameMap;

Type ClassServer::addType(Type parent, const std::string& name)
{
    // check if a type with this name already exists
    std::tr1::unordered_map<std::string, Type>::iterator it;
    if ((it = name2CodeMap.find(name)) != name2CodeMap.end()) {
        //logger().warn("Type \"%s\" has already been added (%d)", name.c_str(), it->second);
        if (inheritanceMap[parent][it->second] == false) {
            setParentRecursively(parent, it->second);
            //logger().warn("Type \"%s\" (%d) was not a parent of \"%s\" (%d) previously.", code2NameMap[parent]->c_str(), parent, name.c_str(), it->second);
        }
        return it->second;
    }

    // assign type code and increment type counter
    Type type = nTypes++;

    // resize inheritanceMap container
    inheritanceMap.resize(nTypes);

    std::for_each(inheritanceMap.begin(), inheritanceMap.end(),
                  std::tr1::bind(&std::vector<bool>::resize, _1, nTypes, false));

    inheritanceMap[type][type]   = true;
    setParentRecursively(parent, type);
    name2CodeMap[name]           = type;
    code2NameMap[type]           = &(name2CodeMap.find(name)->first);

    return type;
}

void ClassServer::setParentRecursively(Type parent, Type type) {
    inheritanceMap[parent][type] = true;
    for (Type i = 0; i < nTypes; ++i) {
        if ((inheritanceMap[i][parent]) && (i != parent)) {
            setParentRecursively(i, type);
        }
    }
}

void ClassServer::init(void)
{
    if (initialized) return;
    logger().info("Initializing ClassServer");

    #include "atom_types.inheritance"
}

unsigned int ClassServer::getNumberOfClasses()
{
    return nTypes;
}

bool ClassServer::isA(Type type, Type parent)
{
    return inheritanceMap[parent][type];
}

bool ClassServer::isDefined(const string& typeName)
{
    return name2CodeMap.find(typeName) != name2CodeMap.end();
}

Type ClassServer::getType(const string& typeName)
{
    std::tr1::unordered_map<std::string, Type>::iterator it = name2CodeMap.find(typeName);
    if (it == name2CodeMap.end()) {
        return NOTYPE;
    }
    return it->second;
}

const std::string& ClassServer::getTypeName(Type type)
{
    static std::string nullString = "";
    std::tr1::unordered_map<Type, const std::string*>::iterator it;
    if ((it = code2NameMap.find(type)) != code2NameMap.end())
        return *(it->second);
    return nullString;
}

