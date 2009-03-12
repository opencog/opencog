/*
 * opencog/atomspace/ClassServer.h
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

#ifndef _OPENCOG_CLASS_SERVER_H
#define _OPENCOG_CLASS_SERVER_H

#include <vector>

#include <stdlib.h>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/atom_types.h>
#include <opencog/util/platform.h>

namespace opencog
{

/**
 * This class keeps track of the complete atom class hierarchy.
 * The current implementation is hardwired. Future versions may include
 * different structures based on run-time type identification.
 */
class ClassServer
{
private:

    /** Private default constructor for this class to make it abstract. */
    ClassServer() {}

    static Type nTypes;
    static bool initialized;

    static std::vector< std::vector<bool> >            inheritanceMap;
    static std::tr1::unordered_map<std::string, Type>  name2CodeMap;
    static std::tr1::unordered_map<Type, const std::string*> code2NameMap;

    static void setParentRecursively(Type parent, Type type);

public:

    static Type addType(Type parent, const std::string& name);
    static void init(void);

    /**
     * Stores the children types on the OutputIterator 'result'. Returns the
     * number of children types.
     */
    template<typename OutputIterator>
    static unsigned int getChildren(Type type, OutputIterator result)
    {
        unsigned int n_children = 0;
        for (Type i = 0; i < nTypes; ++i) {
            if (inheritanceMap[type][i] && (type != i)) {
                *(result++) = i;
                n_children++;
            }
        }
        return n_children;
    }

    /**
     * Returns the total number of classes in the system.
     *
     * @return The total number of classes in the system.
     */
    static unsigned int getNumberOfClasses();

    /**
     * Returns whether a given class is assignable from another.
     *
     * @param Super class.
     * @param Subclass.
     * @return Whether a given class is assignable from another.
     */
    static bool isA(Type super, Type sub);

    /**
     * Returns true if given class is a Link.
     *
     * @param class.
     * @return Whether a given class is Link.
     */
    static bool isLink(Type t) { return isA(t, LINK); }

    /**
     * Returns true if given class is a Node.
     *
     * @param class.
     * @return Whether a given class is Node.
     */
    static bool isNode(Type t) { return isA(t, NODE); }

    /**
     * Returns whether a class with name 'typeName' is defined.
     */
    static bool isDefined(const string& typeName);

    /**
     * Returns the type of a given class.
     *
     * @param Class type name.
     * @return The type of a givenn class.
     */
    static Type getType(const string& typeName);

    /**
     * Returns the string representation of a given atom type.
     *
     * @param Atom type code.
     * @return The string representation of a givenn class.
     */
    static const std::string& getTypeName(Type type);

    /**
     * The typeDesignator of T is the Handle which describes the type T
     * (eg. a ConceptNode whose name is the corresponding type name.)
     */
    static Handle typeDesignatorHandle(Type T);
};

} // namespace opencog

#endif // _OPENCOG_CLASS_SERVER_H
