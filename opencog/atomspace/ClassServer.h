/*
 * opencog/atomspace/ClassServer.h
 *
 * Copyright (C) 2011 by The OpenCog Foundation
 * All Rights Reserved
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

#include <stdlib.h>

#include <mutex>
#include <unordered_map>
#include <vector>

#include <boost/signal.hpp>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/atom_types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class ClassServer; 

typedef ClassServer* ClassServerFactory(void);

/**
 * This class keeps track of the complete atom class hierarchy.
 * The current implementation is hardwired. Future versions may include
 * different structures based on run-time type identification.
 */
typedef boost::signal<void (Type)> TypeSignal;
class ClassServer
{
private:

    /** Private default constructor for this class to make it a singleton. */
    ClassServer();

    mutable std::mutex type_mutex;
    mutable std::mutex signal_mutex;

    Type nTypes;

    std::vector< std::vector<bool> > inheritanceMap;
    std::vector< std::vector<bool> > recursiveMap;
    std::unordered_map<std::string, Type> name2CodeMap;
    std::unordered_map<Type, const std::string*> code2NameMap;
    TypeSignal _addTypeSignal;

    void setParentRecursively(Type parent, Type type);

public:
    /** Returns a new ClassServer instance */
    static ClassServer* createInstance(void);

    /** Adds a new atom type with the given name and parent type */
    Type addType(Type parent, const std::string& name);

    /** Provides ability to get type-added signals.
     * @warning methods connected to this signal must not call ClassServer::addType or
     * things will deadlock.
     */
    TypeSignal& addTypeSignal();

    /**
     * Stores the children types on the OutputIterator 'result'. Returns the
     * number of children types.
     */
    template<typename OutputIterator>
    unsigned int getChildren(Type type, OutputIterator result)
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

    template<typename OutputIterator>
    unsigned int getChildrenRecursive(Type type, OutputIterator result)
    {
        unsigned int n_children = 0;
        for (Type i = 0; i < nTypes; ++i) {
            if (recursiveMap[type][i] && (type != i)) {
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
    unsigned int getNumberOfClasses();

    /**
     * Returns whether a given class is assignable from another.
     *
     * @param super Super class.
     * @param sub Subclass.
     * @return Whether a given class is assignable from another.
     */
    bool isA(Type sub, Type super);
    bool isA_non_recursive(Type sub, Type super);

    /**
     * Returns true if given class is a valid atom type.
     *
     * @param t class.
     * @return Whether a given class is valid.
     */
    bool isValid(Type t) { return isA(t, ATOM); }

    /**
     * Returns true if given class is a Link.
     *
     * @param t class.
     * @return Whether a given class is Link.
     */
    bool isLink(Type t) { return isA(t, LINK); }

    /**
     * Returns true if given class is a Node.
     *
     * @param t class.
     * @return Whether a given class is Node.
     */
    bool isNode(Type t) { return isA(t, NODE); }

    /**
     * Returns whether a class with name 'typeName' is defined.
     */
    bool isDefined(const std::string& typeName);

    /**
     * Returns the type of a given class.
     *
     * @param typeName Class type name.
     * @return The type of a givenn class.
     */
    Type getType(const std::string& typeName);

    /**
     * Returns the string representation of a given atom type.
     *
     * @param type Atom type code.
     * @return The string representation of a givenn class.
     */
    const std::string& getTypeName(Type type);
};

/** Gets the singleton instance (following meyer's design pattern) */
ClassServer& classserver(ClassServerFactory* = ClassServer::createInstance);

/** @}*/
} // namespace opencog

#endif // _OPENCOG_CLASS_SERVER_H
