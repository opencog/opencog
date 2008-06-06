/*
 * src/AtomSpace/ClassServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

using namespace opencog;

std::vector<std::vector<bool> >& ClassServer::getMap()
{
    static std::vector<std::vector<bool> > map = ClassServer::init_map();
    return map;
}
ClassTypeHashMap* ClassServer::getClassType()
{
    static ClassTypeHashMap *class_type = ClassServer::init_type();
    return class_type;
}
ClassNameHashMap* ClassServer::getClassName()
{
    static ClassNameHashMap *class_name = ClassServer::init_name();
    return class_name;
}

/*
 * A function to automatically initialize the map with
 * all classes and super-classes.
 *
*/
void ClassServer::init_inheritance(std::vector<std::vector<bool> >& map, int parentClassNum, int classNum)
{
    map[parentClassNum][classNum] = true;
    //printf("ClassServer inheritance %d : %d \n",classNum,parentClassNum);
    for (int i = 0; i < NUMBER_OF_CLASSES; i++) {
        if (map[i][parentClassNum] && i != parentClassNum && !map[i][classNum])
            init_inheritance(map, i, classNum);
    }
}

/*
 * Mike says: I changed this function to make it easier to debug.
 * I noticed that a few of the original mappings were not completely
 * transitive (that is, A->B and B->C but not A->C).  I assume this was
 * a mistake.
 * Now you only have to indicate the direct parent class of a class.
 * But you now MUST insert every parent class before each of its descendents.
 * This could be made even simpler at the expense of efficiency.
 */
std::vector<std::vector<bool> > ClassServer::init_map()
{
    std::vector<std::vector<bool> > map(NUMBER_OF_CLASSES);
    // map will first receive false for all positions, except when the
    // first index is equal to the second since a type is always
    // assignable from itself
    for (int i = 0; i < NUMBER_OF_CLASSES; i++) {
        map[i].resize(NUMBER_OF_CLASSES);
        for (int j = 0; j < NUMBER_OF_CLASSES; j++) {
            map[i][j] = (i == j);
        }
    }

//=================================================================
// ATOMS
//
// NOTE: The order of the commands below is important. You MUST
// declare every parent class before its descendents. Otherwise
// some superclasses will be missing in the inheritance set of
// some classes.
//=================================================================

#include "type_inheritance.h"
    return map;
}



ClassTypeHashMap *ClassServer::init_type()
{
    static ClassTypeHashMap class_to_int;
    // getClassName/init_name is called to avoid duplicate strings
    getClassName();

    class_to_int.clear();
#include "type_classint.h"
    return &class_to_int;
}

ClassNameHashMap *ClassServer::init_name()
{
    static ClassNameHashMap int_to_class;

    int_to_class.clear();
#include "type_intclass.h"
    return &int_to_class;
}

Type* ClassServer::getChildren(Type type, int &n)
{
    int count = 0;
    // counts the number of subclasses of a given type
    for (int i = 0; i < NUMBER_OF_CLASSES; i++) {
        if ((getMap()[type][i]) && (i != type)) count++;
    }
    Type* answer = new Type[count*sizeof(Type)];

    n = count;
    for (int i = 0, j = 0; i < NUMBER_OF_CLASSES; i++) {
        if ((getMap()[type][i]) && (i != type)) {
            answer[j++] = i;
        }
    }
    return answer;
}

int ClassServer::getNumberOfClasses()
{
    return NUMBER_OF_CLASSES;
}

bool ClassServer::isAssignableFrom(Type super, Type sub)
{
    return getMap()[super][sub];
}

bool ClassServer::isDefined(const char *typeName)
{
    return (getType(typeName) < NUMBER_OF_CLASSES);
}

Type ClassServer::getType(const char *typeName)
{
    if (!typeName) return NOTYPE;

    ClassTypeHashMap::iterator it = getClassType()->find(typeName);

    if (it == getClassType()->end()) {
        return NOTYPE;
    }
    return it->second;
}

const char* ClassServer::getTypeName(Type type)
{
    int t = (int) type;
    if ((t < 0) || (t >=  NUMBER_OF_CLASSES)) {
        return NULL;
    }
    return (*getClassName())[type];
}

// TODO: Implement smarter mapping from atom types to BuiltInTypeHandle IDs?
Handle ClassServer::typeDesignatorHandle(Type T)
{
    return (Handle)T;
}
