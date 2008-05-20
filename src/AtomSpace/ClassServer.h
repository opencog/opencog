/*
 * src/AtomSpace/ClassServer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Written by $Authors_Name <$Authors_email>
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

#ifndef CLASSSERVER_H
#define CLASSSERVER_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "platform.h"
#include "classes.h"
#include "types.h"
#include "utils.h"

#include <stdlib.h>
#include <vector>

using std::vector;

/**
 * This class keeps track of the complete atom class hierarchy.
 * The current implementation is hardwired. Future versions may include
 * different structures based on run-time type identification.
 */
class ClassServer {

private:

   /** Private default constructor for this class to make it abstract. */
   ClassServer() {}
   static void init_inheritance(std::vector<std::vector<bool> >& map, int parentClassNum, int classNum);

public:
   static std::vector<std::vector<bool> >& getMap();
   static ClassTypeHashMap *getClassType();
   static ClassNameHashMap *getClassName();

   /** 
    * Initialize derived class mapping.
    *
    * @return A boolean matrix[NUMBER_OF_CLASSES][NUMBER_OF_CLASSES].
    */
   static std::vector<std::vector<bool> > init_map();

   /** 
    * Initialize class name to type mapping.
    *
    * @return A hash_map pointer to a ClassTypeHashMap.
    */
   static ClassTypeHashMap *init_type();

   /** 
    * Initialize type to class name mapping.
    *
    * @return A hash_map pointer to a ClassTypeHashMap.
    */
   static ClassNameHashMap *init_name();

   /** 
    * Returns an array containing an int, indicating the number of
    * subclasses of a given type, followed by an array with all
    * subclasses.
    *
    * @return An array containing an int, indicating the number of
    * subclasses of a given type, followed by an array with all
    * subclasses.
    */
   static Type* getChildren(Type type, int &n);

   /**
    * Returns the total number of classes in the system.
    *
    * @return The total number of classes in the system.
    */
   static int getNumberOfClasses();

   /**
    * Returns whether a given class is assignable from another.
    *
    * @param Super class.
    * @param Subclass.
    * @return Whether a given class is assignable from another.
    */
   static bool isAssignableFrom(Type super, Type sub);

   /**
    * Returns whether a given class is derived from another.
    *
    * @param Class type name.
    * @return Whether a given class is derived from another.
    */
   static bool isDefined(const char *typeName);

   /**
    * Returns the type of a given class.
    *
    * @param Class type name.
    * @return The type of a givenn class.
    */
   static Type getType(const char *typeName);

   /**
    * Returns the string representation of a given atom type.
    *
    * @param Atom type code.
    * @return The string representation of a givenn class.
    */
   static const char* getTypeName(Type type);

    /**
     * The typeDesignator of T is the Handle which describes the type T
     * (eg. a ConceptNode whose name is the corresponding type name.)
     */
    static Handle typeDesignatorHandle(Type T);
};

#endif
