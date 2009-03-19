#ifndef NodeIDMap_HEADER_GUARD_
#define NodeIDMap_HEADER_GUARD_

/*
 * Copyright 1999-2002,2004 The Apache Software Foundation.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


//
//  This file is part of the internal implementation of the C++ XML DOM.
//  It should NOT be included or used directly by application programs.
//
//  Applications should include the file <xercesc/dom/deprecated/DOM.hpp> for the entire
//  DOM API, or DOM_*.hpp for individual DOM classes, where the class
//  name is substituded for the *.
//




//
//  Class NodeIDMap is a hash table that is used in the implementation of
//   of DOM_Document::getElementsByID().
//
//  Why Yet Another HashTable implementation?  Becuase it can be significantly
//  smaller when tuned for this exact usage, and the generic RefHashTableOf
//  from the xerces utils project is not a paricularly good fit.
//
#include <xercesc/util/PlatformUtils.hpp>

XERCES_CPP_NAMESPACE_BEGIN


class AttrImpl;
class DOMString;


class NodeIDMap : public XMemory {
public:

    // Create a new hash table, sized to hold "initialSize"
    NodeIDMap(int initialSize,
              MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager);
                                   //  Entries.  It will automatically grow if need be.

    virtual ~NodeIDMap();

private:
    NodeIDMap(const NodeIDMap &other);   // No copy, assignement, comparison.
    NodeIDMap &operator = (const NodeIDMap &other);
    bool operator == (const NodeIDMap &other);

public:
    void  add(AttrImpl *attr);       // Add the specified attribute to the table.
    void  remove(AttrImpl *other);   // Remove the specified attribute.
                                           //   Does nothing if the node is not in the table.
    AttrImpl *find(const DOMString &ID);   // Find the attribute node in the table with this ID

private:
    void growTable();

private:
    AttrImpl      **fTable;
    unsigned int  fSizeIndex;              // Index of the current table size in the
                                           //   array of possible table sizes.
	unsigned int  fSize;                   // The current size of the table array
                                           //   (number of slots, not bytes.)
    unsigned int  fNumEntries;             // The number of entries used.
    unsigned int  fMaxEntries;             // The max number of entries to use before
                                           //   growing the table.
    MemoryManager* fMemoryManager;

};

XERCES_CPP_NAMESPACE_END

#endif
