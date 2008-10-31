/*
 * opencog/atomspace/AtomTable.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
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

#include "AtomTable.h"

#include <set>

#include <stdlib.h>
#include <pthread.h>

#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CoreUtils.h>
#include <opencog/atomspace/HandleMap.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/StatisticsMonitor.h>
#include <opencog/atomspace/TLB.h>
extern "C" {
#include <opencog/atomspace/md5.h>
}
#include <opencog/util/Config.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

using namespace opencog;

/* By defining USE_ATOM_HASH_SET, AtomTable lookup performance is
 * improved by a hundredfold(!) or more(!) for large atom tables.
 * It does not appear to affect overall memory usage.  Disabling
 * this has been observed to lead to memory corruption; I guess
 * there's a bug somewhere when this is not defined.
 */
#define USE_ATOM_HASH_SET

AtomTable::AtomTable(bool dsa)
{
    useDSA = dsa;
    size = 0;
    atomSet = new AtomHashSet();

    // There are four indices. One for types, one for target types,
    // one for names and one for importance ranges. The typeIndex
    // is NUMBER_OF_CLASSES+2 because NOTYPE is NUMBER_OF_CLASSES+1
    // and typeIndex[NOTYPE] is asked for if a typename is misspelled.
    // (because ClassServer::getType() returns NOTYPE in this case).
    typeIndex.resize(ClassServer::getNumberOfClasses() + 2, Handle::UNDEFINED);
    targetTypeIndex.resize(ClassServer::getNumberOfClasses() + 2, Handle::UNDEFINED);
    nameIndex.resize(NAME_INDEX_SIZE, Handle::UNDEFINED);
    importanceIndex.resize(IMPORTANCE_INDEX_SIZE, Handle::UNDEFINED);
    predicateIndex.resize(MAX_PREDICATE_INDICES, Handle::UNDEFINED);
    predicateHandles.resize(MAX_PREDICATE_INDICES, Handle::UNDEFINED);
    predicateEvaluators.resize(MAX_PREDICATE_INDICES, NULL);
    numberOfPredicateIndices = 0;
    predicateHandles2Indices = new HandleMap<int>();

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_init(&iteratorsLock, NULL);
#endif

}

AtomTable::~AtomTable()
{
#ifdef USE_ATOM_HASH_SET
    // remove all atoms from AtomTable
    AtomHashSet::iterator it = atomSet->begin();

    while (it != atomSet->end()) {
        //logger().fine("Removing atom %s (atomSet size = %u)", (*it)->toString().c_str(), atomSet->size());
        remove(TLB::getHandle(*it), true);
        it = atomSet->begin();
    }
    atomSet->clear();
    delete (atomSet);
#endif
    delete (predicateHandles2Indices);
}

bool AtomTable::isCleared() const
{
//    tableId = id;
//    useDSA = dsa;
    if (size != 0) {
        //printf("AtomTable::size is not 0\n");
        return false;
    }

    if (atomSet->size() != 0) {
        //printf("AtomTable[%d]::atomSet is not empty. size =%d\n", tableId, atomSet->size());
        return false;
    }

    for (int i = 0; i < ClassServer::getNumberOfClasses(); i++) {
        if (typeIndex[i] != Handle::UNDEFINED) {
            //printf("typeIndex[%d] is not Handle::UNDEFINED\n", i);
            return false;
        }
        if (targetTypeIndex[i] != Handle::UNDEFINED) {
            //printf("targetTypeIndex[%d] is not Handle::UNDEFINED\n", i);
            return false;
        }
    }
    for (int i = 0; i < NAME_INDEX_SIZE; i++) {
        if (nameIndex[i] != Handle::UNDEFINED) {
            //printf("nameIndex[%d] is not Handle::UNDEFINED\n", i);
            return false;
        }
    }
    for (int i = 0; i < IMPORTANCE_INDEX_SIZE; i++) {
        if (importanceIndex[i] != Handle::UNDEFINED) {
            //printf("importanceIndex[%d] is not Handle::UNDEFINED\n", i);
            return false;
        }
    }
    for (int i = 0; i < MAX_PREDICATE_INDICES; i++) {
        if (predicateIndex[i] != Handle::UNDEFINED) {
            //printf("predicateIndex[%d] is not Handle::UNDEFINED\n", i);
            return false;
        }
        if (predicateHandles[i] != Handle::UNDEFINED) {
            //printf("predicateHandles[%d] is not Handle::UNDEFINED\n", i);
            return false;
        }
        if (predicateEvaluators[i] != 0) {
            //printf("predicateEvaluators[%d] is not Handle::UNDEFINED\n", i);
            return false;
        }
    }

    if (numberOfPredicateIndices != 0) {
        //printf("numberOfPredicateIndices is not 0\n");
        return false;
    }
    if (predicateHandles2Indices->getCount() != 0) {
        //printf("predicateHandles2Indices is not empty\n");
        return false;
    }
    for (unsigned int i = 0; i < iterators.size(); i++) {
        if (iterators[i]->hasNext()) {
            //printf("iterators[%d] is not empty\n", i);
            return false;
        }
    }
    return true;
}

void AtomTable::addPredicateIndex(Handle predicateHandle, PredicateEvaluator* evaluator)
throw (InvalidParamException)
{

    if (numberOfPredicateIndices > MAX_PREDICATE_INDICES) {
        throw InvalidParamException(TRACE_INFO,
                                    "AtomTable - Exceeded number of predicate indices = %d", MAX_PREDICATE_INDICES);
    }
    if (predicateHandles2Indices->contains(predicateHandle)) {
        throw InvalidParamException(TRACE_INFO,
                                    "AtomTable - There is already an index for predicate handle %p", predicateHandle.value());
    }

    // Ok, add it.
    predicateHandles2Indices->add(predicateHandle, numberOfPredicateIndices);
    predicateHandles[numberOfPredicateIndices] = predicateHandle;
    predicateEvaluators[numberOfPredicateIndices] = evaluator;
    numberOfPredicateIndices++;
}

void AtomTable::registerIterator(HandleIterator* iterator)
{

    lockIterators();
    iterators.push_back(iterator);
    unlockIterators();
}

void AtomTable::unregisterIterator(HandleIterator* iterator) throw (RuntimeException)
{

    lockIterators();

    std::vector<HandleIterator*>::iterator it = iterators.begin();
    while (it != iterators.end()) {
        if (*it == iterator) {
            iterators.erase(it);
            unlockIterators();
            return;
        } else {
            it++;
        }
    }

    unlockIterators();
    throw RuntimeException(TRACE_INFO, "could not unregister iterator");
}

unsigned int AtomTable::strHash(const char* name) const
{

    // this is a traditional hash algorithm that implements the MD5
    // XXX Why? Isn't MD5 a bit of overkill for this?
    // It'll just be slow, without adding much value.

    // special hash value for NULL names
    if (name == NULL) return 0;

    unsigned int hash = 0;
    MD5_CTX context;
    unsigned char digest[16];

    MD5Init(&context);
    MD5Update(&context, (const unsigned char*) name, strlen(name));
    MD5Final(digest, &context);

    hash = *((int*) digest);
    hash ^= *(((int*) digest) + 1);
    hash ^= *(((int*) digest) + 2);
    hash ^= *(((int*) digest) + 3);

    return (hash % (NAME_INDEX_SIZE - 1)) + 1;
}

inline unsigned int AtomTable::getNameHash(Atom* atom) const
{
    Node *nnn = dynamic_cast<Node *>(atom);
    if (NULL == nnn) return strHash(NULL);
    return strHash(nnn->getName().c_str());
}

unsigned int AtomTable::importanceBin(short importance)
{
    // STI is in range of [-32768, 32767] so adding 32768 puts it in
    // [0, 65535] which is the size of the index
    return importance + 32768;
}

float AtomTable::importanceBinMeanValue(unsigned int bin)
{
    return (float) ((((float) bin) + 0.5) / ((unsigned int) IMPORTANCE_INDEX_SIZE));
}

HandleEntry* AtomTable::makeSet(HandleEntry* set,
                                Handle head, int index) const
{
    while (TLB::isValidHandle(head)) {
        HandleEntry* entry = new HandleEntry(head);
        entry->next = set;
        set = entry;
        head = TLB::getAtom(head)->next(index);
    }

    return set;
}

Handle AtomTable::getTypeIndexHead(Type type) const
{
    return typeIndex[type];
}

Handle AtomTable::getTargetTypeIndexHead(Type type) const
{
    return targetTypeIndex[type];
}

Handle AtomTable::getNameIndexHead(const char* name) const
{
    return nameIndex[strHash(name)];
}

Handle AtomTable::getPredicateIndexHead(int index) const
{
    return predicateIndex[index];
}

PredicateEvaluator* AtomTable::getPredicateEvaluator(Handle gpnHandle) const
{
    PredicateEvaluator* result = NULL;
    if (predicateHandles2Indices->contains(gpnHandle)) {
        int index = predicateHandles2Indices->get(gpnHandle);
        result = predicateEvaluators[index];
    }
    return result;
}

HandleEntry* AtomTable::findHandlesByGPN(const char* gpnNodeName, VersionHandle vh) const
{
    //printf("AtomTable::findHandlesByGPN(%s)\n", gpnNodeName);
    // Get the GroundPredicateNode with such name
    Handle gpnHandle = getHandle(gpnNodeName, GROUNDED_PREDICATE_NODE);
    HandleEntry* result = findHandlesByGPN(gpnHandle);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::findHandlesByGPN(Handle gpnHandle, VersionHandle vh) const
{
    HandleEntry* result = NULL;
    if (TLB::isValidHandle(gpnHandle)) {
        //printf("AtomTable::findHandlesByGPN(): found gnpHandle = %p\n", gpnHandle);
        if (predicateHandles2Indices->contains(gpnHandle)) {
            int index = (int)((long) predicateHandles2Indices->get(gpnHandle));
            //printf("AtomTable::findHandlesByGPN(): found index %d for gpnHandle\n", index);
            result = makeSet(NULL, getPredicateIndexHead(index), PREDICATE_INDEX | index);
        }
    }
    result = HandleEntry::filterSet(result, vh);
    return result;
}

Handle AtomTable::getHandle(const char* name, Type type) const
{
    if (!ClassServer::isAssignableFrom(NODE, type)) {
        return Handle::UNDEFINED;
    }
#ifdef USE_ATOM_HASH_SET
    Node node(type, name); // alloc on stack, avoid memory frag
    return getHandle(&node);
#else
    // creates a set with all atoms whose names have the same hash value as
    // the name key
    HandleEntry* set = makeSet(NULL, getNameIndexHead(name), NAME_INDEX);

    // the set is filtered by name and type
    set = HandleEntry::filterSet(set, name, type, false);

    // if any atom is left on the set, there exists a matching atom which is
    // returned
    Handle result = Handle::UNDEFINED;
    if (set != NULL) {
        result = set->handle;
        delete set;
    }
    return result;
#endif
}

// This call is nearly identical to that above.
// This call signature avoids one string copy in the 
// that would otherwise occur oncall to getHandle
Handle AtomTable::getHandle(const Node *node) const
{
#ifdef USE_ATOM_HASH_SET
    AtomHashSet::const_iterator it = atomSet->find(node);
    Handle result = Handle::UNDEFINED;
    if (it != atomSet->end()) {
        const Atom* resultAtom = *it;
        result = TLB::getHandle(resultAtom);
    }
    return result;
#else
    return getHandle(node->getName().c_str(), node->getType());
#endif
}
HandleEntry* AtomTable::buildSet(Type type, bool subclass,
                                 Handle(AtomTable::*f)(Type) const,
                                 int index) const
{
    // Builds a set for the given type.
    HandleEntry* set = makeSet(NULL, (this->*f)(type), index);

    if (subclass) {
        // If subclasses are accepted, the subclasses are returned in the
        // array types.
        int n;
        Type *types = ClassServer::getChildren(type, n);

        //printf("Checking %d subclasses:\n", n);

        // for all subclasses found, a set is concatenated to the answer set
        for (int i = 0; i < n; i++) {
            //printf("%d\n", i);
            if (index && TARGET_TYPE_INDEX) {
                index = types[i] & TARGET_TYPE_INDEX;
            }

            set = makeSet(set, (this->*f)(types[i]), index);
        }
        //printf("\n");
        delete[](types);
    }

    return set;
}

HandleEntry* AtomTable::getHandleSet(Type type, bool subclass) const
{
    HandleEntry* set = buildSet(type, subclass,
                                &AtomTable::getTypeIndexHead, TYPE_INDEX);
    return set;
}

HandleEntry* AtomTable::getHandleSet(Type type, Type targetType,
                                     bool subclass, bool targetSubclass) const
{
    HandleEntry* set = buildSet(targetType, targetSubclass,
                                &AtomTable::getTargetTypeIndexHead,
                                TARGET_TYPE_INDEX | targetType);
    return HandleEntry::filterSet(set, type, subclass);
}

HandleEntry* AtomTable::getHandleSet(Handle handle, Type type,
                                     bool subclass) const
{
    HandleEntry* set = TLB::getAtom(handle)->getIncomingSet();
    if (set != NULL) set = set->clone();
    set = HandleEntry::filterSet(set, type, subclass);
    // Also filter links that do not belong to this table
    //set = HandleEntry::filterSet(set, tableId);
    return set;
}

Handle AtomTable::getHandle(const Link *link) const
{
    const std::vector<Handle>& handles = link->getOutgoingSet();
    for (int i = 0; i < link->getArity(); i++) {
        if(false == TLB::isValidHandle(handles[i])) return Handle::UNDEFINED;
    }

    AtomHashSet::const_iterator it = atomSet->find(link);
    Handle h = Handle::UNDEFINED;
    if (it != atomSet->end()) {
        h = TLB::getHandle(*it);
    }
    return h;
}

HandleEntry* AtomTable::getHandleSet(const std::vector<Handle>& handles,
                                     Type* types,
                                     bool* subclasses,
                                     Arity arity,
                                     Type type,
                                     bool subclass) const
{
//printf("AtomTable::getHandleSet()\n");

#ifdef USE_ATOM_HASH_SET
    // Check if it is the special case of looking for an specific atom
    if (ClassServer::isAssignableFrom(LINK, type) && 
        (arity == 0 || !handles.empty()))
    {
        //printf("special case\n");
        bool hasAllHandles = true;
        for (int i = 0; hasAllHandles && i < arity; i++) {
            hasAllHandles = TLB::isValidHandle(handles[i]);
        }
        //printf("hasAllHandles = %d, subclass = %d\n", hasAllHandles, subclass);
        if (hasAllHandles && !subclass) {
            //printf("building link for lookup: type = %d, "
            // "handles.size() = %d\n", type, handles.size());
            Link link(type, handles); // local var on stack, avoid malloc
            AtomHashSet::iterator it = atomSet->find(&link);
            Handle h = Handle::UNDEFINED;
            if (it != atomSet->end()) {
                h = TLB::getHandle(*it);
            }
            HandleEntry* result = NULL;
            if (TLB::isValidHandle(h)) {
                result = new HandleEntry(h);
            }
            //cprintf(NORMAL, "Returning HandleSet by using atom hash_set!\n");
            return result;
        }
    }
#endif

    if (ClassServer::isAssignableFrom(LINK, type) && (arity == 0)) {
        HandleEntry* result = getHandleSet(type, subclass);
        result = HandleEntry::filterSet(result, arity);
        return result;
    }

    std::vector<HandleEntry*> sets(arity, NULL);

    int countdown = 0;

    // builds a set for each element in the outgoing set. Empty sets are
    // counted to be removed a posteriori
    for (int i = 0; i < arity; i++) {
        if ((!handles.empty()) && TLB::isValidHandle(handles[i])) {
            sets[i] = TLB::getAtom(handles[i])->getIncomingSet()->clone();
            sets[i] = HandleEntry::filterSet(sets[i], handles[i], i, arity);
            // Also filter links that do not belong to this table
            //sets[i] = HandleEntry::filterSet(sets[i], tableId);
            if (sets[i] == NULL) {
                for (int j = 0; j < i; j++) {
                    delete sets[j];
                }
                return NULL;
            }
        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            bool sub = subclasses == NULL ? false : subclasses[i];
            sets[i] = getHandleSet(type, types[i], subclass, sub);
            // Also filter links that do not belong to this table
            //sets[i] = HandleEntry::filterSet(sets[i], tableId);
            if (sets[i] == NULL) {
                for (int j = 0; j < i; j++) {
                    delete sets[j];
                }
                return NULL;
            }
        } else {
            countdown++;
        }
    }

    int newLength = arity;
    // if the empty set counter is not zero, removes them by shrinking the
    // list of sets
    if (countdown > 0) {
        //printf("newset allocated size = %d\n", (arity - countdown));
        // TODO: Perhaps it's better to simply erase the NULL entries of the sets
        std::vector<HandleEntry*> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i] != NULL) {
                newset.push_back(sets[i]);
            }
        }
        sets = newset;
    }
    //printf("newLength = %d\n", newLength);

    if ((type != ATOM) || (!subclass)) {
        for (int i = 0; i < newLength; i++) {
            // filters by type and subclass in order to remove unwanted elements.
            // This is done before the intersection method to reduce the number of
            // elements being passed (intersection uses qsort, which is n log n)
            sets[i] = HandleEntry::filterSet(sets[i], type, subclass);
        }
    }

    // computes the intersection of all non-empty sets
    HandleEntry* set = HandleEntry::intersection(sets);
    // TODO: Why not move this filtering to the begining... Pehaps it will filter more before the intersection (which seems to be the most expensive operation)
    // filters the answer set for every type in the array of target types
    if (types != NULL) {
        for (int i = 0; i < arity; i++) {
            if (types[i] != NOTYPE) {
                bool sub = subclasses == NULL ? false : subclasses[i];
                set = HandleEntry::filterSet(set, types[i], sub, i, arity);
            }
        }
    }

    return set;
}

HandleEntry* AtomTable::getHandleSet(const char* name, Type type, bool subclass) const
{
    // a list of the given names is built.
    HandleEntry* set = makeSet(NULL, getNameIndexHead(name), NAME_INDEX);

    // then the undesired names, because of a hash table conflict, are removed
    // from the list by filtering it.
    set = HandleEntry::filterSet(set, name);
    return HandleEntry::filterSet(set, type, subclass);
}

HandleEntry* AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass) const
{

    // Gets the exact atom with the given name and type, in any AtomTable.
    Handle handle = getHandle(targetName, targetType);

    HandleEntry* result = NULL;
    // then, if the atom returend is valid, the list with the given target name
    // and types will be returned.
    if (TLB::isValidHandle(handle)) {
        result = getHandleSet(handle, type, subclass);
    }
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass) const throw (RuntimeException)
{
    //printf("getHandleSet begin\n");

    std::vector<HandleEntry*> sets(arity, NULL);

    int countdown = 0;
    // a list for each array of names is built. Then, it's filtered by the
    // name (to avoid hash conflicts) and by the correspondent type in the
    // array of types.
    for (int i = 0; i < arity; i++) {
        //printf("getHandleSet: arity %d\n", i);
        bool sub = subclasses == NULL ? false : subclasses[i];
        if ((names != NULL) && (names[i] != NULL)) {
            if ((types != NULL) && (types[i] != NOTYPE)) {
                sets[i] = getHandleSet(names[i], types[i], type, subclass);
                if (sub) {
                    // if subclasses are accepted, the subclasses are returned in the
                    // array types.
                    int n;

                    Type *subTypes = ClassServer::getChildren(types[i], n);

                    // for all subclasses found, a set is concatenated to the answer set
                    for (int j = 0; j < n; j++) {
                        HandleEntry *subSet = getHandleSet(names[i], subTypes[j], type, subclass);
                        sets[i] = HandleEntry::concatenation(sets[i], subSet);
                    }
                    delete[](subTypes);
                }
                sets[i] = HandleEntry::filterSet(sets[i], names[i], types[i], sub, i, arity);
            } else {
                for (int j = 0; j < i; j++) {
                    delete sets[j];
                }
                throw RuntimeException(TRACE_INFO, "Cannot make this search using only target name!\n");
            }
        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            sets[i] = getHandleSet(type, types[i], subclass, sub);
            sets[i] = HandleEntry::filterSet(sets[i], types[i], sub, i, arity);
        } else {
            countdown++;
        }
    }

    // if the empty set counter is not zero, removes them by shrinking the
    // list of sets
    if (countdown > 0) {
        //printf("newset allocated size = %d\n", (arity - countdown));
        // TODO: Perhaps it's better to simply erase the NULL entries of the sets
        std::vector<HandleEntry*> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i] != NULL) {
                newset.push_back(sets[i]);
            }
        }
        sets = newset;
    }

    /*
    for (int i = 0; i < arity; i++) {
        printf("arity %d\n:", i);
        for (HandleEntry* it = sets[i]; it != NULL; it = it->next) {
            printf("\t%ld: %s\n", it->handle, TLB::getAtom(it->handle)->toString().c_str());
        }
        printf("\n");
    }
    */
    // the intersection is made for all non-empty sets, and then is filtered
    // by the optional specified type. Also, if subclasses are not accepted,
    // it will not pass the filter.
    //printf("getHandleSet: about to call intersection\n");
    HandleEntry* set = HandleEntry::intersection(sets);
    //printf("getHandleSet: about to call filterSet\n");
    //return  HandleEntry::filterSet(set, type, subclass); // This filter redundant, since all getHandleSet above uses type and subclass
    return  set;
}

HandleEntry* AtomTable::getHandleSet(AttentionValue::sti_t lowerBound, AttentionValue::sti_t upperBound) const
{

    // the indice for the lower bound and upper bound lists is returned.
    int lowerBin = importanceBin(lowerBound);
    int upperBin = importanceBin(upperBound);

    // the list of atoms with its importance equal to the lower bound is
    // returned.
    HandleEntry* set = makeSet(NULL, importanceIndex[lowerBin], IMPORTANCE_INDEX);

    // for the lower bound and upper bound index, the list is filtered, because
    // there may be atoms that have the same importanceIndex and whose
    // importance is lower than lowerBound or bigger than upperBound.
    set = HandleEntry::filterSet(set, lowerBound, upperBound);

    if (lowerBin == upperBin) {
        // If both lower and upper bounds are in the same bin,
        // it can ans must return the already built set.
        // Otherwise, it will duplicate entries when concatening the upper set latter.
        return set;
    }

    // for every index within lowerBound and upperBound, the list is
    // concatenated.
    while (++lowerBin < upperBin) {
        set = makeSet(set, importanceIndex[lowerBin], IMPORTANCE_INDEX);
    }

    // the list for the upperBin index is built and filtered.
    HandleEntry* uset = makeSet(NULL, importanceIndex[upperBin], IMPORTANCE_INDEX);
    uset = HandleEntry::filterSet(uset, lowerBound, upperBound);

    // then the two lists built are concatenated.
    return HandleEntry::concatenation(uset, set);
}

HandleEntry* AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass) const
{
    return getHandleSet((const char**) NULL, types, subclasses, arity, type, subclass);
}


void AtomTable::merge(Atom *original, Atom *copy)
{
    original->merge(copy);
    delete copy;
}

Handle AtomTable::add(Atom *atom, bool dont_defer_incoming_links) throw (RuntimeException)
{
    if (atom->getAtomTable() != NULL) {
        // Atom is already inserted
        return  TLB::getHandle(atom);
    }
    Handle existingHandle = Handle::UNDEFINED;
    Node * nnn = dynamic_cast<Node *>(atom);
    Link * lll = dynamic_cast<Link *>(atom);
    if (nnn) {
        // checks if the node handle already exists.
        existingHandle = getHandle(nnn);
    } else if (lll) {
        existingHandle = getHandle(lll);
    }

    if (TLB::isValidHandle(existingHandle)) {
        //printf("Merging existing Atom with the Atom being added ...\n");
        merge(TLB::getAtom(existingHandle), atom);

        // emit add atom signal
        _addAtomSignal(existingHandle);

        return existingHandle;
    }

    // New atom, its Handle will be stored in the AtomTable
    // Increment the size of the table
    size++;

#ifdef USE_ATOM_HASH_SET
    // Adds to the hash_set
    // logger().debug("Inserting atom %p in hash_set (type=%d, hashCode=%d)\n", atom, atom->getType(), atom->hashCode());
    atomSet->insert(atom);
    //logger().debug("[AtomTable::add] atomSet->insert(%p) => size = %d\n", atom, atomSet->size());
#endif

    // Checks for null outgoing set members.
    Link *link = dynamic_cast<Link *>(atom);
    if (link) {
        const std::vector<Handle>& ogs = link->getOutgoingSet();
        for (int i = link->getArity() - 1; i >= 0; i--) {
            if (TLB::isInvalidHandle(ogs[i])) {
                throw RuntimeException(TRACE_INFO,
                                       "AtomTable - Attempting to insert link with invalid (null) outgoing members");
            }
        }
    }

    Handle handle = TLB::getHandle(atom);
    if (TLB::isInvalidHandle(handle)) handle = TLB::addAtom(atom);

    // Inserts atom in the type index of its type (as head of the list).
    Type type = atom->getType();
    atom->setNext(TYPE_INDEX, typeIndex[type]);
    typeIndex[type] = handle;

    // If the atom is a link, the targetIndexTypes list is built. Then, from
    // the atom's arity, it will be checked how many targetTypes are distinct.
    int distinctSize;
    Type* targetTypes = atom->buildTargetIndexTypes(&distinctSize);
    if (distinctSize > 0) {
        // Here, the atom is placed on each target index list.
        Handle* targetIndices = new Handle[distinctSize];
        for (int i = 0; i < distinctSize; i++) {
            // Insert it as head of the corresponding list
            targetIndices[i] = targetTypeIndex[targetTypes[i]];
            targetTypeIndex[targetTypes[i]] = handle;
        }
        atom->setNextTargetTypeIndex(targetIndices);
    }
    delete[](targetTypes);

    // The atom is placed on its proper name index list.
    unsigned int nameHash = getNameHash(atom);
    atom->setNext(NAME_INDEX, nameIndex[nameHash]);
    nameIndex[nameHash] = handle;

    // The atom is placed on its proper importance index list.
    int bin = importanceBin(atom->getAttentionValue().getSTI());
    //logger().debug("adding handle %p with sti %d into importanceIndex (bin = %d)\n", handle, atom->getAttentionValue().getSTI(), bin);
    atom->setNext(IMPORTANCE_INDEX, importanceIndex[bin]);
    importanceIndex[bin] = handle;

    // Checks Atom against predicate indices and inserts it if needed
    for (int i = 0; i < numberOfPredicateIndices; i++) {
        // printf("Processing predicate index %d\n");
        PredicateEvaluator* evaluator = predicateEvaluators[i];
        // printf("Evaluating handle %p with PredicateEvaluator  = %p\n", handle, evaluator);
        if (evaluator->evaluate(handle)) {
            // printf("ADDING HANDLE %p TO THE PREDICATE INDEX %d (HEAD = %p)\n", handle, i, getPredicateIndexHead(i));
            atom->addNextPredicateIndex(i, predicateIndex[i]);
            predicateIndex[i] = handle; // adds as head of the linked list

#ifdef DEBUG_PRINTING
            printf("HEAD AFTER INSERTION = %p\n", getPredicateIndexHead(i));
            HandleEntry* indexedHandles = makeSet(NULL, getPredicateIndexHead(i), PREDICATE_INDEX | i);
            printf("Handles in the index %d: \n", i);
            while (indexedHandles != NULL) {
                printf("%p (%d)\t", indexedHandles->handle, TLB::getAtom(indexedHandles->handle)->getType());
                indexedHandles = indexedHandles->next;
            }
            printf("\n");
#endif /* DEBUG_PRINTING */
        }
    }

    // Updates incoming set of all targets.
    if (dont_defer_incoming_links && (link != NULL)) {
        for (int i = 0; i < link->getArity(); i++) {
            link->getOutgoingAtom(i)->addIncomingHandle(handle);
        }
    }

    // updates statistics
    //float heat = atom->getHeat();
    //atom->rawSetHeat(0);
    //atom->setHeat(heat);

    atom->setAtomTable(this);

    if (useDSA) {
        StatisticsMonitor::getInstance()->add(atom);
    }

    // emit add atom signal
    _addAtomSignal(handle);

    logger().debug("[AtomTable] add: %p", handle.value());

    return handle;
}

bool AtomTable::updateImportanceIndex(Atom* atom, int bin)
{

    // current receives the first element of the list that the atom is in.
    Handle current = importanceIndex[bin];
    Handle wanted = TLB::getHandle(atom);

    // checks if current is valid.
    if (TLB::isInvalidHandle(current)) {
        return(false);
    }
    // here is checked if the atom is on the first position of its importance
    // index list.
    if (current == wanted) {
        // if so, the new first element will be the next one.
        importanceIndex[bin] = atom->next(IMPORTANCE_INDEX);
    } else {
        // if not, the list will be scanned until the atom is found.
        Handle p;
        while ((p = TLB::getAtom(current)->next(IMPORTANCE_INDEX)) != wanted) {
            current = p;
            if (TLB::isInvalidHandle(p)) {
                return(false);
            }
        }
        TLB::getAtom(current)->setNext(IMPORTANCE_INDEX, TLB::getAtom(wanted)->next(IMPORTANCE_INDEX));
    }

    // the atom is placed on the last position of the new list.
    atom->setNext(IMPORTANCE_INDEX, importanceIndex[importanceBin(atom->getAttentionValue().getSTI())]);
    importanceIndex[importanceBin(atom->getAttentionValue().getSTI())] = TLB::getHandle(atom);
    return(true);
}

int AtomTable::getSize() const
{
    return(size);
}

void AtomTable::log(Logger& logger, Type type, bool subclass) const
{
#ifdef USE_ATOM_HASH_SET
    for (AtomHashSet::const_iterator it = atomSet->begin(); it != atomSet->end(); it++) {
        const Atom* atom = *it;
        bool matched = (subclass && ClassServer::isAssignableFrom(type, atom->getType())) || type == atom->getType();
        if (matched) logger.debug("%d: %s", TLB::getHandle(atom).value(), atom->toString().c_str());
    }
#else
    logger().error("AtomTable::log() method is not implemented when USE_ATOM_HASH_SET is disabled");
#endif

}

void AtomTable::print(std::ostream& output, Type type, bool subclass) const
{
#ifdef USE_ATOM_HASH_SET
    for (AtomHashSet::const_iterator it = atomSet->begin(); it != atomSet->end(); it++) {
        const Atom* atom = *it;
        bool matched = (subclass && ClassServer::isAssignableFrom(type, atom->getType())) || type == atom->getType();
        if (matched) output << TLB::getHandle(atom) << ": " << atom->toString() << endl;
    }
#else
    output << "[ERROR] AtomTable::print() method is not implemented when USE_ATOM_HASH_SET is disabled" << endl;
#endif
}

HandleEntry* AtomTable::extractOld(Handle handle, bool recursive)
{
    HandleEntry* result = NULL;
    Atom *atom = TLB::getAtom(handle);

    if (atom->getFlag(REMOVED_BY_DECAY)) return result;
    else atom->setFlag(REMOVED_BY_DECAY, true);

    // if recursive-flag is set, also extract all the links in the atom's incoming set
    if (recursive) {
        for (HandleEntry* in = atom->getIncomingSet(); in != NULL; in = in->next) {
            Atom *a = TLB::getAtom(in->handle);
            if (a->isOld(minSTI)) {
                result = HandleEntry::concatenation(extractOld(in->handle, true), result);
            }
        }
    }

    // only return if there is at least one incoming atom that is not marked for
    // removal by decay
    for (HandleEntry* in = atom->getIncomingSet(); in != NULL; in = in->next) {
        if (TLB::getAtom(in->handle)->getFlag(REMOVED_BY_DECAY) == false) {
            atom->setFlag(REMOVED_BY_DECAY, false);
            return result;
        }
    }
    result = HandleEntry::concatenation(new HandleEntry(handle), result);
    return result;
}

HandleEntry* AtomTable::extract(Handle handle, bool recursive)
{
    // TODO: Check if this atom is really inserted in this AtomTable and get the
    // exact Atom object
    HandleEntry* result = NULL;

    Atom *atom = TLB::getAtom(handle);
    if (atom->isMarkedForRemoval()) return result;
    atom->markForRemoval();

    // if recursive-flag is set, also extract all the links in the atom's incoming set
    if (recursive) {
        /* we need to make a copy of the incoming set because the 'incoming set'
         * container is actually a list, so the same link may appear twice in an
         * incoming set. Hopefully we'll eventually use the right container */
        std::set<Handle> is;
        for (HandleEntry* in = atom->getIncomingSet(); in != NULL; in = in->next)
            is.insert(in->handle);

        std::set<Handle>::iterator is_it;
        for (is_it = is.begin(); is_it != is.end(); ++is_it) {
            logger().debug("[AtomTable::extract] incoming set: %s", TLB::isValidHandle(*is_it) ? TLB::getAtom(*is_it)->toString().c_str() : "INVALID HANDLE");
            if (TLB::getAtom(*is_it)->isMarkedForRemoval() == false) {
                logger().debug("[AtomTable::extract] marked for removal is false");
                result = HandleEntry::concatenation(extract(*is_it, true), result);
            }
        }
    }
    if (atom->getIncomingSet()) {
        logger().warn("AtomTable.extract(): attempting to extract atom with non-empty incoming set: %s\n", atom->toShortString().c_str());
        for (HandleEntry* it = atom->getIncomingSet(); it != NULL; it = it->next) {
            logger().warn("\t%s\n", TLB::getAtom(it->handle)->toShortString().c_str());
        }
        atom->unsetRemovalFlag();
        return result;
    }

    //decrements the size of the table
    size--;

#ifdef USE_ATOM_HASH_SET
    atomSet->erase(atom);
#endif

    // updates all global statistics regarding the removal of this atom
    if (useDSA) StatisticsMonitor::getInstance()->remove(atom);

    // remove from indices
    removeFromIndex(atom, typeIndex, TYPE_INDEX, atom->getType());
    removeFromIndex(atom, nameIndex, NAME_INDEX, getNameHash(atom));
    removeFromIndex(atom, importanceIndex, IMPORTANCE_INDEX, importanceBin(atom->getAttentionValue().getSTI()));
    removeFromTargetTypeIndex(atom);
    removeFromPredicateIndex(atom);

    // remove from incoming sets
    Link* link = dynamic_cast<Link*>(atom);
    if (link) {
        for (int i = 0; i < link->getArity(); i++) {
            Atom* target = link->getOutgoingAtom(i);
            target->removeIncomingHandle(handle);
        }
    }

    // remove from iterators
    lockIterators();
    for (unsigned int i = 0; i < iterators.size(); i++) {
        // TODO: CAN THIS REALLY BE CALLED AFTER THE ATOM HAS BEEN REMOVED FROM
        // TYPE INDEX ALREADY ?
        removeFromIterator(atom, iterators[i]);
    }
    unlockIterators();

    return HandleEntry::concatenation(new HandleEntry(handle), result);
}

bool AtomTable::remove(Handle handle, bool recursive)
{
    HandleEntry* extractedHandles = extract(handle, recursive);
    if (extractedHandles) {
        removeExtractedHandles(extractedHandles);
        return true;
    }
    return false;
}

void AtomTable::removeExtractedHandles(HandleEntry* extractedHandles)
{
    if (extractedHandles == NULL) return;

    // We must to iterate from the end to the begining of the list of atoms so that 
    // link's target atoms are not removed before the link   
    HandleSeq hs;
    extractedHandles->toHandleVector(hs);
    delete extractedHandles;

    for (HandleSeq::reverse_iterator it = hs.rbegin(); it < hs.rend(); ++it) {
        // emit remove atom signal
        _removeAtomSignal(*it);

        Atom* atom = TLB::getAtom(*it);
        TLB::removeAtom(atom);
        delete atom;
    }
}

void AtomTable::removeFromIndex(Atom *victim,
                                std::vector<Handle>& index,
                                int indexID, int headIndex)
throw (RuntimeException)
{
    //logger().fine("AtomTable::removeFromIndex(): index.size() = %d, indexId = %d(%x), headIndex = %d(%x)", index.size(), indexID, indexID, headIndex, headIndex);
    Handle victimHandle = TLB::getHandle(victim);
    //logger().fine("victim = %s", victim?victim->toString().c_str():"NULL");

    Handle p = index[headIndex];
    Handle q = Handle::UNDEFINED;
    while (p != victimHandle) {
        if (TLB::isInvalidHandle(p)) {
            throw RuntimeException(TRACE_INFO,
                                   "AtomTable - Unable to remove atom. NULL atom at index 0x%X.", indexID);
        }
        Atom *patom = TLB::getAtom(p);
        //logger().fine("Next atom in index = %s", patom?patom->toString().c_str():"NULL");
        q = p;
        p = patom->next(indexID);
    }

    //cprintf(DEBUG,"removeFromIndex(): found position in the index\n");

    if (victimHandle == index[headIndex]) {
        index[headIndex] = victim->next(indexID);
    } else {
        Atom *qatom = TLB::getAtom(q);
        Atom *patom = TLB::getAtom(p);
        qatom->setNext(indexID, patom->next(indexID));
    }

    victim->setNext(indexID, Handle::UNDEFINED);
}

void AtomTable::removeFromTargetTypeIndex(Atom *atom)
{
    //logger().fine("AtomTable::removeFromTargetTypeIndex(%p)", atom);

    int arraySize;
    Type *types = atom->buildTargetIndexTypes(&arraySize);

    for (int i = 0; i < arraySize; i++) {
        removeFromIndex(atom, targetTypeIndex, TARGET_TYPE_INDEX | types[i], types[i]);
    }

    delete[](types);
}

void AtomTable::removeFromPredicateIndex(Atom *atom)
{
    if (!atom->hasPredicateIndexInfo()) {
        //logger().fine("removeFromPredicateIndex(%p): No predicate index info", atom);
        return;
    }
    //logger().fine("removeFromPredicateIndex(%p): has predicate index info", atom);

    int arraySize;
    int *predicateIndices = atom->buildPredicateIndices(&arraySize);

    //cprintf(DEBUG,"Found %d predicate indices\n", arraySize);
    for (int i = 0; i < arraySize; i++) {
        //cprintf(DEBUG,"removing from index %d => %p\n", predicateIndices[i], PREDICATE_INDEX | predicateIndices[i]);
        removeFromIndex(atom, predicateIndex, PREDICATE_INDEX | predicateIndices[i], predicateIndices[i]);
    }

    delete[](predicateIndices);
}

void AtomTable::decayShortTermImportance()
{
    for (unsigned int band = 0; band < (unsigned int) IMPORTANCE_INDEX_SIZE; band++) {
        Handle current = importanceIndex[band];
        while (TLB::isValidHandle(current)) {
            Atom* atom = TLB::getAtom(current);
            Handle next = atom->next(IMPORTANCE_INDEX);

            // update sti
            atom->getAVPointer()->decaySTI();

            // update importanceIndex
            // potential optimization: if we may reliably assume that all atoms
            // decrease the sti by one unit (i.e. --sti), we could update the
            // indexes with "importanceIndex[band - 1] = importanceIndex[band];"
            unsigned int newBand = AtomTable::importanceBin(atom->getAttentionValue().getSTI());
            removeFromIndex(atom, importanceIndex, IMPORTANCE_INDEX, band);
            atom->setNext(IMPORTANCE_INDEX, importanceIndex[newBand]);
            importanceIndex[newBand] = current;

            current = next;
        }
    }

    // cache the minSTI
    minSTI = config().get_int("MIN_STI");
    unsigned int lowerStiBand = importanceBin(minSTI);
    HandleEntry* oldAtoms = NULL;

    for (unsigned int band = 0; band <= lowerStiBand; band++) {
        Handle current = importanceIndex[band];
        while (TLB::isValidHandle(current)) {
            Atom* atom = TLB::getAtom(current);
            Handle next = atom->next(IMPORTANCE_INDEX);
            // remove it if too old
            if (atom->isOld(minSTI))
                oldAtoms = HandleEntry::concatenation(extractOld(current, true), oldAtoms);
            current = next;
        }
    }

    // clone is copied to the real importance index lists.
    if (oldAtoms) clearIndexesAndRemoveAtoms(oldAtoms);
}

void AtomTable::removeMarkedAtomsFromIndex(std::vector<Handle>& index, int indexID)
{
    //visit all atoms in index
    for (unsigned int i = 0; i < index.size(); i++) {
        Handle p = index[i];
        Handle q = Handle::UNDEFINED;
        while (!TLB::isInvalidHandle(p)) {
            Atom *patom = TLB::getAtom(p);
            //found marked element
            if (patom->getFlag(REMOVED_BY_DECAY)) {
                //search next valid atom to put in index
                while (!TLB::isInvalidHandle(p) && patom->getFlag(REMOVED_BY_DECAY)) {
                    p = patom->next(indexID);
                    //clear old index
                    patom->setNext(indexID, Handle::UNDEFINED);
                    patom = TLB::getAtom(p);
                }
                // it is the first element of list
                if (TLB::isInvalidHandle(q)) index[i] = p;
                else TLB::getAtom(q)->setNext(indexID, p);
            } else {
                //save previous element in q
                q = p;
                p = patom->next(indexID);
            }
        }
    }
}

void AtomTable::removeMarkedAtomsFromMultipleIndex(std::vector<Handle>& index, int indexID)
{
    //visit all atoms in index
    for (unsigned int i = 0; i < index.size(); i++) {
        int targetIndexID = indexID | i;
        Handle p = index[i];
        Handle q = Handle::UNDEFINED;
        while (!TLB::isInvalidHandle(p)) {
            Atom *patom = TLB::getAtom(p);
            //found marked element
            if (patom->getFlag(REMOVED_BY_DECAY)) {
                //search next valid atom to put in index
                while (!TLB::isInvalidHandle(p) && patom->getFlag(REMOVED_BY_DECAY)) {
                    p = patom->next(targetIndexID);
                    //clear old index
                    patom->setNext(targetIndexID, Handle::UNDEFINED);
                    patom = TLB::getAtom(p);
                }
                // it is the first element of list
                if (TLB::isInvalidHandle(q)) index[i] = p;
                else TLB::getAtom(q)->setNext(targetIndexID, p);
            } else {
                //save previous element in q
                q = p;
                p = patom->next(targetIndexID);
            }
        }
    }

}

void AtomTable::clearIndexesAndRemoveAtoms(HandleEntry* extractedHandles)
{
    if (extractedHandles == NULL) return;

    // remove from indices
    removeMarkedAtomsFromIndex(importanceIndex, IMPORTANCE_INDEX);
    removeMarkedAtomsFromIndex(typeIndex, TYPE_INDEX);
    removeMarkedAtomsFromIndex(nameIndex, NAME_INDEX);
    removeMarkedAtomsFromIndex(importanceIndex, IMPORTANCE_INDEX);
    removeMarkedAtomsFromMultipleIndex(targetTypeIndex, TARGET_TYPE_INDEX);
    removeMarkedAtomsFromMultipleIndex(predicateIndex, PREDICATE_INDEX);

    for (HandleEntry* curr = extractedHandles; curr != NULL; curr = curr->next) {
        Handle h = curr->handle;
        Atom* atom = TLB::getAtom(h);
#ifdef USE_ATOM_HASH_SET
        //Extracts atom from hash_set
        atomSet->erase(atom);
#endif
        // update the AtomTable's size
        size--;

        if (useDSA)
            // updates all global statistics regarding the removal of this atom
            StatisticsMonitor::getInstance()->remove(atom);

        // remove from incoming sets
        for (int i = 0; i < atom->getArity(); i++) {
            Atom *outgoing = atom->getOutgoingAtom(i);
            if (outgoing)
                outgoing->removeIncomingHandle(h);
        }

        // remove from iterators
        lockIterators();
        for (unsigned int i = 0; i < iterators.size(); i++) {
            // TODO: CAN THIS REALLY BE CALLED AFTER THE ATOM HAS BEEN
            // REMOVED FROM TYPE INDEX ALREADY ?
            removeFromIterator(atom, iterators[i]);
        }
        unlockIterators();
    }

    removeExtractedHandles(extractedHandles);
}

void AtomTable::removeFromIterator(Atom *atom, HandleIterator *iterator)
{
    if (iterator->currentHandle == TLB::getHandle(atom)) {
        iterator->next();
    }
}
void AtomTable::lockIterators()
{
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_lock(&iteratorsLock);
#endif
}

void AtomTable::unlockIterators()
{
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_unlock(&iteratorsLock);
#endif
}


HandleIterator* AtomTable::getHandleIterator()
{
    return new HandleIterator(this, (Type)ATOM, true);
}

HandleIterator* AtomTable::getHandleIterator(Type type, bool subclass, VersionHandle vh)
{
    return new HandleIterator(this, type, subclass, vh);
}

Handle AtomTable::getImportanceIndexHead(int i) const
{
    return importanceIndex[i];
}

bool AtomTable::usesDSA() const
{
    return useDSA;
}


HandleEntry* AtomTable::getHandleSet(Type type, bool subclass,
                                     VersionHandle vh) const
{
    //printf("AtomTable::getHandleSet(Type =%d, bool=%d, AtomTableList=%d)\n", type, subclass, tableId);
    //printf("About to call AtomTable::getHandleSet()\n");
    HandleEntry* result = this->getHandleSet(type, subclass);
    //printf("Got handles from AtomTable\n");
    result = HandleEntry::filterSet(result, vh);
    //printf("Returning %p\n", result);
    return result;
}

HandleEntry* AtomTable::getHandleSet(Type type, Type targetType, bool subclass, bool targetSubclass, VersionHandle vh, VersionHandle targetVh) const
{
    //printf("AtomTable::getHandleSet(Type type, Type targetType, bool subclass, bool targetSubclass, VersionHandle vh, VersionHandle targetVh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(type, targetType, subclass, targetSubclass);
    result = HandleEntry::filterSet(result, vh);
    result = HandleEntry::filterSet(result, targetType, targetSubclass, targetVh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(Handle handle, Type type, bool subclass, VersionHandle vh) const
{
    //printf("AtomTable::getHandleSet(Handle handle, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(handle, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const std::vector<Handle>& handles, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const
{
    //printf("AtomTable::getHandleSet(const std::vector<Handle>& handles, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(handles, types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char* name, Type type, bool subclass, VersionHandle vh) const
{
    //printf("AtomTable::getHandleSet(const char* name, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = NULL;
    if (name == NULL) {
        result = getHandleSet(type, subclass, vh);
    } else {
        result = this->getHandleSet(name, type, subclass);
        result = HandleEntry::filterSet(result, vh);
    }
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass, VersionHandle vh, VersionHandle targetVh) const
{
    //printf("AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass, VersionHandle vh, VersionHandle targetVh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(targetName, targetType, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    result = HandleEntry::filterSet(result, targetName, targetType, targetVh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const
{
    //printf("AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(names, types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const
{
    //printf("AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId\n");
    HandleEntry* result = this->getHandleSet(types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

/*
 * If this method is needed it needs to be refactored to use AttentionValue instead of floats
HandleEntry* AtomTable::getHandleSet(float lowerBound, float upperBound, VersionHandle vh) const{
    //printf("AtomTable::getHandleSet(float lowerBound, float upperBound, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(lowerBound, upperBound);
    result = HandleEntry::filterSet(result, vh);
    return result;
}
*/

void AtomTable::scrubIncoming(void)
{
#ifdef USE_ATOM_HASH_SET
    for (AtomHashSet::const_iterator it = atomSet->begin(); it != atomSet->end(); it++) {
        const Atom* atom = *it;
        Handle handle = TLB::getHandle(atom);

        // Updates incoming set of all targets.
        const Link * link = dynamic_cast<const Link *>(atom);
        if (link) {
            for (int i = 0; i < link->getArity(); i++) {
                Atom *oa = link->getOutgoingAtom(i);
                HandleEntry *he = oa->getIncomingSet();
                if (false == he->contains(handle)) {
                    oa->addIncomingHandle(handle);
                }
            }
        }
    }
#endif
}

std::size_t opencog::atom_ptr_hash::operator()(const Atom* const& x) const
{
    return x->hashCode();
}

bool opencog::atom_ptr_equal_to::operator()(const Atom* const& x, 
                                            const Atom* const& y) const
{
    return *x == *y;
}

boost::signal<void (Handle)>& AtomTable::addAtomSignal()
{
    return _addAtomSignal;
}

boost::signal<void (Handle)>& AtomTable::removeAtomSignal()
{
    return _removeAtomSignal;
}
