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
#include <boost/bind.hpp>

#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/HandleMap.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/StatisticsMonitor.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/util/Config.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

//#define DPRINTF printf
//#define tableId (0) // Hack around some DPRINTF statements that want an old tableID member variable
#define DPRINTF(...)

using namespace opencog;

AtomTable::AtomTable(bool dsa)
{
    useDSA = dsa;
    size = 0;
    // This allows one to tune how often the unordered map resizes itself.
    //atomSet.max_load_factor(100.0f);

    //logger().fine("Max load factor for TLB handle map is: %f", TLB::handle_map.max_load_factor());

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_init(&iteratorsLock, NULL);
#endif

    //connect signals
    addedTypeConnection =
        classserver().addTypeSignal().connect(boost::bind(&AtomTable::typeAdded,
                    this, _1));

}

AtomTable::~AtomTable()
{
    //disconnect signals
    addedTypeConnection.disconnect();

    // remove all atoms from AtomTable
    AtomHashSet::iterator it = atomSet.begin();

    while (it != atomSet.end()) {
        DPRINTF("Removing atom %s (atomSet size = %zu)\n",
                (*it)->toString().c_str(), atomSet.size());
        remove((*it)->getHandle(), true);
        it = atomSet.begin();
    }
    atomSet.clear();
}

bool AtomTable::isCleared(void) const
{
    if (size != 0) {
        DPRINTF("AtomTable::size is not 0\n");
        return false;
    }

    if (atomSet.size() != 0) {
        DPRINTF("AtomTable[%d]::atomSet is not empty. size =%zu\n", tableId, atomSet.size());
        return false;
    }

    // if (nameIndex.size() != 0) return false;
    if (typeIndex.size() != 0) return false;
    if (importanceIndex.size() != 0) return false;
    if (targetTypeIndex.size() != 0) return false;
    if (predicateIndex.size() != 0) return false;
    return true;
}

AtomTable& AtomTable::operator=(const AtomTable& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "AtomTable - Cannot copy an object of this class");
}

AtomTable::AtomTable(const AtomTable& other) 
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "AtomTable - Cannot copy an object of this class");
}

Handle AtomTable::getHandle(const char* name, Type t) const {
    return nodeIndex.getHandle(t, name);
}
Handle AtomTable::getHandle(const Node* n) const {
    return getHandle(n->getName().c_str(), n->getType());
}

Handle AtomTable::getHandle(Type t, const HandleSeq &seq) const {
    return linkIndex.getHandle(t, seq);
}
Handle AtomTable::getHandle(const Link* l) const {
    return getHandle(l->getType(), l->getOutgoingSet());
}

HandleEntry* AtomTable::findHandlesByGPN(Handle h,
                                  VersionHandle vh) const
{
   HandleEntry* result = 
       HandleEntry::fromHandleSet(predicateIndex.findHandlesByGPN(h));
   result = HandleEntry::filterSet(result, vh);
   return result;
}

HandleEntry* AtomTable::findHandlesByGPN(const char* gpnNodeName, VersionHandle vh) const
{
    DPRINTF("AtomTable::findHandlesByGPN(%s)\n", gpnNodeName);
    // Get the GroundPredicateNode with such name
    Handle gpnHandle = getHandle(gpnNodeName, GROUNDED_PREDICATE_NODE);
    HandleEntry* result = findHandlesByGPN(gpnHandle);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const std::vector<Handle>& handles,
                                     Type* types,
                                     bool* subclasses,
                                     Arity arity,
                                     Type type,
                                     bool subclass) const
{
    // Check if it is the special case of looking for an specific atom
    if (classserver().isA(type, LINK) && 
        (arity == 0 || !handles.empty()))
    {
        DPRINTF("special case arity=%d\n", arity);
        bool hasAllHandles = true;
        for (int i = 0; hasAllHandles && i < arity; i++) {
            hasAllHandles = TLB::isValidHandle(handles[i]);
        }
        DPRINTF("hasAllHandles = %d, subclass = %d\n", hasAllHandles, subclass);
        if (hasAllHandles && !subclass) {
            DPRINTF("building link for lookup: type = %d, handles.size() = %zu\n", type, handles.size());
            Link link(type, handles); // local var on stack, avoid malloc
            AtomHashSet::const_iterator it = atomSet.find(&link);
            Handle h = Handle::UNDEFINED;
            if (it != atomSet.end()) {
                h = (*it)->getHandle();
            }
            HandleEntry* result = NULL;
            if (TLB::isValidHandle(h)) {
                result = new HandleEntry(h);
            }
            DPRINTF("Returning HandleSet by using atom hash_set!\n");
            return result;
        }
    }

    if (classserver().isA(type, LINK) && (arity == 0)) {
        HandleSeq hs;
        getHandlesByType(back_inserter(hs), type, subclass);
        HandleEntry* result = HandleEntry::fromHandleVector(hs);
        result = HandleEntry::filterSet(result, arity);
        return result;
    }

    std::vector<HandleEntry*> sets(arity, NULL);

    int countdown = 0;

    // builds a set for each element in the outgoing set. Empty sets are
    // counted to be removed a posteriori
    for (int i = 0; i < arity; i++) {
        if ((!handles.empty()) && TLB::isValidHandle(handles[i])) {
            sets[i] = getIncomingSetE(handles[i]);
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
            HandleSeq hs;
            getHandlesByTargetTypeVH(back_inserter(hs), type, types[i], subclass, sub);
            sets[i] = HandleEntry::fromHandleVector(hs);
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
        DPRINTF("newset allocated size = %d\n", (arity - countdown));
        // TODO: Perhaps it's better to simply erase the NULL entries of the sets
        std::vector<HandleEntry*> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i] != NULL) {
                newset.push_back(sets[i]);
            }
        }
        sets = newset;
    }
    DPRINTF("newLength = %d\n", newLength);

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

HandleEntry* AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass) const
{

    // Gets the exact atom with the given name and type, in any AtomTable.
    Handle handle = getHandle(targetName, targetType);

    HandleEntry* result = NULL;
    // then, if the atom returend is valid, the list with the given target name
    // and types will be returned.
    if (TLB::isValidHandle(handle)) {
        HandleSeq hs;
        getIncomingSetByType(back_inserter(hs), handle, type, subclass);
        result = HandleEntry::fromHandleVector(hs);
    }
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char** names, 
                                     Type* types, 
                                     bool* subclasses, 
                                     Arity arity, 
                                     Type type, 
                                     bool subclass)
    const throw (RuntimeException)
{
    std::vector<HandleEntry*> sets(arity, NULL);

    int countdown = 0;
    // A list for each array of names is built. Then, it's filtered by
    // the name (to avoid hash conflicts) and by the correspondent type
    // in the array of types.
    for (int i = 0; i < arity; i++) {
        DPRINTF("getHandleSet: arity %d\n", i);
        bool sub = subclasses == NULL ? false : subclasses[i];
        if ((names != NULL) && (names[i] != NULL)) {
            if ((types != NULL) && (types[i] != NOTYPE)) {
                sets[i] = getHandleSet(names[i], types[i], type, subclass);
                if (sub) {
                    // If subclasses are accepted, the subclasses are
                    // returned in the array types.
                    std::vector<Type> subTypes;

                    classserver().getChildrenRecursive(types[i], 
                                           std::back_inserter(subTypes));

                    // For all subclasses found, a set is concatenated
                    // to the answer set
                    for (unsigned int j = 0; j < subTypes.size(); j++) {
                        HandleEntry *subSet = getHandleSet(names[i], 
                                          subTypes[j], type, subclass);
                        sets[i] = HandleEntry::concatenation(sets[i], subSet);
                    }
                }
                sets[i] = HandleEntry::filterSet(sets[i], names[i], 
                                           types[i], sub, i, arity);
            } else {
                for (int j = 0; j < i; j++) {
                    delete sets[j];
                }
                throw RuntimeException(TRACE_INFO, 
                    "Cannot make this search using only target name!\n");
            }
        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            HandleSeq hs;
            getHandlesByTargetTypeVH(back_inserter(hs), type, types[i], subclass, sub);
            sets[i] = HandleEntry::fromHandleVector(hs);
            sets[i] = HandleEntry::filterSet(sets[i], types[i], sub, i, arity);
        } else {
            countdown++;
        }
    }

    // If the empty set counter is not zero, removes them by shrinking
    // the list of sets
    if (countdown > 0) {
        DPRINTF("newset allocated size = %d\n", (arity - countdown));
        // TODO: Perhaps it's better to simply erase the NULL entries of the sets
        std::vector<HandleEntry*> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i] != NULL) {
                newset.push_back(sets[i]);
            }
        }
        sets = newset;
    }

#ifdef DEBUG
    for (int i = 0; i < arity; i++) {
        DPRINTF("arity %d\n:", i);
        for (HandleEntry* it = sets[i]; it != NULL; it = it->next) {
            DPRINTF("\t%ld: %s\n", it->handle, TLB::getAtom(it->handle)->toString().c_str());
        }
        DPRINTF("\n");
    }
#endif
    // The intersection is made for all non-empty sets, and then is
    // filtered by the optional specified type. Also, if subclasses are
    // not accepted, it will not pass the filter.
    DPRINTF("AtomTable::getHandleSet: about to call intersection\n");
    HandleEntry* set = HandleEntry::intersection(sets);
    return  set;
}

HandleEntry* AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass) const
{
    return getHandleSet((const char**) NULL, types, subclasses, arity, type, subclass);
}

void AtomTable::merge(Handle h, const TruthValue& tvn)
{
    if (TLB::isValidHandle(h)) {
        Atom* atom = TLB::getAtom(h);
        // Merge the TVs
        if (!tvn.isNullTv()) {
            const TruthValue& currentTV = atom->getTruthValue();
            if (currentTV.isNullTv()) {
                atom->setTruthValue(tvn);
            } else {
                TruthValue* mergedTV = currentTV.merge(tvn);
                atom->setTruthValue(*mergedTV);
                delete mergedTV;
            }
        }
        if (logger().isFineEnabled()) 
            logger().fine("Atom merged: %d => %s", h.value(), atom->toString().c_str());
    } 
}

Handle AtomTable::add(Atom *atom) throw (RuntimeException)
{
    if (atom->getAtomTable() != NULL) {
        // Atom is already inserted
        return atom->getHandle();
    }
    Handle existingHandle = Handle::UNDEFINED;
    Node * nnn = dynamic_cast<Node *>(atom);
    Link * lll = dynamic_cast<Link *>(atom);
    // Check if the node or link handle already exists in the indexers
    if (nnn) {
        existingHandle = getHandle(nnn);
    } else if (lll) {
        existingHandle = getHandle(lll);
    }

    if (TLB::isValidHandle(existingHandle)) {
        DPRINTF("Merging existing Atom with the Atom being added ...\n");
        merge(existingHandle, atom->getTruthValue());
        delete atom;
        return existingHandle;
    }

    // New atom, its Handle will be stored in the AtomTable
    // Increment the size of the table
    size++;

    // Adds to the hash_set
    DPRINTF("Inserting atom %p intoAtomTable (type=%d)\n", atom, atom->getType());
    atomSet.insert(atom);
    DPRINTF("AtomTable::add atomSet->insert(%p) => size = %zu\n", atom, atomSet.size());

    // Checks for null outgoing set members.
    if (lll) {
        const std::vector<Handle>& ogs = lll->getOutgoingSet();
        size_t arity = ogs.size();
        for (int i = arity - 1; i >= 0; i--) {
            if (TLB::isInvalidHandle(ogs[i])) {
                throw RuntimeException(TRACE_INFO,
                           "AtomTable - Attempting to insert link with "
                           "invalid outgoing members");
            }
        }
    }

    // Its possible that the atom is already in the TLB -- 
    // e.g. if it was fetched from persistent storage; this
    // was done to preserve handle consistency.
    Handle handle = atom->handle;
    if (TLB::isInvalidHandle(handle)) handle = TLB::addAtom(atom);

    nodeIndex.insertAtom(atom);
    linkIndex.insertAtom(atom);
    typeIndex.insertAtom(atom);
    incomingIndex.insertAtom(atom);
    targetTypeIndex.insertAtom(atom);
    importanceIndex.insertAtom(atom);
    predicateIndex.insertAtom(atom);

    atom->setAtomTable(this);

    if (useDSA) {
        StatisticsMonitor::getInstance()->add(atom);
    }

    DPRINTF("Atom added: %ld => %s\n", handle.value(), atom->toString().c_str());

    return handle;
}

int AtomTable::getSize() const
{
    return size;
}

void AtomTable::log(Logger& logger, Type type, bool subclass) const
{
    AtomHashSet::const_iterator it;
    for (it = atomSet.begin(); it != atomSet.end(); ++it) {
        const Atom* atom = *it;
        bool matched = (subclass && classserver().isA(atom->getType(), type)) || type == atom->getType();
        if (matched)
            logger.debug("%d: %s", atom->getHandle().value(),
                    atom->toString().c_str());
    }
}

void AtomTable::print(std::ostream& output, Type type, bool subclass) const
{
    AtomHashSet::const_iterator it;
    for (it = atomSet.begin(); it != atomSet.end(); ++it) {
        const Atom* atom = *it;
        bool matched = (subclass && classserver().isA(atom->getType(), type)) || type == atom->getType();
        if (matched) output << atom->getHandle() << ": " << atom->toString() << std::endl;
    }
}

HandleEntry* AtomTable::extract(Handle handle, bool recursive)
{
    // TODO: Check if this atom is really inserted in this AtomTable and get the
    // exact Atom object
    HandleEntry* result = NULL;

    Atom *atom = TLB::getAtom(handle);
    if (!atom || atom->isMarkedForRemoval()) return result;
    atom->markForRemoval();

    // if recursive-flag is set, also extract all the links in the atom's
    // incoming set
    if (recursive) {
        // we need to make a copy of the incoming set because the 'incoming
        // set' container is actually a list, so the same link may appear twice
        // in an incoming set. Hopefully we'll eventually use the right
        // container
        std::set<Handle> is;
        for (HandleEntry* in = getIncomingSetE(handle); in != NULL; in = in->next)
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

    HandleEntry* he = getIncomingSetE(handle);
    if (he)
    {
        Logger::Level save = logger().getBackTraceLevel();
        logger().setBackTraceLevel(Logger::NONE);
        logger().warn("AtomTable.extract(): "
           "attempting to extract atom with non-empty incoming set: %s\n",
           atom->toShortString().c_str());
        for (HandleEntry* it = he; it != NULL; it = it->next)
        {
            logger().warn("\tincoming: %s\n", 
                 TLB::getAtom(it->handle)->toShortString().c_str());
        }
        logger().setBackTraceLevel(save);
        logger().warn("AtomTable.extract(): stack trace for previous error follows");
        atom->unsetRemovalFlag();
        return result;
    }

    //decrements the size of the table
    size--;

    atomSet.erase(atom);

    // updates all global statistics regarding the removal of this atom
    if (useDSA) StatisticsMonitor::getInstance()->remove(atom);

    nodeIndex.removeAtom(atom);
    linkIndex.removeAtom(atom);
    typeIndex.removeAtom(atom);
    incomingIndex.removeAtom(atom);
    targetTypeIndex.removeAtom(atom);
    importanceIndex.removeAtom(atom);
    predicateIndex.removeAtom(atom);

    return HandleEntry::concatenation(new HandleEntry(handle), result);
}

bool AtomTable::remove(Handle handle, bool recursive)
{
    HandleEntry* extractedHandles = extract(handle, recursive);
    if (extractedHandles) {
        removeExtractedHandles(extractedHandles);
        delete extractedHandles;
        return true;
    }
    return false;
}

void AtomTable::removeExtractedHandles(HandleEntry* extractedHandles)
{
    HandleSeq hs;
    if (extractedHandles == NULL) return;

    // We must to iterate from the end to the begining of the list of atoms so that 
    // link's target atoms are not removed before the link   
    // TODO: this is inefficient. if we alter extract to build the HandleEntry
    // list in reverse, then we can avoid initialising a temporary vector.
    hs = extractedHandles->toHandleVector();

    for (HandleSeq::reverse_iterator it = hs.rbegin(); it < hs.rend(); ++it) {
        Atom* atom = TLB::removeAtom(*it);
        if (logger().isFineEnabled())
            logger().fine("Atom removed: %d => %s", it->value(), atom->toString().c_str());
        delete atom;
    }
}

HandleEntry* AtomTable::decayShortTermImportance(void)
{
    return importanceIndex.decayShortTermImportance(this);
}

bool AtomTable::decayed(Handle h)
{
    Atom *a = TLB::getAtom(h);

    // XXX This should be an assert ... I think something is seriously
    // wrong if the handle isn't being found!  XXX FIXME
    if (NULL == a) return false;
    return a->getFlag(REMOVED_BY_DECAY);
}

void AtomTable::clearIndexesAndRemoveAtoms(HandleEntry* extractedHandles)
{
    if (extractedHandles == NULL) return;

    importanceIndex.remove(decayed);
    targetTypeIndex.remove(decayed);
    predicateIndex.remove(decayed);
    incomingIndex.remove(decayed);
    nodeIndex.remove(decayed);
    linkIndex.remove(decayed);
    typeIndex.remove(decayed);

    for (HandleEntry* curr = extractedHandles; curr != NULL; curr = curr->next) {
        Handle h = curr->handle;
        Atom* atom = TLB::getAtom(h);
        //Extracts atom from hash_set
        atomSet.erase(atom);

        // update the AtomTable's size
        size--;

        if (useDSA)
            // updates all global statistics regarding the removal of this atom
            StatisticsMonitor::getInstance()->remove(atom);
    }
}

Handle AtomTable::getRandom(RandGen *rng) const
{
    size_t x = rng->randint(getSize());
    size_t b;
    for(b=0; b<atomSet.bucket_count(); b++) {
      if(x < atomSet.bucket_size(b)) {
        break;
      } else
        x -= atomSet.bucket_size(b);
    }
    std::unordered_set<const Atom*>::const_local_iterator l = atomSet.begin(b);
    while(x>0) {
      l++;
      assert(l!=atomSet.end(b));
      x--;
    }
    return (*l)->handle;
}

bool AtomTable::usesDSA() const
{
    return useDSA;
}

HandleEntry* AtomTable::getHandleSet(const std::vector<Handle>& handles, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const
{
    DPRINTF("AtomTable::getHandleSet(const std::vector<Handle>& handles, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = getHandleSet(handles, types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass, VersionHandle vh, VersionHandle targetVh) const
{
    DPRINTF("AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass, VersionHandle vh, VersionHandle targetVh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(targetName, targetType, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    result = HandleEntry::filterSet(result, targetName, targetType, targetVh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const
{
    DPRINTF("AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(names, types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const
{
    DPRINTF("AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId\n");
    HandleEntry* result = this->getHandleSet(types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
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

void AtomTable::typeAdded(Type t)
{
    //resize all Type-based indexes
    nodeIndex.resize();
    linkIndex.resize();
    typeIndex.resize();
    targetTypeIndex.resize();
}
