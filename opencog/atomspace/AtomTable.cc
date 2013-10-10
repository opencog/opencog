/*
 * opencog/atomspace/AtomTable.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2013 Linas Vepstas <linasvepstas@gmail.com>
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
#include <opencog/atomspace/Intersect.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

//#define DPRINTF printf
//#define tableId (0) // Hack around some DPRINTF statements that want an old tableID member variable
#define DPRINTF(...)

using namespace opencog;

std::mutex AtomTable::_mtx;

AtomTable::AtomTable()
{
    size = 0;

    //connect signals
    addedTypeConnection =
        classserver().addTypeSignal().connect(boost::bind(&AtomTable::typeAdded,
                    this, _1));
}

AtomTable::~AtomTable()
{
    //disconnect signals
    addedTypeConnection.disconnect();

#if DONT_BOTHER_WITH_THIS
    // WTF!? XXX TODO why are we removing these one by one? Lets
    // just blow away all the indexes. That would be more efficeient,
    // right!?
    // Make a copy.
    HandleSeq all;
    getHandlesByType(back_inserter(all), ATOM, true);

    // remove all atoms from AtomTable
    for (HandleSeq::const_iterator it = all.begin(); it != all.end(); it++)
    {
        DPRINTF("Removing atom %s\n", (*it)->toString().c_str());
        remove(*it, true);
    }
#endif
}

bool AtomTable::isCleared(void) const
{
    if (size != 0) {
        DPRINTF("AtomTable::size is not 0\n");
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

Handle AtomTable::getHandle(Type t, const std::string& name) const
{
    return nodeIndex.getHandle(t, name.c_str());
}

/// Find an equivalent atom that has exactly the same name and type.
/// That is, if there is an atom with this name and type already in
/// the table, then return that; else return undefined.
Handle AtomTable::getHandle(const NodePtr n) const
{
    return getHandle(n->getType(), n->getName());
}

Handle AtomTable::getHandle(Type t, const HandleSeq &seq) const
{
    return linkIndex.getHandle(t, seq);
}

/// Find an equivalent atom that has exactly the same type and outgoing
/// set.  That is, if there is an atom with this ype and outset already
/// in the table, then return that; else return undefined.
Handle AtomTable::getHandle(LinkPtr l) const
{
    return getHandle(l->getType(), l->getOutgoingSet());
}

/// Find an equivalent atom that is exactly the same as the arg. If
/// such an atom is in the table, it is returned, else the return
/// is the bad handle.
Handle AtomTable::getHandle(AtomPtr a) const
{
    NodePtr nnn(NodeCast(a));
    if (nnn) 
         return getHandle(nnn);
    else {
        LinkPtr lll(LinkCast(a));
        if (lll)
            return getHandle(lll);
    }
    return Handle::UNDEFINED;
}

Handle AtomTable::getHandle(Handle h) const
{
    // If we have an atom, but don't know the uuid, find uuid.
    if (Handle::UNDEFINED.value() == h.value())
        return getHandle(AtomPtr(h));

    // If we have a uuid but no atom pointer, find the atom pointer.
    auto hit = _atom_set.find(h);
    if (hit != _atom_set.end())
        return *hit;
    return Handle::UNDEFINED;
}

UnorderedHandleSet AtomTable::getHandlesByOutgoing(const HandleSeq& handles,
                                     Type* types,
                                     bool* subclasses,
                                     Arity arity,
                                     Type type,
                                     bool subclass,
                                     VersionHandle vh) const
{
    // Check if it is the special case of looking for an specific atom
    if (classserver().isA(type, LINK) && 
        (arity == 0 || !handles.empty()))
    {
        DPRINTF("special case arity=%d\n", arity);
        bool hasAllHandles = true;
        for (Arity i = 0; hasAllHandles && i < arity; i++) {
            hasAllHandles = TLB::isValidHandle(handles[i]);
        }
        DPRINTF("hasAllHandles = %d, subclass = %d\n", hasAllHandles, subclass);
        if (hasAllHandles && !subclass) {
            DPRINTF("building link for lookup: type = %d, handles.size() = %zu\n", type, handles.size());
            Handle h = getHandle(type, handles);

            UnorderedHandleSet result;
            if (TLB::isValidHandle(h) and containsVersionedTV(h, vh)) {
                result.insert(h);
            }
            DPRINTF("Returning HandleSet by using atom hash_set!\n");
            return result;
        }
    }

    if (classserver().isA(type, LINK) && (arity == 0)) {
        UnorderedHandleSet uhs;
        getHandlesByType(inserter(uhs), type, subclass);

        UnorderedHandleSet result;
        std::copy_if(uhs.begin(), uhs.end(), inserter(result), 
            // result = HandleEntry::filterSet(result, arity);
            [&](Handle h)->bool { 
                LinkPtr l(LinkCast(h));
                // If a Node, then accept it.
                if (NULL == l) return containsVersionedTV(h, vh);
                return (0 == l->getArity()) and containsVersionedTV(h, vh);
        });
        return result;
    }

    std::vector<UnorderedHandleSet> sets(arity);

    int countdown = 0;

    // builds a set for each element in the outgoing set. Empty sets are
    // counted to be removed a posteriori
    for (Arity i = 0; i < arity; i++) {
        if ((!handles.empty()) && TLB::isValidHandle(handles[i])) {
            UnorderedHandleSet hs = getIncomingSet(handles[i]);

            std::copy_if(hs.begin(), hs.end(), inserter(sets[i]),
                // sets[i] = HandleEntry::filterSet(sets[i], handles[i], i, arity);
                [&](Handle h)->bool {
                    LinkPtr l(LinkCast(h));
                    // If a Node, then accept it.
                    if (NULL == l) return true;
                    return (l->getArity() == arity) and
                           (handles[i] == l->getOutgoingSet()[i]);
                });

            if (sets[i].size() == 0)
                return UnorderedHandleSet();

        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            bool sub = subclasses == NULL ? false : subclasses[i];
            getHandlesByTargetTypeVH(inserter(sets[i]), type, types[i], subclass, sub);
            if (sets[i].size() == 0)
                return UnorderedHandleSet();
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
        std::vector<UnorderedHandleSet> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i].size() != 0)
                newset.push_back(sets[i]);
        }
        sets = newset;
    }
    DPRINTF("newLength = %d\n", newLength);

    if ((type != ATOM) || (!subclass)) {
        for (int i = 0; i < newLength; i++) {
            // filters by type and subclass in order to remove unwanted elements.
            // This is done before the intersection method to reduce the number of
            // elements being passed (intersection uses qsort, which is n log n)
            // sets[i] = HandleEntry::filterSet(sets[i], type, subclass);
            UnorderedHandleSet hs;
            std::copy_if(sets[i].begin(), sets[i].end(), inserter(hs),
                [&](Handle h)->bool { return isType(h, type, subclass); });
        }
    }

    // computes the intersection of all non-empty sets
    UnorderedHandleSet set = intersection(sets);
    // TODO: Why not move this filtering to the begining...
    // Pehaps it will filter more before the intersection
    // (which seems to be the most expensive operation)
    // filters the answer set for every type in the array of target types
    if (types != NULL) {
        for (int i = 0; i < arity; i++) {
            if (types[i] != NOTYPE) {
                bool sub = subclasses == NULL ? false : subclasses[i];
                // set = HandleEntry::filterSet(set, types[i], sub, i, arity);
                UnorderedHandleSet filt;
                std::copy_if(set.begin(), set.end(), inserter(filt),
                    [&](Handle h)->bool {
                        LinkPtr l(LinkCast(h));
                        // If a Node, then accept it.
                        if (NULL == l) return containsVersionedTV(h, vh);
                        if (l->getArity() != arity) return false;
                        return isType(l->getOutgoingSet()[i], types[i], sub)
                               and containsVersionedTV(h, vh);
                    });
                set = filt;
            }
        }
    }

    return set;
}

UnorderedHandleSet AtomTable::getHandlesByNames(const char** names, 
                                     Type* types, 
                                     bool* subclasses, 
                                     Arity arity, 
                                     Type type, 
                                     bool subclass,
                                     VersionHandle vh)
    const throw (RuntimeException)
{
    std::vector<UnorderedHandleSet> sets(arity);

    int countdown = 0;
    // A list for each array of names is built. Then, it's filtered by
    // the name (to avoid hash conflicts) and by the correspondent type
    // in the array of types.
    for (int i = 0; i < arity; i++) {
        DPRINTF("getHandleSet: arity %d\n", i);
        bool sub = subclasses == NULL ? false : subclasses[i];
        if ((names != NULL) && (names[i] != NULL)) {
            if ((types != NULL) && (types[i] != NOTYPE)) {
                getIncomingSetByName(inserter(sets[i]), names[i], types[i], type, subclass);
                if (sub) {
                    // If subclasses are accepted, the subclasses are
                    // returned in the array types.
                    std::vector<Type> subTypes;

                    classserver().getChildrenRecursive(types[i], 
                                           std::back_inserter(subTypes));

                    // For all subclasses found, a set is concatenated
                    // to the answer set
                    for (unsigned int j = 0; j < subTypes.size(); j++) {
                        UnorderedHandleSet subSet;
                        getIncomingSetByName(inserter(subSet), names[i], 
                                          subTypes[j], type, subclass);
                        sets[i].insert(subSet.begin(), subSet.end());
                    }
                }
                // sets[i] = HandleEntry::filterSet(sets[i], names[i], types[i], sub, i, arity);
                UnorderedHandleSet filt;
                std::copy_if(sets[i].begin(), sets[i].end(), inserter(filt),
                    [&](Handle h)->bool {
                        LinkPtr l(LinkCast(h));
                        if (l->getArity() != arity) return false;
                        Handle oh = l->getOutgoingSet()[i];
                        if (not isType(oh, types[i], sub)) return false;
                        AtomPtr oa = l->getOutgoingAtom(i);
                        if (LinkCast(oa))
                            return (NULL == names[i]) or (0 == names[i][0]);
                        NodePtr on(NodeCast(oa));
                        return on->getName() == names[i];
                    });
                sets[i] = filt;

            } else {
                throw RuntimeException(TRACE_INFO, 
                    "Cannot make this search using only target name!\n");
            }
        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            UnorderedHandleSet hs;
            getHandlesByTargetTypeVH(inserter(hs), type, types[i], subclass, sub);
            // sets[i] = HandleEntry::filterSet(sets[i], types[i], sub, i, arity);
            std::copy_if(hs.begin(), hs.end(), inserter(sets[i]),
                [&](Handle h)->bool {
                    LinkPtr l(LinkCast(h));
                    if (l->getArity() != arity) return false;
                    Handle oh = l->getOutgoingSet()[i];
                    return isType(oh, types[i], sub);
                });
        } else {
            countdown++;
        }
    }

    // If the empty set counter is not zero, removes them by shrinking
    // the list of sets
    if (countdown > 0) {
        DPRINTF("newset allocated size = %d\n", (arity - countdown));
        // TODO: Perhaps it's better to simply erase the NULL entries of the sets
        std::vector<UnorderedHandleSet> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i].size() != 0) {
                newset.push_back(sets[i]);
            }
        }
        sets = newset;
    }

#ifdef DEBUG
    for (int i = 0; i < arity; i++) {
        DPRINTF("arity %d\n:", i);
        UnorderedHandleSet::const_iterator it;
        for (it = sets[i].begin(); it != sets[i].end(); it++) {
            DPRINTF("\t%ld: %s\n", it->value(), (*it)->toString().c_str());
        }
        DPRINTF("\n");
    }
#endif
    // The intersection is made for all non-empty sets, and then is
    // filtered by the optional specified type. Also, if subclasses are
    // not accepted, it will not pass the filter.
    DPRINTF("AtomTable::getHandleSet: about to call intersection\n");
    return intersection(sets);
}

void AtomTable::merge(Handle h, const TruthValue& tvn)
{
    if (NULL == h) return;

    // Merge the TVs
    if (!tvn.isNullTv()) {
        const TruthValue& currentTV = h->getTruthValue();
        if (currentTV.isNullTv()) {
            h->setTruthValue(tvn);
        } else {
            TruthValue* mergedTV = currentTV.merge(tvn);
            h->setTruthValue(*mergedTV);
            delete mergedTV;
        }
    }
    if (logger().isFineEnabled()) 
        logger().fine("Atom merged: %d => %s", h.value(), h->toString().c_str());
}

Handle AtomTable::add(AtomPtr atom) throw (RuntimeException)
{
    if (atom->getAtomTable() != NULL) {
        // Atom is already inserted
        return atom->getHandle();
    }

    std::lock_guard<std::mutex> lck(_mtx);

#if LATER
    // XXX FIXME -- technically, this throw is correct, except
    // thatSavingLoading gives us atoms with handles preset.
    // So we have to accept that, and hope its correct and consistent.
    if (atom->_uuid != Handle::UNDEFINED.value())
        throw RuntimeException(TRACE_INFO,
          "AtomTable - Attempting to insert atom with handle already set!");
#endif

    // Is the equivalent of this atom already in the table?
    // If so, then we merge the truth values.
    Handle hexist = getHandle(atom);
    if (hexist) {
        DPRINTF("Merging existing Atom with the Atom being added ...\n");
        merge(hexist, atom->getTruthValue());
        // XXX TODO -- should merege attention value too, right ???
        return hexist;
    }

    // Check for bad outgoing set members; fix them up if needed.
    LinkPtr lll(LinkCast(atom));
    if (lll) {
        const HandleSeq ogs = lll->getOutgoingSet();
        size_t arity = ogs.size();
        for (size_t i = 0; i < arity; i++) {
            Handle h = ogs[i];
            // It can happen that the uuid is assigned, but the pointer
            // is NULL. In that case, we should at least know about this
            // uuid.
            if (NULL == h and Handle::UNDEFINED != h) {
                auto it = _atom_set.find(h);
                if (it != _atom_set.end()) {
                    h = *it;

                    // OK, here's the deal. We really need to fixup
                    // link so that it holds a valid atom pointer. We
                    // do that here. Unfortunately, this is not really
                    // thread-safe, and there is no particularly elegant
                    // way to lock. So we punt.  This makes sense,
                    // because it is unlikely that one threa is going to
                    // be winging on the outgoing set, whille another
                    // thread is performing an atom-table add.  I'm pretty
                    // sure its a user error if the user fails to serialize
                    // atom table adds appropriately for their app.
                    lll->_outgoing[i] = h;
                    continue;
                }
            }
            if (Handle::UNDEFINED == h) {
                throw RuntimeException(TRACE_INFO,
                           "AtomTable - Attempting to insert link with "
                           "invalid outgoing members");
            }
        }
    }

    // Its possible that the atom already has a UUID assigned,
    // e.g. if it was fetched from persistent storage; this
    // was done to preserve handle consistency. SavingLoading does
    // this too.  XXX Review for corrrectness...
    if (atom->_uuid == Handle::UNDEFINED.value()) {
       // Atom doesn't yet have a valid uuid assigned to it. Ask the TLB
       // to issue a valid uuid.  And then memorize it.
       TLB::addAtom(atom);
    } else {
       TLB::reserve_range(0, atom->_uuid);
    }
    Handle h = atom->getHandle();
    size++;
    _atom_set.insert(h);

    nodeIndex.insertAtom(atom);
    linkIndex.insertAtom(atom);
    typeIndex.insertAtom(atom);
    incomingIndex.insertAtom(atom);
    targetTypeIndex.insertAtom(atom);
    importanceIndex.insertAtom(atom);
    predicateIndex.insertAtom(atom);

    atom->setAtomTable(this);

    DPRINTF("Atom added: %ld => %s\n", atom->_uuid, atom->toString().c_str());

    return atom->getHandle();
}

int AtomTable::getSize() const
{
    return size;
}

void AtomTable::log(Logger& logger, Type type, bool subclass) const
{
    foreachHandleByType( 
        [&](Handle h)->void {
            logger.debug("%d: %s", h.value(), h->toString().c_str());
        },
        type, subclass);
}

void AtomTable::print(std::ostream& output, Type type, bool subclass) const
{
    foreachHandleByType( 
        [&](Handle h)->void {
            output << h << ": " << h->toString() << std::endl;
        },
        type, subclass);
}

Handle AtomTable::getRandom(RandGen *rng) const
{
    size_t x = rng->randint(getSize());

    Handle randy = Handle::UNDEFINED;

    foreachHandleByType( 
        [&](Handle h)->void {
            if (0 == x) randy = h;
            x--;
        },
        ATOM, true);
    return randy;
}

AtomPtrSet AtomTable::extract(Handle handle, bool recursive)
{
    AtomPtrSet result;

    handle = getHandle(handle);
    AtomPtr atom(handle);
    if (!atom || atom->isMarkedForRemoval()) return result;
    atom->markForRemoval();

    // If recursive-flag is set, also extract all the links in the atom's
    // incoming set
    if (recursive) {
        // We need to make a copy of the incoming set because the
        // recursive call will trash the incoming set when the atom
        // is removed.  
        UnorderedHandleSet is = getIncomingSet(handle);

        UnorderedHandleSet::const_iterator is_it;
        for (is_it = is.begin(); is_it != is.end(); ++is_it)
        {
            Handle his = *is_it;
            DPRINTF("[AtomTable::extract] incoming set: %s",
                 (his) ? his->toString().c_str() : "INVALID HANDLE");

            if (not his->isMarkedForRemoval()) {
                DPRINTF("[AtomTable::extract] marked for removal is false");

                AtomPtrSet ex = extract(his, true);
                result.insert(ex.begin(), ex.end());
            }
        }
    }

    const UnorderedHandleSet& is = getIncomingSet(handle);
    if (0 < is.size())
    {
        // It is very tempting to just throw, here, but apparently,
        // someone somewhere thinks that it is more appropriate to
        // log a warning, instead.  Not clear to my why this is a
        // wise decision .. perhaps there is some race condition
        // removal due to attention value miscalculation? ???
        // XXX TODO Review the policy here and rationalize it.
        // throw RuntimeException(TRACE_INFO,
        //   "Cannot extract an atom with a non-trivial incoming set!");

        // XXX well, I guess we could/should check to see if any atoms 
        // in the incoming set belong to this atomspace. Because if
        // none of them do, then it would be ok to extract...
        Logger::Level save = logger().getBackTraceLevel();
        logger().setBackTraceLevel(Logger::NONE);
        logger().warn("AtomTable.extract(): "
           "attempting to extract atom with non-empty incoming set: %s\n",
           atom->toShortString().c_str());
        UnorderedHandleSet::const_iterator it;
        for (it = is.begin(); it != is.end(); it++)
        {
            logger().warn("\tincoming: %s\n", 
                 (*it)->toShortString().c_str());
        }
        logger().setBackTraceLevel(save);
        logger().warn("AtomTable.extract(): stack trace for previous error follows");

        atom->unsetRemovalFlag();
        return AtomPtrSet();
    }

    // decrements the size of the table
    size--;

    nodeIndex.removeAtom(atom);
    linkIndex.removeAtom(atom);
    typeIndex.removeAtom(atom);
    incomingIndex.removeAtom(atom);
    targetTypeIndex.removeAtom(atom);
    importanceIndex.removeAtom(atom);
    predicateIndex.removeAtom(atom);

    atom->atomTable = NULL;

    result.insert(atom);
    return result;
}

bool AtomTable::decayed(Handle h)
{
    // XXX This should be an assert ... I think something is seriously
    // wrong if the handle isn't being found!  XXX FIXME
    if (NULL == h) return false;
    return h->getFlag(REMOVED_BY_DECAY);
}

AtomPtrSet AtomTable::decayShortTermImportance()
{
    UnorderedHandleSet exh = importanceIndex.decayShortTermImportance(this);

    AtomPtrSet aps;
    // update the AtomTable's size
    UnorderedHandleSet::const_iterator it;
    for (it = exh.begin(); it != exh.end(); it++) {
        AtomPtrSet exa = extract(*it);
        aps.insert(exa.begin(), exa.end());
    }
    return aps;
}

void AtomTable::typeAdded(Type t)
{
    //resize all Type-based indexes
    nodeIndex.resize();
    linkIndex.resize();
    typeIndex.resize();
    targetTypeIndex.resize();
}

