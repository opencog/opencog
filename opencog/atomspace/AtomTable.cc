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

#include <iterator>
#include <set>

#include <stdlib.h>
#include <boost/bind.hpp>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Intersect.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/functional.h>
#include <opencog/util/Logger.h>

//#define DPRINTF printf
//#define tableId (0) // Hack around some DPRINTF statements that want an old tableID member variable
#define DPRINTF(...)

using namespace opencog;

std::recursive_mutex AtomTable::_mtx;

AtomTable::AtomTable()
{
    size = 0;

    // Set resolver before doing anything else, such as getting
    // the atom-added signals.  Just in case some other thread
    // is busy adding types while we are being created.
    Handle::set_resolver(this);

    // Connect signal to find out about type additions
    addedTypeConnection =
        classserver().addTypeSignal().connect(
            boost::bind(&AtomTable::typeAdded, this, _1));
}

AtomTable::~AtomTable()
{
    // Disconnect signals. Only then clear the resolver.
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    addedTypeConnection.disconnect();
    Handle::clear_resolver(this);

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

    std::lock_guard<std::recursive_mutex> lck(_mtx);
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
    std::lock_guard<std::recursive_mutex> lck(_mtx);
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
    std::lock_guard<std::recursive_mutex> lck(_mtx);
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

Handle AtomTable::getHandle(Handle& h) const
{
    // If we have an atom, but don't know the uuid, find uuid.
    if (Handle::UNDEFINED.value() == h.value())
        return getHandle(AtomPtr(h));

    // If we have both a uuid and pointer, there's nothing to do.
    // Note: we access the naked pointer itself; that's because
    // Handle itself calls this method to resolve null pointers.
    if (h._ptr) return h;

    // Read-lock for the _atom_set.
    std::lock_guard<std::recursive_mutex> lck(_mtx);

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
                                     bool subclass) const
{
    // Check if it is the special case of looking for an specific atom
    if (classserver().isA(type, LINK) and
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
            Handle h(getHandle(type, handles));

            UnorderedHandleSet result;
            if (TLB::isValidHandle(h)) {
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
                if (NULL == l) return true;
                return (0 == l->getArity());
        });
        return result;
    }

    std::vector<UnorderedHandleSet> sets(arity);

    int countdown = 0;

    // builds a set for each element in the outgoing set. Empty sets are
    // counted to be removed a posteriori
    for (Arity i = 0; i < arity; i++) {
        if ((!handles.empty()) && TLB::isValidHandle(handles[i])) {
            Handle h(handles[i]);
            HandleSeq hs;
            h->getIncomingSet(back_inserter(hs));

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
            getHandlesByTargetType(inserter(sets[i]), type, types[i], subclass, sub);
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
                [&](Handle h)->bool { return h->isType(type, subclass); });
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
                        if (NULL == l) return true;
                        if (l->getArity() != arity) return false;
                        Handle hosi(l->getOutgoingSet()[i]);
                        return hosi->isType(types[i], sub);
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
                                     bool subclass)
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
                Handle targh(getHandle(types[i], names[i]));
                targh->getIncomingSetByType(inserter(sets[i]), type, subclass);
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
                        Handle targh(getHandle(subTypes[j], names[i]));
                        targh->getIncomingSetByType(inserter(subSet), type, subclass);
                        // XXX wait .. why are we copying, again?
                        sets[i].insert(subSet.begin(), subSet.end());
                    }
                }
                // sets[i] = HandleEntry::filterSet(sets[i], names[i], types[i], sub, i, arity);
                UnorderedHandleSet filt;
                std::copy_if(sets[i].begin(), sets[i].end(), inserter(filt),
                    [&](Handle h)->bool {
                        LinkPtr l(LinkCast(h));
                        if (l->getArity() != arity) return false;
                        Handle oh(l->getOutgoingSet()[i]);
                        if (not oh->isType(types[i], sub)) return false;
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
            getHandlesByTargetType(inserter(hs), type, types[i], subclass, sub);
            // sets[i] = HandleEntry::filterSet(sets[i], types[i], sub, i, arity);
            std::copy_if(hs.begin(), hs.end(), inserter(sets[i]),
                [&](Handle h)->bool {
                    LinkPtr l(LinkCast(h));
                    if (l->getArity() != arity) return false;
                    Handle oh(l->getOutgoingSet()[i]);
                    return oh->isType(types[i], sub);
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

Handle AtomTable::add(AtomPtr atom) throw (RuntimeException)
{
    // Sometimes one inserts an atom that was previously deleted.
    // In this case, the removal flag might still be set. Clear it.
    atom->unsetRemovalFlag();

    // Is the atom already in the table?
    if (atom->getAtomTable() != NULL) {
        return atom->getHandle();
    }

#if LATER
    // XXX FIXME -- technically, this throw is correct, except
    // thatSavingLoading gives us atoms with handles preset.
    // So we have to accept that, and hope its correct and consistent.
    if (atom->_uuid != Handle::UNDEFINED.value())
        throw RuntimeException(TRACE_INFO,
          "AtomTable - Attempting to insert atom with handle already set!");
#endif

    // Lock before checking to see if this kind of atom can already
    // be found in the atomspace.  We need to lock here, to avoid two
    // different threads from trying to add exactly the same atom.
    std::unique_lock<std::recursive_mutex> lck(_mtx);

    // Is the equivalent of this atom already in the table?
    // If so, then we merge the truth values.
    Handle hexist(getHandle(atom));
    if (hexist) {
        DPRINTF("Merging existing Atom with the Atom being added ...\n");
        hexist->merge(atom->getTruthValue());
        // XXX TODO -- should merge attention value too, right ???
        return hexist;
    }

    // Check for bad outgoing set members; fix them up if needed.
    LinkPtr lll(LinkCast(atom));
    if (lll) {
        const HandleSeq ogs = lll->getOutgoingSet();
        size_t arity = ogs.size();
        for (size_t i = 0; i < arity; i++) {
            Handle h(ogs[i]);
            // It can happen that the uuid is assigned, but the pointer
            // is NULL. In that case, we should at least know about this
            // uuid.  We explicitly test h._ptr.get() so as not to
            // accidentally resolve during the test.
            if (NULL == h._ptr.get() and Handle::UNDEFINED != h) {
                auto it = _atom_set.find(h);
                if (it != _atom_set.end()) {
                    h = *it;

                    // OK, here's the deal. We really need to fixup
                    // link so that it holds a valid atom pointer. We
                    // do that here. Unfortunately, this is not really
                    // thread-safe, and there is no particularly elegant
                    // way to lock. So we punt.  This makes sense,
                    // because it is unlikely that one thread is going to
                    // be wingeing on the outgoing set, while another
                    // thread is performing an atom-table add.  I'm pretty
                    // sure its a user error if the user fails to serialize
                    // atom table adds appropriately for their app.
                    lll->_outgoing[i] = h;
                } else {
                    throw RuntimeException(TRACE_INFO,
                        "AtomTable - Atom in outgoing set must have been "
                        "previously inserted into the atom table!");
                }
            }
            else if (Handle::UNDEFINED == h) {
                throw RuntimeException(TRACE_INFO,
                           "AtomTable - Attempting to insert link with "
                           "invalid outgoing members");
            }
            h->insert_atom(lll);
        }
    }

    // Its possible that the atom already has a UUID assigned,
    // e.g. if it was fetched from persistent storage; this
    // was done to preserve handle consistency. SavingLoading does
    // this too.  XXX Review SavingLoading for corrrectness...
    if (atom->_uuid == Handle::UNDEFINED.value()) {
       // Atom doesn't yet have a valid uuid assigned to it. Ask the TLB
       // to issue a valid uuid.  And then memorize it.
       TLB::addAtom(atom);
    } else {
       TLB::reserve_range(0, atom->_uuid);
    }
    Handle h(atom->getHandle());
    size++;
    _atom_set.insert(h);

    nodeIndex.insertAtom(atom);
    linkIndex.insertAtom(atom);
    typeIndex.insertAtom(atom);
    atom->keep_incoming_set();
    targetTypeIndex.insertAtom(atom);
    importanceIndex.insertAtom(atom);
    predicateIndex.insertAtom(atom);

    atom->setAtomTable(this);

    // We can now unlock, since we are done. In particular, the signals
    // need to run unlocked, since they may result in more atom table
    // additions.
    lck.unlock();

    // Now that we are completely done, emit the added signal.
    _addAtomSignal(h);

    DPRINTF("Atom added: %ld => %s\n", atom->_uuid, atom->toString().c_str());
    return h;
}

size_t AtomTable::getSize() const
{
    return size;
}

size_t AtomTable::getNumNodes() const
{
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    return nodeIndex.size();
}

size_t AtomTable::getNumLinks() const
{
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    return linkIndex.size();
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
            output << h->toString() << std::endl;
        },
        type, subclass);
}

Handle AtomTable::getRandom(RandGen *rng) const
{
    size_t x = rng->randint(getSize());

    Handle randy(Handle::UNDEFINED);

    // XXX TODO it would be considerably mor efficient to go into the
    // the type index, and decrement x by the size of the index for
    // each type.  This would speed up the algo by about 100 (by about
    // the number of types that are in use...).
    foreachHandleByType(
        [&](Handle h)->void {
            if (0 == x) randy = h;
            x--;
        },
        ATOM, true);
    return randy;
}

AtomPtrSet AtomTable::extract(Handle& handle, bool recursive)
{
    AtomPtrSet result;

    // Make sure the atom is fully resolved before we go about
    // deleting it.
    handle = getHandle(handle);
    AtomPtr atom(handle);
    if (!atom || atom->isMarkedForRemoval()) return result;

    // Perhaps the atom is not in the table?
    if (atom->getAtomTable() == NULL) return result;

    atom->markForRemoval();
    // lock before fetching the incoming set. Since getting the
    // incoming set also grabs a lock, we need this mutex to be
    // recursive. We need to lock here to avoid confusion if multiple
    // threads are trying to delete the same atom.
    std::unique_lock<std::recursive_mutex> lck(_mtx);

    // If recursive-flag is set, also extract all the links in the atom's
    // incoming set
    if (recursive) {
        // We need to make a copy of the incoming set because the
        // recursive call will trash the incoming set when the atom
        // is removed.
        HandleSeq is;
        handle->getIncomingSet(back_inserter(is));

        HandleSeq::iterator is_it = is.begin();
        HandleSeq::iterator is_end = is.end();
        for (; is_it != is_end; ++is_it)
        {
            Handle his(*is_it);
            DPRINTF("[AtomTable::extract] incoming set: %s",
                 (his) ? his->toString().c_str() : "INVALID HANDLE");

            if (not his->isMarkedForRemoval()) {
                DPRINTF("[AtomTable::extract] marked for removal is false");

                AtomPtrSet ex = extract(his, true);
                result.insert(ex.begin(), ex.end());
            }
        }
    }

    // Check for an invalid condition that should not occur. See:
    // https://github.com/opencog/opencog/commit/a08534afb4ef7f7e188e677cb322b72956afbd8f#commitcomment-5842682
    if (0 < handle->getIncomingSetSize())
    {
        atom->unsetRemovalFlag();

        throw RuntimeException(TRACE_INFO,
           "Cannot extract an atom with a non-empty incoming set!");
    }

    // Issue the atom removal signal *BEFORE* the atom is actually
    // removed.  This is needed so that certain subsystems, e.g. the
    // Agent system activity table, can correctly manage the atom;
    // it needs info that gets blanked out during removal.
    lck.unlock();
    _removeAtomSignal(atom);
    lck.lock();

    // Decrements the size of the table
    size--;
    _atom_set.erase(handle);

    nodeIndex.removeAtom(atom);
    linkIndex.removeAtom(atom);
    typeIndex.removeAtom(atom);
    LinkPtr lll(LinkCast(atom));
    if (lll) {
        foreach(AtomPtr a, lll->_outgoing) {
            a->remove_atom(lll);
        }
    }
    targetTypeIndex.removeAtom(atom);
    importanceIndex.removeAtom(atom);
    predicateIndex.removeAtom(atom);

    // XXX Setting the atom table causes AVChanged signals to be emitted.
    // We should really do this unlocked, but I'm tooo lazy to fix, and
    // am hoping no one will notice.
    atom->setAtomTable(NULL);

    result.insert(atom);
    return result;
}

// This is the resize callback, when a new type is dynamically added.
void AtomTable::typeAdded(Type t)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    //resize all Type-based indexes
    nodeIndex.resize();
    linkIndex.resize();
    typeIndex.resize();
    targetTypeIndex.resize();
}

