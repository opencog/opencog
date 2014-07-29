/*
 * opencog/atomspace/Atom.cc
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

#include <set>

#ifndef WIN32
#include <unistd.h>
#endif

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/foreach.h>
#include <opencog/util/misc.h>
#include <opencog/util/platform.h>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>

//#define DPRINTF printf
#define DPRINTF(...)

#undef Type

using namespace opencog;

#define _avmtx _mtx

Atom::Atom(Type t, TruthValuePtr tv, AttentionValuePtr av)
  : _uuid(Handle::UNDEFINED.value()),
    _atomTable(NULL),
    _type(t),
    _flags(0),
    _truthValue(tv),
    _attentionValue(av)
{}

Atom::~Atom()
{
    _atomTable = NULL;
    if (0 < getIncomingSetSize()) {
        // This can't ever possibly happen. If it does, then there is
        // some very sick bug with the reference counting that the
        // shared pointers are doing. (Or someone explcitly called the
        // destructor! Which they shouldn't do.)
        OC_ASSERT(0 == getIncomingSet().size(),
             "Atom deletion failure; incoming set not empty for %s h=%d",
             classserver().getTypeName(_type).c_str(), _uuid);
    }
    drop_incoming_set();
}

// ==============================================================
// Whole lotta truthiness going on here.  Does it really need to be
// this complicated!?

void Atom::setTruthValue(TruthValuePtr newTV)
{
    if (newTV->isNullTv()) return;

    // We need to guarantee that the signal goes out with the
    // correct truth value.  That is, another setter could be changing
    // this, even as we are.  So make a copy, first. 
    TruthValuePtr oldTV(getTruthValue());

    // ... and we still need to make sure that only one thread is
    // writing this at a time. std:shared_ptr is NOT thread-safe against
    // multiple writers: see "Example 5" in
    // http://www.boost.org/doc/libs/1_53_0/libs/smart_ptr/shared_ptr.htm#ThreadSafety
    std::unique_lock<std::mutex> lck (_mtx);
    _truthValue = newTV;
    lck.unlock();

    if (_atomTable != NULL) {
        TVCHSigl& tvch = _atomTable->TVChangedSignal();
        tvch(getHandle(), oldTV, newTV);
    }
}

TruthValuePtr Atom::getTruthValue()
{
    // OK. The atomic thread-safety of shared-pointers is subtle. See
    // http://www.boost.org/doc/libs/1_53_0/libs/smart_ptr/shared_ptr.htm#ThreadSafety
    // and http://cppwisdom.quora.com/shared_ptr-is-almost-thread-safe
    // What it boils down to here is that we must *always* make a copy
    // of _truthValue before we use it, since it can go out of scope
    // because it can get set in another thread.  Viz, using it to
    // dereference can return a raw pointer to an object that has been
    // deconstructed.  The AtomSpaceAsyncUTest will hit this, as will
    // the multi-threaded async atom store in the SQL peristance backend.
    // Furthermore, we must make a copy while holding the lock! Got that?

    std::lock_guard<std::mutex> lck(_mtx);
    TruthValuePtr local(_truthValue);
    return local;
}

void Atom::merge(TruthValuePtr tvn)
{
    if (NULL == tvn or tvn->isDefaultTV() or tvn->isNullTv()) return;

    // No locking to be done here. It is possible that between the time
    // that we read the TV here (i.e. set currentTV) and the time that
    // we look to see if currentTV is default, that some other thread
    // will have changed _truthValue. This is a race, but we don't care,
    // because if two threads are trying to simultaneously set the TV on
    // one atom, without co-operating with one-another, they get what
    // they deserve -- a race. (We still use getTruthValue() to avoid a
    // read-write race on the shared_pointer itself!)
    TruthValuePtr currentTV(getTruthValue());
    if (currentTV->isDefaultTV() or currentTV->isNullTv()) {
        setTruthValue(tvn);
        return;
    }

    TruthValuePtr mergedTV(currentTV->merge(tvn));
    setTruthValue(mergedTV);
}

// ==============================================================

AttentionValuePtr Atom::getAttentionValue()
{
    // OK. The atomic thread-safety of shared-pointers is subtle. See
    // http://www.boost.org/doc/libs/1_53_0/libs/smart_ptr/shared_ptr.htm#ThreadSafety
    // and http://cppwisdom.quora.com/shared_ptr-is-almost-thread-safe
    // What it boils down to here is that we must *always* make a copy
    // of _attentionValue before we use it, since it can go out of scope
    // because it can get set in another thread.  Viz, using it to
    // dereference can return a raw pointer to an object that has been
    // deconstructed. Furthermore, we must make a copy while holding
    // the lock! Got that?

    std::lock_guard<std::mutex> lck(_mtx);
    AttentionValuePtr local(_attentionValue);
    return local;
}

void Atom::setAttentionValue(AttentionValuePtr av) throw (RuntimeException)
{
    // Must obtain a local copy of the AV, since there may be
    // parallel writers in other threads.
    AttentionValuePtr local(getAttentionValue());
    if (av == local) return;
    if (*av == *local) return;

    // Need to lock, shared_ptr is NOT atomic!
    std::unique_lock<std::mutex> lck (_avmtx);
    local = _attentionValue; // Get it again, to avoid races.
    _attentionValue = av;
    lck.unlock();

    // If the atom free-floating, we are done.
    if (NULL == _atomTable) return;

    // Get old and new bins.
    int oldBin = ImportanceIndex::importanceBin(local->getSTI());
    int newBin = ImportanceIndex::importanceBin(av->getSTI());

    // If the atom importance has changed its bin,
    // update the importance index.
    if (oldBin != newBin) {
        AtomPtr a(shared_from_this());
        _atomTable->updateImportanceIndex(a, oldBin);
    }

    // Notify any interested parties that the AV changed.
    AVCHSigl& avch = _atomTable->AVChangedSignal();
    avch(getHandle(), local, av);
}

void Atom::chgVLTI(int unit)
{
    AttentionValuePtr old_av = getAttentionValue();
    AttentionValuePtr new_av = createAV(
        old_av->getSTI(),
        old_av->getLTI(),
        old_av->getVLTI() + unit);
    setAttentionValue(new_av);
}

// ==============================================================
// Flag stuff
bool Atom::isMarkedForRemoval() const
{
    return (_flags & MARKED_FOR_REMOVAL) != 0;
}

bool Atom::getFlag(int flag) const
{
    return (_flags & flag) != 0;
}

void Atom::setFlag(int flag, bool value)
{
    if (value) {
        _flags |= flag;
    } else {
        _flags &= ~(flag);
    }
}

void Atom::unsetRemovalFlag(void)
{
    _flags &= ~MARKED_FOR_REMOVAL;
}

void Atom::markForRemoval(void)
{
    _flags |= MARKED_FOR_REMOVAL;
}

// ==============================================================

void Atom::setAtomTable(AtomTable *tb)
{
    if (tb == _atomTable) return;

    if (NULL != _atomTable) {
        // UUID's belong to the atom table, not the atom. Reclaim it.
        _uuid = Handle::UNDEFINED.value();
    }
    _atomTable = tb;
}

// ==============================================================
// Incoming set stuff

/// Start tracking the incoming set for this atom.
/// An atom can't know what it's incoming set is, until this method
/// is called.  If this atom is added to any links before this call
/// is made, those links won't show up in the incoming set.
///
/// We don't automatically track incoming sets for two reasons:
/// 1) std::set takes up 48 bytes
/// 2) adding and remoiving uses up cpu cycles.
/// Thus, if the incoming set isn't needed, then don't bother
/// tracking it.
void Atom::keep_incoming_set()
{
    if (_incoming_set) return;
    _incoming_set = std::make_shared<InSet>();
}

/// Stop tracking the incoming set for this atom.
/// After this call, the incoming set for this atom can no longer
/// be queried; it is erased.
void Atom::drop_incoming_set()
{
    if (NULL == _incoming_set) return;
    std::lock_guard<std::mutex> lck (_mtx);
    _incoming_set->_iset.clear();
    // delete _incoming_set;
    _incoming_set = NULL;
}

/// Add an atom to the incoming set.
void Atom::insert_atom(LinkPtr a)
{
    if (NULL == _incoming_set) return;
    std::lock_guard<std::mutex> lck (_mtx);
    _incoming_set->_iset.insert(a);
#ifdef INCOMING_SET_SIGNALS
    _incoming_set->_addAtomSignal(shared_from_this(), a);
#endif /* INCOMING_SET_SIGNALS */
}

/// Remove an atom from the incoming set.
void Atom::remove_atom(LinkPtr a)
{
    if (NULL == _incoming_set) return;
    std::lock_guard<std::mutex> lck (_mtx);
#ifdef INCOMING_SET_SIGNALS
    _incoming_set->_removeAtomSignal(shared_from_this(), a);
#endif /* INCOMING_SET_SIGNALS */
    _incoming_set->_iset.erase(a);
}

size_t Atom::getIncomingSetSize()
{
    if (NULL == _incoming_set) return 0;
    std::lock_guard<std::mutex> lck (_mtx);
    return _incoming_set->_iset.size();
}

// We return a copy here, and not a reference, because the set itself
// is not thread-safe during reading while simultaneous insertion and
// deletion.  Besides, the incoming set is weak; we have to make it
// strong in order to hand it out.
IncomingSet Atom::getIncomingSet()
{
    static IncomingSet empty_set;
    if (NULL == _incoming_set) return empty_set;

    // Prevent update of set while a copy is being made.
    std::lock_guard<std::mutex> lck (_mtx);
    IncomingSet iset;
    foreach (WinkPtr w, _incoming_set->_iset)
    {
        LinkPtr l(w.lock());
        if (l) iset.push_back(l);
    }
    return iset;
}
