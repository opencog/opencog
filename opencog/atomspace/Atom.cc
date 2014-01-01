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
{
    _uuid = Handle::UNDEFINED.value();
    _flags = 0;
    _atomTable = NULL;
    _type = t;
    _attentionValue = av;

    if (tv and not tv->isNullTv()) _truthValue = tv;
}

Atom::~Atom()
{
    _atomTable = NULL;
    drop_incoming_set();
}

// ==============================================================
// Whole lotta truthiness going on here.  Does it really need to be
// this complicated!?

void Atom::setTruthValue(TruthValuePtr newTV)
{
    if (newTV->isNullTv()) return;

    // We need to gauranteee that the signal goes out with the
    // correct truth value.  That is, another setter could be changing
    // this, even as we are.  So make a copy, first. No, we can't quite
    // do this with swap() ... althouh swap is atomic, there's still a
    // race between here and the signal-send.
    TruthValuePtr oldTV(_truthValue);

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

void Atom::setTV(TruthValuePtr new_tv, VersionHandle vh)
{
    // The std::shared_ptr is *almost* atomic. Its subtle, but what it
    // boils down to is that we must make a local copy, and access that
    // because the global copy in _truthValue might be changed in a
    // different thread, possibly causing us to refrence a freed pointer.
    TruthValuePtr local(_truthValue);

    if (!isNullVersionHandle(vh))
    {
        CompositeTruthValuePtr ctv((local->getType() == COMPOSITE_TRUTH_VALUE) ?
                            CompositeTruthValue::createCTV(local) :
                            CompositeTruthValue::createCTV(local, NULL_VERSION_HANDLE));
        ctv->setVersionedTV(new_tv, vh);
        new_tv = std::static_pointer_cast<TruthValue>(ctv);
    }
    else if (local->getType() == COMPOSITE_TRUTH_VALUE and
             new_tv->getType() != COMPOSITE_TRUTH_VALUE)
    {
        CompositeTruthValuePtr ctv(CompositeTruthValue::createCTV(local));
        ctv->setVersionedTV(new_tv, NULL_VERSION_HANDLE);
        new_tv = std::static_pointer_cast<TruthValue>(ctv);
    }
    setTruthValue(new_tv); // always call setTruthValue to update indices
}

TruthValuePtr Atom::getTV(VersionHandle vh) const
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
    //
    // So, make a copy:
    TruthValuePtr local(_truthValue);

    if (isNullVersionHandle(vh)) {
        return local;
    }

    if (local->getType() == COMPOSITE_TRUTH_VALUE)
    {
        return std::dynamic_pointer_cast<CompositeTruthValue>(local)->getVersionedTV(vh);
    }
    return TruthValue::NULL_TV();
}


void Atom::setMean(float mean) throw (InvalidParamException)
{
    TruthValuePtr newTv(_truthValue);
    if (newTv->getType() == COMPOSITE_TRUTH_VALUE) {
        // Since CompositeTV has no setMean() method, we must handle it differently
        CompositeTruthValuePtr ctv = std::static_pointer_cast<CompositeTruthValue>(newTv);

        TruthValuePtr primaryTv = ctv->getPrimaryTV();
        if (primaryTv->getType() == SIMPLE_TRUTH_VALUE) {
            primaryTv = SimpleTruthValue::createTV(mean, primaryTv->getCount());
        } else if (primaryTv->getType() == INDEFINITE_TRUTH_VALUE) {
            IndefiniteTruthValuePtr itv = IndefiniteTruthValue::createITV(primaryTv);
            itv->setMean(mean);
            primaryTv = itv;
        } else {
            throw InvalidParamException(TRACE_INFO,
               "Atom::setMean(): Got a primaryTV with an invalid or unknown type");
        }
        ctv->setVersionedTV(primaryTv, NULL_VERSION_HANDLE);
    } else {
        if (newTv->getType() == SIMPLE_TRUTH_VALUE) {
            newTv = SimpleTruthValue::createTV(mean, newTv->getCount());
        } else if (newTv->getType() == INDEFINITE_TRUTH_VALUE) {
            IndefiniteTruthValuePtr itv = IndefiniteTruthValue::createITV(newTv);
            itv->setMean(mean);
            newTv = itv;
        } else {
            throw InvalidParamException(TRACE_INFO,
               "Atom::setMean(): - Got a TV with an invalid or unknown type");
        }
    }
    setTV(newTv);
}

void Atom::merge(TruthValuePtr tvn)
{
    if (NULL == tvn or tvn->isNullTv()) return;

    // As far as I can tell, there is no need to lock this
    // section of the code; all changes to the TV are essentially
    // atomic, from what I can tell (right?)
    TruthValuePtr currentTV = getTruthValue();
    TruthValuePtr mergedTV;
    if (currentTV->isNullTv()) {
        mergedTV = tvn;
    } else {
        mergedTV = currentTV->merge(tvn);
    }
    setTruthValue(mergedTV);
}

// ==============================================================

void Atom::setAttentionValue(AttentionValuePtr av) throw (RuntimeException)
{
    if (av == _attentionValue) return;
    if (*av == *_attentionValue) return;

    // I don't think we need to lock here, since I believe that swap
    // is atomic (right??)
    _attentionValue.swap(av);

    // If the atom free-floating, we are done.
    if (NULL == _atomTable) return;

    std::unique_lock<std::mutex> lck (_avmtx);

    // gets old and new bins
    int oldBin = ImportanceIndex::importanceBin(av->getSTI());
    int newBin = ImportanceIndex::importanceBin(_attentionValue->getSTI());

    // if the atom importance has changed its bin,
    // updates the importance index
    if (oldBin != newBin) {
        AtomPtr a(shared_from_this());
        _atomTable->updateImportanceIndex(a, oldBin);
    }
    // Must unlock before sending signals, to avoid future deadlocks.
    lck.unlock();

    // Notify any interested parties that the AV changed.
    AVCHSigl& avch = _atomTable->AVChangedSignal();
    avch(getHandle(), av, _attentionValue);  // av is old, after swap.
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

    // Notify any interested parties that the AV changed.
    if (NULL == tb and NULL != _atomTable) {
        // remove, as far as the old table is concerned
        AVCHSigl& avch = _atomTable->AVChangedSignal();
        avch(getHandle(), _attentionValue, AttentionValue::DEFAULT_AV());

        // UUID's belong to the atom table, not the atom. Reclaim it.
        _uuid = Handle::UNDEFINED.value();
    }
    _atomTable = tb;
    if (NULL != tb) {
        // add, as far as the old table is concerned
        AVCHSigl& avch = tb->AVChangedSignal();
        avch(getHandle(), AttentionValue::DEFAULT_AV(), _attentionValue);
    }
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
/// be queried; it si erased.
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
    foreach(WinkPtr w, _incoming_set->_iset)
    {
        LinkPtr l(w.lock());
        if (l) iset.push_back(l);
    }
    return iset;
}
