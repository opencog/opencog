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

    if (not tv->isNullTv()) _truthValue = tv;

    // XXX FIXME for right now, all atoms will always keep their
    // incoming sets.  In the future, this should only be set by
    // the user, or by the atomtable, when an atom is added to
    // the atomtable.
    keep_incoming_set();
}

Atom::~Atom()
{
    _atomTable = NULL;
    drop_incoming_set();
}

void Atom::setTruthValue(TruthValuePtr tv)
{
    // OK, I think the below is thread-safe, and nees no locking. Why?
    // isNullTV is atomic, because the call-by-pointer is.
    // The shared-pointer swap is atomic, I think (not 100% sure)
    // The check for _atomTable is atomic, and nothing inside that needs
    // to be serialized. So I think we're good (unless swap isn't atomic).
    if (tv->isNullTv()) return;
    _truthValue.swap(tv);  // after swap, tv points at old tv.
    if (_atomTable != NULL) {
        TVCHSigl& tvch = _atomTable->TVChangedSignal();
        tvch(getHandle(), tv, _truthValue);
    }
}

void Atom::setTV(TruthValuePtr new_tv, VersionHandle vh)
{
    // XXX lock here.  I don't get it, but running AtomSpaceAsyncUTest
    // over and over in a loop eventually leads to a crash, deep inside
    // of the smat pointer deref: _truthValue->getType() below.
    // specifically: opencog::Atom::setTV <+528>: callq  *0x30(%rax)
    // and rax contains a garbage value --  a string "node 1" from the
    // unit test.  Looks to me like a bug in the shared_ptr code.
    // Until gcc/boost fixes this ... I guess we have to lock, for now.
    // I'm assuming this is a gcc bug, I just don't see any bug in the
    // code here; it should all be thread-safe and atomic. November 2013
    // This is with gcc version (Ubuntu/Linaro 4.6.4-1ubuntu1~12.04) 4.6.4
    // With this lock, no crash after 150 minutes of testing.
    // Anyway, the point is that this lock is wasteful, and should be
    // un-needed.
    std::lock_guard<std::mutex> lck (_mtx);
    if (!isNullVersionHandle(vh))
    {
        CompositeTruthValuePtr ctv = (_truthValue->getType() == COMPOSITE_TRUTH_VALUE) ?
                            CompositeTruthValue::createCTV(_truthValue) :
                            CompositeTruthValue::createCTV(_truthValue, NULL_VERSION_HANDLE);
        ctv->setVersionedTV(new_tv, vh);
        new_tv = std::static_pointer_cast<TruthValue>(ctv);
    }
    else if (_truthValue->getType() == COMPOSITE_TRUTH_VALUE and
                 new_tv->getType() != COMPOSITE_TRUTH_VALUE)
    {
        CompositeTruthValuePtr ctv(CompositeTruthValue::createCTV(_truthValue));
        ctv->setVersionedTV(new_tv, NULL_VERSION_HANDLE);
        new_tv = std::static_pointer_cast<TruthValue>(ctv);
    }
    setTruthValue(new_tv); // always call setTruthValue to update indices
}

TruthValuePtr Atom::getTV(VersionHandle vh) const
{
    if (isNullVersionHandle(vh)) {
        return _truthValue;
    }
    else if (_truthValue->getType() == COMPOSITE_TRUTH_VALUE)
    {
        return std::dynamic_pointer_cast<CompositeTruthValue>(_truthValue)->getVersionedTV(vh);
    }
    return TruthValue::NULL_TV();
}


void Atom::setMean(float mean) throw (InvalidParamException)
{
    TruthValuePtr newTv = _truthValue;
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

void Atom::setAttentionValue(AttentionValuePtr av) throw (RuntimeException)
{
    if (av == _attentionValue) return;
    if (*av == *_attentionValue) return;

    // I don't think we need to lock here, since I believe that swap
    // is atomic (right??)
    _attentionValue.swap(av);

    if (_atomTable != NULL) {
        std::lock_guard<std::mutex> lck (_avmtx);

        // gets old and new bins
        int oldBin = ImportanceIndex::importanceBin(av->getSTI());
        int newBin = ImportanceIndex::importanceBin(_attentionValue->getSTI());

        // if the atom importance has changed its bin,
        // updates the importance index
        if (oldBin != newBin) {
            AtomPtr a(shared_from_this());
            _atomTable->updateImportanceIndex(a, oldBin);
        }

        // Notify any interested parties that the AV changed.
        AVCHSigl& avch = _atomTable->AVChangedSignal();
        avch(getHandle(), av, _attentionValue);  // av is old, after swap.
    }
}

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
        if (l) iset.insert(l);
    }
    return iset;
}
