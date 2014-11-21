/*
 * opencog/atomspace/Atom.h
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

#ifndef _OPENCOG_ATOM_H
#define _OPENCOG_ATOM_H

#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <boost/signals2.hpp>

#include <opencog/util/exceptions.h>

#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/types.h>

class AtomUTest;

namespace opencog
{

/** \addtogroup grp_atomspace
 *  @{
 */

class Link;
typedef std::shared_ptr<Link> LinkPtr;
typedef std::vector<LinkPtr> IncomingSet; // use vector; see below.
typedef std::weak_ptr<Link> WinkPtr;
typedef std::set<WinkPtr, std::owner_less<WinkPtr> > WincomingSet;
typedef boost::signals2::signal<void (AtomPtr, LinkPtr)> AtomPairSignal;

// We use a std:vector instead of std::set for IncomingSet, because
// virtually all access will be either insert, or iterate, so we get
// O(1) performance. We use std::set for WincomingSet, because we want
// both good insert and good remove performance.  Note that sometimes
// incoming sets can be huge (millions of atoms).

/**
 * Atoms are the basic implementational unit in the system that
 * represents nodes and links. In terms of inheritance, nodes and
 * links are specialization of atoms, that is, they inherit all
 * properties from atoms.
 */
class Atom
    : public std::enable_shared_from_this<Atom>
{
    friend class ::AtomUTest;     // Needs to call setFlag()
    friend class AtomStorage;     // Needs to set _uuid
    friend class AtomTable;       // Needs to call MarkedForRemoval()
    friend class ImportanceIndex; // Needs to call setFlag()
    friend class Handle;          // Needs to view _uuid
    friend class SavingLoading;   // Needs to set _uuid
    friend class TLB;             // Needs to view _uuid

private:
    //! Sets the AtomTable in which this Atom is inserted.
    void setAtomTable(AtomTable *);

    //! Returns the AtomTable in which this Atom is inserted.
    AtomTable *getAtomTable() const { return _atomTable; }

protected:
    UUID _uuid;
    AtomTable *_atomTable;

    Type _type;

    // Byte of flags (each bit is a flag, see AtomSpaceDefinites.h)
    char _flags;

    TruthValuePtr _truthValue;
    AttentionValuePtr _attentionValue;

    // Lock needed to serialize changes.
    // This costs 40 bytes per atom.
    // Tried using a single, global lock, but there seemed to be too
    // much contention for it, so using a lock-per-atom, even though
    // this makes it kind-of fat.
    std::mutex _mtx;

    /**
     * Constructor for this class.
     *
     * @param The type of the atom.
     * @param Outgoing set of the atom, that is, the set of atoms this
     * atom references. It must be an empty vector if the atom is a node.
     * @param The truthValue of the atom. note: This is not cloned as
     *        in setTruthValue.
     */
    Atom(Type, TruthValuePtr = TruthValue::NULL_TV(),
            AttentionValuePtr = AttentionValue::DEFAULT_AV());

    struct InSet
    {
        // incoming set is not tracked by garbage collector,
        // to avoid cyclic references.
        // std::set<ptr> uses 48 bytes (per atom).
        WincomingSet _iset;
#ifdef INCOMING_SET_SIGNALS
        // Some people want to know if the incoming set has changed...
        // However, these make the atom quite fat, so this is disabled
        // just right now. If users really start clamoring, then we can
        // turn this on.
        AtomPairSignal _addAtomSignal;
        AtomPairSignal _removeAtomSignal;
#endif /* INCOMING_SET_SIGNALS */
    };
    typedef std::shared_ptr<InSet> InSetPtr;
    InSetPtr _incoming_set;
    void keep_incoming_set();
    void drop_incoming_set();

    // Insert and remove links from the incoming set.
    void insert_atom(LinkPtr);
    void remove_atom(LinkPtr);

private:
    /** Returns whether this atom is marked for removal.
     *
     * @return Whether this atom is marked for removal.
     */
    bool isMarkedForRemoval() const;

    /** Returns an atom flag.
     * A byte represents all flags. Each bit is one of them.
     *
     * @param An int indicating which of the flags will be returned.
     * @return A boolean indicating if that flag is set or not.
     */
    bool getFlag(int) const;

    /** Changes the value of the given flag.
     *
     * @param An int indicating which of the flags will be set.
     * @param A boolean indicating the new value of the flag.
     */
    void setFlag(int, bool);

    //! Marks the atom for removal.
    void markForRemoval();

    //! Unsets removal flag.
    void unsetRemovalFlag();

    /** Change the Very-Long-Term Importance */
    void chgVLTI(int unit);

public:

    virtual ~Atom();

    /** Returns the type of the atom.
     *
     * @return The type of the atom.
     */
    inline Type getType() const { return _type; }

    /** Basic predicate */
    bool isType(Type t, bool subclass) const
    {
        Type at(getType());
        if (not subclass) return t == at;
        return classserver().isA(at, t);
    }

    /** Returns the handle of the atom.
     *
     * @return The handle of the atom.
     */
    inline Handle getHandle() {
        return Handle(shared_from_this());
    }

    /** Returns the AttentionValue object of the atom.
     *
     * @return The pointer to the AttentionValue object
     * of the atom.
     */
    AttentionValuePtr getAttentionValue();

    //! Sets the AttentionValue object of the atom.
    void setAttentionValue(AttentionValuePtr) throw (RuntimeException);

    /// Handy-dandy convenience getters for attention values.
    AttentionValue::sti_t getSTI()
    {
        return getAttentionValue()->getSTI();
    }

    AttentionValue::lti_t getLTI()
    {
        return getAttentionValue()->getLTI();
    }

    AttentionValue::vlti_t getVLTI()
    {
        return getAttentionValue()->getVLTI();
    }

    /** Change the Short-Term Importance */
    void setSTI(AttentionValue::sti_t stiValue)
    {
        /* Make a copy */
        AttentionValuePtr old_av = getAttentionValue();
        AttentionValuePtr new_av = createAV(
            stiValue,
            old_av->getLTI(),
            old_av->getVLTI());
        setAttentionValue(new_av);
    }

    /** Change the Long-term Importance */
    void setLTI(AttentionValue::lti_t ltiValue)
    {
        AttentionValuePtr old_av = getAttentionValue();
        AttentionValuePtr new_av = createAV(
            old_av->getSTI(),
            ltiValue,
            old_av->getVLTI());
        setAttentionValue(new_av);
    }

    /** Increase the Very-Long-Term Importance by 1 */
    void incVLTI() { chgVLTI(+1); }

    /** Decrease the Very-Long-Term Importance by 1 */
    void decVLTI() { chgVLTI(-1); }

    /** Returns the TruthValue object of the atom.
     *
     * @return The const referent to the TruthValue object of the atom.
     */
    TruthValuePtr getTruthValue();

    //! Sets the TruthValue object of the atom.
    void setTruthValue(TruthValuePtr);

    /** merge truth value into this */
    void merge(TruthValuePtr);

    //! Get the size of the incoming set.
    size_t getIncomingSetSize();

    //! Return the incoming set of this atom.
    //! The resulting incoming set consists of strong pointers,
    //! that is, to valid, non-null handles that were part of the
    //! incoming set at the time this call was made.
    IncomingSet getIncomingSet();

    //! Place incoming set into STL container of Handles.
    //! Example usage:
    //!     std::vector<Handle> hvect;
    //!     atom->getIncomingSet(back_inserter(hvect));
    //! The resulting vector hvect will contain only valid handles
    //! that were actually part of the incoming set at the time of
    //! the call to this function.
    template <typename OutputIterator> OutputIterator
    getIncomingSet(OutputIterator result)
    {
        if (NULL == _incoming_set) return result;
        std::lock_guard<std::mutex> lck(_mtx);
        // Sigh. I need to compose copy_if with transform. I could
        // do this wih boost range adaptors, but I don't feel like it.
        auto end = _incoming_set->_iset.end();
        for (auto w = _incoming_set->_iset.begin(); w != end; w++)
        {
            Handle h(w->lock());
            if (h) { *result = h; result ++; }
        }
        return result;
    }

    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     * That is, returns the incoming set of Handle h, with some optional
     * filtering.
     *
     * @param The handle that must be in the outgoing set of the atom.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type with the given handle
     *         in their outgoing set.
     */
    template <typename OutputIterator> OutputIterator
    getIncomingSetByType(OutputIterator result,
                         Type type, bool subclass = false)
    {
        if (NULL == _incoming_set) return result;
        std::lock_guard<std::mutex> lck(_mtx);
        // Sigh. I need to compose copy_if with transform. I could
        // do this wih boost range adaptors, but I don't feel like it.
        auto end = _incoming_set->_iset.end();
        for (auto w = _incoming_set->_iset.begin(); w != end; w++)
        {
            Handle h(w->lock());
            if (h and h->isType(type, subclass)) {
                *result = h;
                result ++;
            }
        }
        return result;
    }


    /** Returns a string representation of the node.
     *
     * @return A string representation of the node.
     * cannot be const, because observing the TV and AV requires a lock.
     */
    virtual std::string toString(std::string indent = "") = 0;
    virtual std::string toShortString(std::string indent = "") = 0;

    /** Returns whether two atoms are equal.
     *
     * @return true if the atoms are equal, false otherwise.
     */
    virtual bool operator==(const Atom&) const = 0;

    /** Returns whether two atoms are different.
     *
     * @return true if the atoms are different, false otherwise.
     */
    virtual bool operator!=(const Atom&) const = 0;


};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOM_H
