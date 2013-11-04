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

#include <boost/signal.hpp>

#include <opencog/util/exceptions.h>

#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/CompositeTruthValue.h>
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
typedef std::set<LinkPtr> IncomingSet;
typedef boost::signal<void (AtomPtr, LinkPtr)> AtomPairSignal;

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
    char _flags;

    TruthValuePtr _truthValue;
    AttentionValuePtr _attentionValue;

    // Lock needed to serialize AV changes.
    // Just right now, we will use a single shared mutex for all
    // locking.  If this causes too much contention, then we can
    // fall back to a non-global lock, at the cost of 40 additional
    // bytes per atom.
    static std::mutex _avmtx;
    static std::mutex _mtx;

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
        IncomingSet _iset;
        // Some people want to know if the incoming set has changed...
        AtomPairSignal _addAtomSignal;
        AtomPairSignal _removeAtomSignal;
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

public:

    virtual ~Atom();

    /** Returns the type of the atom.
     *
     * @return The type of the atom.
     */
    inline Type getType() const { return _type; }

    /** Returns the handle of the atom.
     *
     * @return The handle of the atom.
     */
    inline Handle getHandle() {
        return Handle(shared_from_this());
    }

    /** Returns the AttentionValue object of the atom.
     *
     * @return The const reference to the AttentionValue object
     * of the atom.
     */
    AttentionValuePtr getAttentionValue() const { return _attentionValue; }

    //! Sets the AttentionValue object of the atom.
    void setAttentionValue(AttentionValuePtr) throw (RuntimeException);

    /** Returns the TruthValue object of the atom.
     *
     * @return The const referent to the TruthValue object of the atom.
     */
    TruthValuePtr getTruthValue() const { return _truthValue; }

    //! Sets the TruthValue object of the atom.
    void setTruthValue(TruthValuePtr);

    //! The get,setTV methods deal with versioning. Yuck.
    void setTV(TruthValuePtr, VersionHandle = NULL_VERSION_HANDLE);
    TruthValuePtr getTV(VersionHandle = NULL_VERSION_HANDLE) const;

    /** Change the primary TV's mean */
    void setMean(float) throw (InvalidParamException);

    //! Return the incoming set of this atom.
    IncomingSet getIncomingSet() const;

    /** Returns a string representation of the node.
     *
     * @return A string representation of the node.
     */
    virtual std::string toString(std::string indent = "") const = 0;
    virtual std::string toShortString(std::string indent = "") const = 0;

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
