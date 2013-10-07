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

#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/util/exceptions.h>
#ifdef ZMQ_EXPERIMENT
	#include "ProtocolBufferSerializer.h"
#endif

class AtomUTest;

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class AtomTable;

/**
 * Atoms are the basic implementational unit in the system that
 * represents nodes and links. In terms of inheritance, nodes and
 * links are specialization of atoms, that is, they inherit all
 * properties from atoms.
 */
class Atom : public AttentionValueHolder
{
    friend class CommitAtomASR;   // needs access to clone
    friend class SavingLoading;   // needs to set flags diectly
    friend class AtomTable;
    friend class TLB;
    friend class ::AtomUTest;
#ifdef ZMQ_EXPERIMENT
    friend class ProtocolBufferSerializer;
#endif

private:
#ifdef ZMQ_EXPERIMENT
    Atom() {};
#endif

    //! Sets the AtomTable in which this Atom is inserted.
    void setAtomTable(AtomTable *);

    //! Returns the AtomTable in which this Atom is inserted.
    AtomTable *getAtomTable() const { return atomTable; }

protected:
    /** @todo atoms fundamentally must not be clonable! Yeah? */
    virtual AtomPtr clone() const = 0;

    Handle handle;
    AtomTable *atomTable;

    Type type;
    char flags;

    // XXX TODO: this should be a std::shared_ptr not a raw pointer.
    TruthValue *truthValue;

    /**
     * Constructor for this class.
     *
     * @param The type of the atom.
     * @param Outgoing set of the atom, that is, the set of atoms this
     * atom references. It must be an empty vector if the atom is a node.
     * @param The truthValue of the atom. note: This is not cloned as
     *        in setTruthValue.
     */
    Atom(Type, const TruthValue& = TruthValue::NULL_TV(),
            const AttentionValue& = AttentionValue::DEFAULT_AV());

    struct IncomingSet
    {
        // Just right now, we will use a single shared mutex for all
        // locking on the incoming set.  If this causes too much
        // contention, then we can fall back to a non-global lock,
        // at the cost of 40 additional bytes per atom.
        static std::mutex _mtx;
        // incoming set is not tracked by garbage collector,
        // to avoid cyclic references.
        // std::set<ptr> uses 48 bytes (per atom).
        std::set<LinkPtr> _iset;
    };
    typedef std::shared_ptr<IncomingSet> IncomingSetPtr;
    IncomingSetPtr _incoming_set;
    void keep_incoming_set();
    void drop_incoming_set();

    // Insert and remove links from the incoming set.
    void insert_atom(LinkPtr);
    void remove_atom(LinkPtr);

public:

    virtual ~Atom();

    /** Returns the type of the atom.
     *
     * @return The type of the atom.
     */
    inline Type getType() const { return type; }

    /** Returns the handle of the atom.
     *
     * @return The handle of the atom.
     */
    inline Handle getHandle() const { return handle; }

    /** Returns the AttentionValue object of the atom.
     *
     * @return The const reference to the AttentionValue object
     * of the atom.
     */
    const AttentionValue& getAttentionValue() const;

    //! Sets the AttentionValue object of the atom.
    void setAttentionValue(const AttentionValue&) throw (RuntimeException);

    /** Returns the TruthValue object of the atom.
     *
     * @return The const referent to the TruthValue object of the atom.
     */
    const TruthValue& getTruthValue() const;

    //! Sets the TruthValue object of the atom.
    void setTruthValue(const TruthValue&);

    /** Returns whether this atom is marked for removal.
     *
     * @return Whether this atom is marked for removal.
     */
    bool isMarkedForRemoval() const
    {
        return (flags & MARKED_FOR_REMOVAL) != 0;
    }

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

    /** Returns a string representation of the node.
     *
     * @return A string representation of the node.
     */
    virtual std::string toString(void) const = 0;
    virtual std::string toShortString(void) const = 0;

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
