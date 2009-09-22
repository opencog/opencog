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

#include <string>

#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/HandleEntry.h>
#include <opencog/atomspace/ImportanceIndex.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/util/exceptions.h>

#define PUT_OUTGOING_SET_IN_LINKS

namespace opencog
{

class AtomTable;
class HandleEntry;

/**
 * Atoms are the basic implementational unit in the system that
 * represents nodes and links. In terms of inheritance, nodes and
 * links are specialization of atoms, that is, they inherit all
 * properties from atoms.
 */
class Atom : public AttentionValueHolder
{
    friend class SavingLoading;
    friend class AtomTable;
    friend class ImportanceIndex;
    friend class NMXmlParser;
    friend class TLB;

private:

    // Called by constructors to init this object
#ifndef PUT_OUTGOING_SET_IN_LINKS
    void init(Type, const std::vector<Handle>&, const TruthValue&);
#else
    void init(Type, const TruthValue&);
#endif

#ifndef PUT_OUTGOING_SET_IN_LINKS
    // Adds a new handle to the outgoing set. Note that this is
    // used only in the NativeParser friend class, and, due to
    // performance issues, it should not be used anywhere else...
    void addOutgoingAtom(Handle h);
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    /**
     * Sets the AtomTable in which this Atom is inserted.
     */
    void setAtomTable(AtomTable *);

    /**
     * Returns the AtomTable in which this Atom is inserted.
     */
    AtomTable *getAtomTable() const {
        return atomTable;
    }

protected:
    Handle handle;
    Type type;

    AtomTable *atomTable;

    // Linked-list that dynamically changes when new links
    // point to this atom.
    HandleEntry *incoming;

#ifndef PUT_OUTGOING_SET_IN_LINKS
    // Array that does not change during atom lifespan.
    std::vector<Handle> outgoing;
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    char flags;

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
#ifndef PUT_OUTGOING_SET_IN_LINKS
    Atom(Type, const std::vector<Handle>&,
         const TruthValue& = TruthValue::NULL_TV());
#else
    Atom(Type, const TruthValue& = TruthValue::NULL_TV());
#endif

#ifndef PUT_OUTGOING_SET_IN_LINKS
    /**
      * Sets the outgoing set of the atom
      * This method can be called only if the atom is not inserted
      * in an AtomTable yet.
      * Otherwise, it throws a RuntimeException.
      */
    virtual void setOutgoingSet(const std::vector<Handle>& o)
    throw (RuntimeException);
#endif /* PUT_OUTGOING_SET_IN_LINKS */

public:

    /**
     * Destructor for this class.
     */
    virtual ~Atom() throw (RuntimeException);

    /**
     * Returns the type of the atom.
     *
     * @return The type of the atom.
     */
    inline Type getType() const {
        return type;
    }

    /**
     * Returns the arity of the atom.
     *
     * @return The arity of the atom.
     */
#ifndef PUT_OUTGOING_SET_IN_LINKS
    inline Arity getArity(void) const {
        return outgoing.size();
    }
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    /**
     * Returns the AttentionValue object of the atom.
     *
     * IMPORTANT NOTE: This method returns a "const reference" to
     * the AV object of the Atom because it must not be changed
     * at all. Otherwise, it will break internal core structures
     * (like search indices). Never cast the result of this
     * method to non-const references or get the pointer to the
     * AV object by using the address-of (&) operator.
     *
     * XXX Why do we need this "important note"? It should go
     * without saying that no programmer should ever do anyting
     * like this for any routine that ever returns a const ref!
     *
     * @return The const referent to the AttentionValue object
     *       of the atom.
     */

    const AttentionValue& getAttentionValue() const;

    /**
     * Sets the AttentionValue object of the atom.
     */
    void setAttentionValue(const AttentionValue&) throw (RuntimeException);

    /**
     * Returns the TruthValue object of the atom.
     *
     * @return The const referent to the TruthValue object of the atom.
     */
    const TruthValue& getTruthValue() const;

    /**
     * Sets the TruthValue object of the atom.
     */
    void setTruthValue(const TruthValue&);

    /**
     * Returns a pointer to a linked-list containing the atoms that are
     * members of this one's incoming set.
     *
     * @return A pointer to a linked-list containing the atoms that are
     * members of this one's incoming set.
     */
    inline HandleEntry* getIncomingSet() const {
        return incoming;
    }

#ifndef PUT_OUTGOING_SET_IN_LINKS
    /**
     * Returns a const reference to the array containing this
     * atom's outgoing set.
     *
     * @return A const reference to this atom's outgoing set.
     */
    inline const std::vector<Handle>& getOutgoingSet() const {
        return outgoing;
    }
    /**
     * Returns a specific atom in the outgoing set (using the TLB).
     *
     * @param The position of the atom in the array.
     * @return A specific atom in the outgoing set (using the TLB).
     */
    Atom * getOutgoingAtom(int) const throw (RuntimeException);
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    /**
     * Adds a new entry to this atom's incoming set.
     *
     * @param The handle of the atom to be included.
     */
    void addIncomingHandle(Handle);

    /**
     * Removes an entry from this atom's incoming set.
     *
     * @param The handle of the atom to be excluded.
     */
    void removeIncomingHandle(Handle) throw (RuntimeException);

    /**
     * Returns whether this atom is marked for removal.
     *
     * @return Whether this atom is marked for removal.
     */
    bool isMarkedForRemoval() const
    {
        return (flags & MARKED_FOR_REMOVAL) != 0;
    }

    /**
     * Returns a desired atom flag. A byte represents all flags. Each bit
     * is one of them.
     *
     * @param An int indicating which of the flags will be returned.
     * @return A boolean indicating if that flag is set or not.
     */
    bool getFlag(int) const;

    /**
     * Changes the value of the given flag.
     *
     * @param An int indicating which of the flags will be set.
     * @param A boolean indicating the new value of the flag.
     */
    void setFlag(int, bool);

    /**
     * Marks the atom for removal.
     */
    void markForRemoval();

    /**
     * Unsets removal flag.
     */
    void unsetRemovalFlag();

    /** tests the atom's sti 
     * Joel: What is this used for and why is the LTI threshold
     * hardcoded at 1??
     */
    bool isOld(const AttentionValue::sti_t threshold) const
    {
        return ((attentionValue.getSTI() < threshold) &&
                (attentionValue.getLTI() < 1));
    }

    /**
     * Returns neighboring atoms, following links and returning their
     * target sets.
     * @param fanin Whether directional links point to this node should be
     * considered.
     * @param fanout Whether directional links point from this node to
     * another should be considered.
     */
    HandleEntry *getNeighbors(bool fanin = true,
                              bool fanout = true,
                              Type linkType = LINK,
                              bool subClasses = true) const;

    /**
     * Returns a string representation of the node.
     *
     * @return A string representation of the node.
     */
    virtual std::string toString(void) const = 0;
    virtual std::string toShortString(void) const = 0;

    /**
     * Returns whether two atoms are equal.
     * @return true if the atoms are equal, false otherwise.
     */
    virtual bool operator==(const Atom&) const = 0;

    /**
     * Returns whether two atoms are different.
     * @return true if the atoms are different, false otherwise.
     */
    virtual bool operator!=(const Atom&) const = 0;

    /**
     * Returns the hashCode of the Atom.
     * @return an unsigned integer value as the hashCode of the Atom.
     */
    virtual size_t hashCode(void) const = 0;
};

} // namespace opencog

#endif // _OPENCOG_ATOM_H
