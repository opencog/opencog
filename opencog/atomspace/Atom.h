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

#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/classes.h>
#include <opencog/atomspace/HandleEntry.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/util/exceptions.h>

// #define USE_STD_VECTOR_FOR_OUTGOING
// #define PUT_OUTGOING_SET_IN_LINKS

namespace opencog
{

typedef struct _predicateIndexStruct {
    unsigned long predicateIndexMask;
    Handle* predicateIndex;
} PredicateIndexStruct;

class AtomTable;
class HandleEntry;

/**
 * Atoms are the basic implementational unit in the system that
 * represents nodes and links. In terms of inheritance, nodes and
 * links are specialization of atoms, that is, they inherit all
 * properties from atoms.
 */
class Atom
{

    friend class SavingLoading;
    friend class AtomTable;
    friend class NMXmlParser;

private:

    // Called by constructors to init this object
    void init(Type, const std::vector<Handle>&, const TruthValue&);

#ifndef PUT_OUTGOING_SET_IN_LINKS
    // Adds a new handle to the outgoing set. Note that this is
    // used only in the NativeParser friend class, and, due to
    // performance issues, it should not be used anywhere else...
    void addOutgoingAtom(Handle h);
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    // Used by AtomTable::decayShortTermImportance() and
    // by EconomicAttentionAllocation.
    AttentionValue* getAVPointer();

    /**
     * Sets the AtomTable in which this Atom is inserted.
     */
    void setAtomTable(AtomTable *);

    /**
     * Returns the AtomTable in which this Atom is inserted.
     */
    AtomTable *getAtomTable() const;

protected:

    // properties

    Type type;

    AtomTable *atomTable;

    // connectivity

    // Linked-list that dynamically changes as new atoms are inserted.
    HandleEntry *incoming;


#ifndef PUT_OUTGOING_SET_IN_LINKS
#ifdef USE_STD_VECTOR_FOR_OUTGOING
    // Array that does not change during atom lifespan.
    std::vector<Handle> outgoing;
#else
    Arity arity;
    // Pointer array that does not change during atom lifespan.
    Handle *outgoing;
#endif
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    // indices
    Handle* indices;
    Handle* targetTypeIndex;
    PredicateIndexStruct* predicateIndexInfo;

    // state
    AttentionValue *attentionValue;
    TruthValue *truthValue;

    //ShortFloat importance;
    //ShortFloat heat;

    char flags;

    /**
     * Constructor for this class.
     *
     * @param The type of the atom.
     * @param Outgoing set of the atom, that is, the set of atoms this
     * atom references. It must be an empty vector if the atom is a node.
     * @param The truthValue of the atom. note: This is not cloned as
     *        in setTruthValue.
     */
    Atom(Type, const std::vector<Handle>&,
         const TruthValue& = TruthValue::NULL_TV());

    Handle getNextHandleInPredicateIndex(int index) const;
    void setNextHandleInPredicateIndex(int index, Handle nextHandle);

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
     * Tells whether the atom is real or not (a type designator)
     */
    bool isReal() const;

    /**
     * Returns the type of the atom.
     *
     * @return The type of the atom.
     */
    inline Type getType() const {
        // TODO: Implement smarter mapping from atom types to
        // BuiltInTypeHandle IDs?
        if (isReal())
            return type;
        else
            return (Type)((long)this);
    }

    /**
     * Returns the arity of the atom.
     *
     * @return The arity of the atom.
     */
#ifndef PUT_OUTGOING_SET_IN_LINKS
    inline Arity getArity(void) const {
#ifdef USE_STD_VECTOR_FOR_OUTGOING
        return outgoing.size();
#else
        return arity;
#endif
    }
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    //**
    // * Returns the heat value of the atom.
    // *
    // * @return The heat value of the atom.
    // */
    //float getHeat();

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
     *
     * NOTE: The passed AV object will actually be cloned before
     * being stored internaly. So, if the given AV object is
     * dynamically allocated, the caller is its owner and, so, must
     * take care of memory deallocation.
     *
     * XXX Why do we need this note? It should go without
     * saying, as this is the semantics of the C++ programming
     * language!
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

    ///**
    // * Changes the heat value.
    // *
    // * @param The new heat value.
    // */
    //void setHeat(float);

    ///**
    // * Changes the heat value with no statistics update.
    // *
    // * @param The new heat value.
    // */
    //void rawSetHeat(float);

    ///**
    // * Returns the importance value of the atom.
    // *
    // * @return The importance value of the atom.
    // */
    //float getImportance();

    ///**
    // * Changes the importance value.
    // *
    // * @param The new importance value.
    // */
    //void setImportance(float);
    //
    ///**
    // * Also changes the importance value of this atom, but it does not
    // * changes the importance linked-list to which it belongs, not does it
    // * update statistics.
    // *
    // * @param float The new importance value.
    // */
    //void rawSetImportance(float);

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
#ifdef USE_STD_VECTOR_FOR_OUTGOING
    /**
     * Returns a const reference to the array containing this
     * atom's outgoing set.
     *
     * @return A const reference to this atom's outgoing set.
     */
    inline const std::vector<Handle>& getOutgoingSet() const {
        return outgoing;
    }
#else
    /**
    * Returns a pointer to an array containing this atom's
    * outgoing set.
    *
    * XXX This is rather inefficient, as the -on-stack
    * allocation requires two (!!) copies.
    *
    * @return A pointer to an array containing this atom's outgoing set.
    */
    inline std::vector<Handle> getOutgoingSet() const {
        std::vector<Handle> result;
        std::copy(&outgoing[0], &outgoing[arity], back_inserter(result));
        return result;
    }
#endif

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

#ifndef PUT_OUTGOING_SET_IN_LINKS
    /**
     * Sets the next entry pointed by this atom in one of the indices
     * linked-lists.
     *
     * @param Which index to change.
     * @param The next entry in the given index linked-list.
     */
    void setNext(int, Handle);
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    /**
     * Returns a pointer to the target types index.
     *
     * @return A pointer to the target types index.
     */
    Handle* getTargetTypeIndex() const;

    /**
     * Sets the next entries pointed by this atom in the target types
     * index linked-list.
     *
     * @param An array of handles representing the next entries in the
     * target types index linked-list.
     */
    void setNextTargetTypeIndex(Handle*);

    /**
     * Adds the next entry pointed by this atom in the given predicate
     * index linked-list.
     *
     * @param A handle representing the next entry in the
     * predicate index linked-list.
     */
    void addNextPredicateIndex(int, Handle);

    /**
     * Merges two atoms.
     *
     * @param A pointer to the atom that will be merged to the current
     * one.
     */
    virtual void merge(Atom*) throw (InconsistenceException);

    /**
     * Returns the next entry in one of the indices linked-lists.
     *
     * @param Which index will be followed.
     * @return The next entry in one of the indices linked-lists.
     */
    Handle next(int index);

#ifndef PUT_OUTGOING_SET_IN_LINKS
    /**
     * Builds the target type index structure according to the types of
     * elements in the outgoing set of the given atom.
     *
     * @return A pointer to target types array built.
     * NOTE: The argument size gets the size of the returned array.
     */
    Type* buildTargetIndexTypes(int *size);

    /**
     * Returns the position of a certain type on the reduced array of
     * target types of an atom.
     *
     * @param The type which will be searched in the reduced target types
     * index array.
     * @return The position of the given type in the reduced array of
     * target types.
     */
    int locateTargetIndexTypes(Type) const;

    /**
     * Returns the number of different target types of an atom.
     *
     * @return The number of different target types of an atom.
     */
    int getTargetTypeIndexSize() const;
#endif /* PUT_OUTGOING_SET_IN_LINKS */

    /**
     * Indicates if this atom has any predicate index information
     *
     * @return true if this atoms is in any predicate index. false otherwise
     */
    bool hasPredicateIndexInfo();

    /**
     * Builds the predicate index structure according to the predicates
     * matched by the given atom.
     *
     * @return A pointer to predicate index array built.
     * NOTE: The argument size gets the size of the returned array.
     */
    int* buildPredicateIndices(int *size) const;

    /**
     * Returns whether this atom is marked for removal.
     *
     * @return Whether this atom is marked for removal.
     */
    bool isMarkedForRemoval() const;

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

    /* tests the atom's sti */
    bool isOld(const AttentionValue::sti_t threshold) const;

    /**
     * Returns neighboring atoms, following links and returning their
     * target sets.
     * @param fanin Whether directional links point to this node sould be
     * consired.
     * @param fanout Whether directional links point from this node to
     * another sould be consired.
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
