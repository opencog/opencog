/*
 * opencog/atomspace/AtomTable.h
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#ifndef _OPENCOG_ATOMTABLE_H
#define _OPENCOG_ATOMTABLE_H

#include <iostream>
#include <vector>
#include <unordered_set>

#include <boost/signal.hpp>

#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/FixedIntegerIndex.h>
#include <opencog/atomspace/HandleEntry.h>
#include <opencog/atomspace/HandleIterator.h>
#include <opencog/atomspace/ImportanceIndex.h>
#include <opencog/atomspace/IncomingIndex.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/LinkIndex.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/NodeIndex.h>
#include <opencog/atomspace/PredicateEvaluator.h>
#include <opencog/atomspace/PredicateIndex.h>
#include <opencog/atomspace/TypeIndex.h>
#include <opencog/atomspace/TargetTypeIndex.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/platform.h>

namespace opencog
{

struct atom_ptr_hash : public std::unary_function<const Atom*, std::size_t>
{
    std::size_t operator()(const Atom* const& __x) const;
};
struct atom_ptr_equal_to : public std::binary_function<const Atom*, const Atom*, bool>
{
    bool operator()(const Atom* const& __x, const Atom* const& __y) const;
};
typedef std::unordered_set<const Atom*, atom_ptr_hash, atom_ptr_equal_to> AtomHashSet;

class SavingLoading;

/**
 * This class provides mechanisms to store atoms and keep indices for
 * efficient lookups. It implements the local storage data structure of
 * OpenCog. It contains methods to add and remove atoms, as well as to
 * retrieve specific sets according to different criteria.
 */
class AtomTable
{
    friend class ImportanceUpdatingAgent;
    friend class SavingLoading;
    friend class AtomSpace;
    friend class AtomSpaceImpl;
    friend class HandleIterator;
    friend class ::AtomTableUTest;

private:

    int size;

    /**
     * Lookup table ... XXX is this really needed? The various indexes
     * below should be enough, maybe? This should probably be eliminated
     * if possible.
     */
    AtomHashSet atomSet;

    /**
     * Indicates whether DynamicStatisticsAgent should be used
     * for atoms inserted in this table or not.
     */
    bool useDSA;

    /**
     * Indexes for quick retreival of certain kinds of atoms.
     */
    TypeIndex typeIndex;
    NodeIndex nodeIndex;
    LinkIndex linkIndex;
    IncomingIndex incomingIndex;
    ImportanceIndex importanceIndex;
    TargetTypeIndex targetTypeIndex;
    PredicateIndex predicateIndex;

    /** iterators, used in an (incomplete) attempt at thread-safety. */
    std::vector<HandleIterator*> iterators;
    void lockIterators();
    void unlockIterators();
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_t iteratorsLock;
#endif

    /**
     * signal connection used to keep track of atom type addition in the
     * ClassServer 
     */
    boost::signals::connection addedTypeConnection; 

    /**
     * Handler of the 'type added' signal from ClassServer
     */
    void typeAdded(Type);

    static bool decayed(Handle h);
    // Warning, this should only be called by decayShortTermImportance
    void clearIndexesAndRemoveAtoms(HandleEntry* extractedHandles);

    /**
     * Extracts atoms from the table. Table will not contain the
     * extracted atoms anymore, but they will not be deleted.
     * Instead, they are returned by this method.
     *
     * Note: The caller is responsible for releasing the memory of
     * both the returned list and the refered Atoms inside it.
     *
     * @param The atom to be extracted.
     * @param Recursive-removal flag; if set, the links in the
     *        incoming set will also be extracted.
     * @return A list with the Handles of all extracted Atoms.
     */
    HandleEntry* extract(Handle, bool recursive = false);

    /**
     * Removes the previously extracted Handles (using the extract
     * method) from this table.
     * @param The list of the Handles previously extracted.
     *
     * NOTE: This method also frees the memory of both list
     * and Atom objects corresponding to the Handles inside the list.
     * XXX Huh ??? What if there are other things referenceing these
     * atoms/handles?
     */
    void removeExtractedHandles(HandleEntry* extractedHandles);

    // JUST FOR TESTS:
    bool isCleared() const;

    /**
     * Overrides and declares copy constructor and equals operator as private 
     * for avoiding large object copying by mistake.
     */
    AtomTable& operator=(const AtomTable&);
    AtomTable(const AtomTable&);

public:

    /**
     * Constructor and destructor for this class.
     */
    AtomTable(bool dsa = true);
    ~AtomTable();

    /**
     * Prints atoms of this AtomTable to the given output stream.
     * @param output  the output stream where the atoms will be printed
     * @param type  the type of atoms that should be printed
     * @param subclass  if true, matches all atoms whose type is
     *        subclass of the given type. If false, matches only atoms of the
     *        exact type.
     */
    void print(std::ostream& output = std::cout,
               Type type = ATOM,
               bool subclass = true) const;

    /**
     * Prints atoms of this AtomTable though the given logger
     * @param logger the logger used to print the atoms
     * @param type  the type of atoms that should be printed
     * @param subclass  if true, matches all atoms whose type is
     *        subclass of the given type. If false, matches only atoms of the
     *        exact type.
     */
    void log(Logger& l = logger(), Type t = ATOM, bool subclass = true) const;

    /**
     * Return the number of atoms contained in a table.
     */
    int getSize() const;

    /**
     * Registers an iterator. Iterators must be registered because when
     * using a multi-threaded configuration, the contents of an active
     * iterator may become invalid due to the removal of atoms. All
     * registered iterators are then properly notified. This method is
     * automatically called every time a new iterator is created.
     *
     * @param The iterator to be registered.
     */
    void registerIterator(HandleIterator*);

    /**
     * Unregisters an iterator. This method is automatically called every
     * time a new iterator is deleted.
     *
     * @param The iterator to be unregistered.
     */
    void unregisterIterator(HandleIterator*) throw (RuntimeException);

    /**
     * Creates a new handle iterator that iterates on all atoms of the
     * atom table.
     *
     * @return The newly created iterator.
     */
    HandleIterator* getHandleIterator();

    /**
     * Creates a new handle iterator that iterates on atoms of a specific
     * type, and optionally in its subclasses as well.
     *
     * @param The type of atom to be iterated.
     * @param Whether subclasses of the given type are to be iterated as
     * well.
     * @param VersionHandle for filtering the resulting atoms by context.
     *         NULL_VERSION_HANDLE indicates no filtering
     * @return The newly created iterator.
     */
    HandleIterator* getHandleIterator(Type,
                                      bool subclass = false,
                                      VersionHandle vh = NULL_VERSION_HANDLE);

    /**
     * Makes a set from a index head. It receives a linked-list and an
     * index. The received linked-list is the tail of the newly created
     * list, where its head is created by iterating the received index
     * head until the end.
     *
     * @param The tail linked-list.
     * @param The first element of the index linked-list that will be
     * placed in the beginning of the newly created linked-list.
     * @param Which index is to be followed.
     * @return The concatenation of the parameter list with the list
     * created by following given index head.
     */
    HandleEntry* makeSet(HandleEntry*, Handle, int) const;

    /**
     * Adds a new predicate index to this atom table given the Handle of
     * the PredicateNode.
     * @param The handle of the predicate node, whose name is the id
     *        of the predicate.
     * @param The evaluator used to check if such predicate is true
     *        for a given handle.
     * Throws exception if:
     *      - the given Handle is not in the AtomTable
     *      - there is already an index for this predicate id/Handle
     *      - the predicate index table is full.
     * NOTE: Does not apply the new predicate index to the atoms
     * inserted previously in the AtomTable.
     */
    void addPredicateIndex(Handle h, PredicateEvaluator *pe)
    {
        predicateIndex.addPredicateIndex(h,pe);
    }

    /**
     * Returns the Predicate evaluator for a given
     * GroundedPredicateNode Handle, if it is being used as a
     * lookup index. Otherwise, returns NULL.
     */
    PredicateEvaluator* getPredicateEvaluator(Handle h) const
    {
        return predicateIndex.getPredicateEvaluator(h);
    }

    /**
     * Returns a list of handles that matches the GroundedPredicateNode
     * with the given name (id).
     * @param the id of the predicate node.
     * @param VersionHandle for filtering the resulting atoms by
     *        context. NULL_VERSION_HANDLE indicates no filtering
     */
    HandleEntry* findHandlesByGPN(const char*,
                                  VersionHandle = NULL_VERSION_HANDLE) const;

    /**
     * Returns a list of handles that matches the GroundedPredicateNode
     * with the given Handle.
     * @param the Handle of the predicate node.
     * @param VersionHandle for filtering the resulting atoms by
     *       context. NULL_VERSION_HANDLE indicates no filtering
     **/
    HandleEntry* findHandlesByGPN(Handle h,
                                  VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        return predicateIndex.findHandlesByGPN(h, vh);
    }

    /**
     * Returns the exact atom for the given name and type.
     * Note: Type must inherit from NODE. Otherwise, it returns
     * Handle::UNDEFINED.
     *
     * @param The name of the desired atom.
     * @param The type of the desired atom.
     * @return The handle of the desired atom if found.
     */
    Handle getHandle(const char* name, Type t) const;
    Handle getHandle(const Node* n) const;

    Handle getHandle(Type t, const HandleSeq &seq) const;
    Handle getHandle(const Link* l) const;

    /**
     * Returns the set of atoms of a given type (subclasses optionally).
     *
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @return The set of atoms of a given type (subclasses optionally).
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 bool subclass = false) const
    {
        return std::copy_if(typeIndex.begin(type, subclass),
                            typeIndex.end(),
                            result,
             [&](Handle h)->bool{ return h != Handle::UNDEFINED;});
    }

    /**
     * Returns the set of atoms of a given type which have atoms of a
     * given target type in their outgoing set (subclasses optionally).
     *
     * @param The desired type.
     * @param The desired target type.
     * @param Whether type subclasses should be considered.
     * @param Whether target type subclasses should be considered.
     * @return The set of atoms of a given type and target type
     *         (subclasses optionally).
     */
    HandleEntry* getHandleSet(Type type, Type targetType,
                              bool subclass = false,
                              bool targetSubclass = false) const
    {
        HandleEntry *set = HandleEntry::fromHandleSet(targetTypeIndex.getHandleSet(targetType,targetSubclass));
        return HandleEntry::filterSet(set, type, subclass);
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
     * @return The set of atoms of the given type with the given handle in
     * their outgoing set.
     */
    HandleEntry* getHandleSet(Handle h,
                              Type type = ATOM,
                              bool subclass = true) const;


    /**
     * Return the incoming set associated with handle h.
     */
    HandleEntry* getIncomingSet(Handle h) const;

    /**
     * Returns the set of atoms with the given target handles and types
     * (order is considered) in their outgoing sets, where the type and
     * subclasses of the atoms are optional.
     *
     * @param  An array of handles to match the outgoing sets of the
     *         searched atoms. This array can be empty (or each of its
     *         elements can be null), if the handle value does not
     *         matter or if it does not apply to the specific search.
     *         Note that if this array is not empty, it must contain
     *         "arity" elements.
     * @param  An array of target types to match the types of the atoms
     *         in the outgoing set of searched atoms.
     * @param  An array of boolean values indicating whether each of the
     *         above types must also consider subclasses. This array can
     *         be null, which means that subclasses will not be considered.
     *         Note that if this array is not null, it must contains
     *         "arity" elements.
     * @param  The length of the outgoing set of the atoms being searched.
     * @param  The optional type of the atom.
     * @param  Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type with the matching
     *         criteria in their outgoing set.
     */
    HandleEntry* getHandleSet(const std::vector<Handle>&,
                              Type*, bool*, Arity,
                              Type type = ATOM,
                              bool subclass = true) const;

    /**
     * Returns the set of atoms of a given name (atom type and subclasses
     * optionally).
     *
     * @param The desired name of the atoms.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type and name.
     */
    HandleEntry* getHandleSet(const char* name,
                              Type type = ATOM, bool subclass = true) const
    {
        if (name == NULL || *name == 0)
        {
            HandleEntry *set = HandleEntry::fromHandleSet(typeIndex.getHandleSet(type, subclass));
            return HandleEntry::filterSet(set, "");
        }
        return HandleEntry::fromHandleSet(nodeIndex.getHandleSet(type, name, subclass));
    }

    /**
     * Returns the set of atoms whose outgoing set contains at least one
     * atom with the given name and type (atom type and subclasses
     * optionally).
     *
     * @param The name of the atom in the outgoing set of the searched
     *        atoms.
     * @param The type of the atom in the outgoing set of the searched
     *        atoms.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type and name whose outgoing
     *         set contains at least one atom of the given type and name.
     */
    HandleEntry* getHandleSet(const char*, Type,
                              Type type = ATOM, bool subclass = true) const;

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param An array of names to match the outgoing sets of the searched
     * atoms. This array (or each of its elements) can be null, if
     * the names do not matter or if do not apply to the specific search.
     * Note that if this array is not null, it must contain "arity" elements.
     * @param An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. If array of names is not null,
     * this parameter *cannot* be null as well. Besides, if an element in a
     * specific position in the array of names is not null, the corresponding
     * type element in this array *cannot* be NOTYPE as well.
     * @param An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param The length of the outgoing set of the atoms being searched.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     */
    HandleEntry* getHandleSet(const char**, Type*, bool*, Arity,
                              Type type = ATOM, bool subclass = true) const
    throw (RuntimeException);

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. This parameter (as well as any of
     * its elements can be NOTYPE), what means that the type doesnt matter.
     * Not that if this array is not null, it must contains "arity" elements.
     * @param An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param The length of the outgoing set of the atoms being searched.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     */
    HandleEntry* getHandleSet(Type*, bool*, Arity,
                              Type type = ATOM, bool subclass = true) const;

    /**
     * Returns the set of atoms within the given importance range.
     *
     * @param Importance range lower bound (inclusive).
     * @param Importance range upper bound (inclusive).
     * @return The set of atoms within the given importance range.
     */
    HandleEntry* getHandleSet(AttentionValue::sti_t lowerBound,
                              AttentionValue::sti_t upperBound = 32767) const
    {
        return importanceIndex.getHandleSet(lowerBound, upperBound);
    }

    HandleEntry* getPredicateHandleSet(int index)
    {
        return predicateIndex.getHandleSet(index);
    }

    /**
     * Updates the importance index for the given atom. According to the
     * new importance of the atom, it may change importance bins.
     *
     * @param The atom whose importance index will be updated.
     * @param The old importance bin where the atom originally was.
     */
    void updateImportanceIndex(Atom* a, int bin)
    {
        importanceIndex.updateImportance(a,bin);
    }

    /**
     * Merge the existing atom with the given handle with the given truth value.
     * If the handle is valid, emits atom merged signal.
     * @param h     Handle of the Atom to be merged
     * @param tvn   TruthValue to be merged to current atom's truth value.  
     */
    void merge(Handle h, const TruthValue& tvn);

    /**
     * Adds an atom to the table, checking for duplicates and merging
     * when necessary. When an atom is added, the atom table takes over
     * the memory management for that atom. In other words, no other
     * code should ever attempt to delete the pointer that is passed 
     * into this method.
     *
     * @param The new atom to be added.
     * @return The handle of the newly added atom.
     */
    Handle add(Atom*) throw (RuntimeException);

    /**
     * Return true if the atom table holds this handle, else return false.
     */
    bool holds(const Handle& h) const {
        Atom *a = getAtom(h);
        if (NULL == a) return false;
        return true;
    }

    /** Get Atom object already in the AtomTable.
     *
     * @param h Handle of the atom to retrieve.
     * @return pointer to Atom object, NULL if no atom within this AtomTable is
     * associated with handle.
     */
    inline Atom* getAtom(const Handle& h) const {
        if (h == Handle::UNDEFINED) return NULL;
        Atom *atom = TLB::getAtom(h);
        if (atom)
            // if the atom isn't linked to this AtomTable
            // then blank pointer
            if (this != atom->getAtomTable()) atom = NULL;
        return atom;
    }

    /** Get Node object already in the AtomTable.
     *
     * @param h Handle of the node to retrieve.
     * @return pointer to Node object, NULL if no atom within this AtomTable is
     * associated with handle or if the atom is a link.
     */
    inline Node* getNode(const Handle& h) const {
        return dynamic_cast<Node*>(getAtom(h));
    }

    /** Get Link object already in the AtomTable.
     *
     * @param h Handle of the link to retrieve.
     * @return pointer to Link object, NULL if no atom within this AtomTable is
     * associated with handle or if the atom is a node.
     */
    inline Link* getLink(const Handle& h) const {
        return dynamic_cast<Link*>(getAtom(h));
    }

    /**
     * Removes atom from the table.
     *
     * @param The atom to be removed.
     * @param Recursive-removal flag; if set, the links in the incoming
     *        set will also be removed.
     * @return True if the removal operation was successful. False, otherwise.
     */
    bool remove(Handle, bool recursive = false);


    /**
     * Return a random atom in the AtomTable.
     * @note Uses the atomSet buckets to provide reasonbly quick look-up.
     */
    Handle getRandom(RandGen* rng) const;

    /**
     * Decays importance of all atoms in the table, reindexing
     * importanceIndex accordingly and extracting the atoms that fall
     * below the "LOWER_STI_VALUE" threshold.
     * @return the list of the handles that should be removed.
     */
    HandleEntry* decayShortTermImportance();

    /**
     * Returns whether DynamicsStatisticsAgent is to be used with
     * this table or not.
     */
    bool usesDSA() const;

    HandleEntry* getHandleSet(Type type, bool subclass, VersionHandle vh) const;
    HandleEntry* getHandleSet(Type type, Type targetType, bool subclass, bool
            targetSubclass, VersionHandle vh, VersionHandle targetVh) const;
    HandleEntry* getHandleSet(Handle handle, Type type, bool subclass,
            VersionHandle vh) const;
    HandleEntry* getHandleSet(const std::vector<Handle>& handles, Type* types,
            bool* subclasses, Arity arity, Type type, bool subclass,
            VersionHandle vh) const;
    HandleEntry* getHandleSet(const char* name, Type type, bool subclass,
            VersionHandle vh) const;
    HandleEntry* getHandleSet(const char* targetName, Type targetType,
            Type type, bool subclass, VersionHandle vh,
            VersionHandle targetVh) const;
    HandleEntry* getHandleSet(const char** names, Type* types, bool*
            subclasses, Arity arity, Type type, bool subclass,
            VersionHandle vh) const;
    HandleEntry* getHandleSet(Type* types, bool* subclasses, Arity arity,
            Type type, bool subclass, VersionHandle vh) const;

    /*
    * If this method is needed it needs to be refactored to use
    * AttentionValue instead of floats
    HandleEntry* getHandleSet(float lowerBound, float upperBound, VersionHandle vh) const;
    */

    /**
     * Invoke the callback cb for *every* atom in the AtomTable
     * This assumes that the callback does *not* modify the AtomTable,
     * specifically, does not insert or remove atoms from the atom table.
     */
    template<class T>
    inline bool foreach_atom(bool (T::*cb)(const Atom *), T *data) const
    {
        AtomHashSet::const_iterator it;
        for (it = atomSet.begin(); it != atomSet.end(); it++) {
            const Atom* atom = *it;
            bool rc = (data->*cb)(atom);
            if (rc) return rc;
        }
        return false;
    }
};

} //namespace opencog

#endif // _OPENCOG_ATOMTABLE_H
