/**
 * AtomTable.h
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#ifndef ATOMTABLE_H
#define ATOMTABLE_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "Atom.h"
#include "classes.h"
#include "HandleEntry.h"
#include "HandleIterator.h"
#include "types.h"
#include "HandleMap.h"
#include "PredicateEvaluator.h"
#include <AttentionValue.h>
#include <iostream>
#include "exceptions.h"

#ifndef WIN32
#include <ext/hash_set>
using __gnu_cxx::hash_set;
#endif

struct hashAtom{
    int operator()(Atom* a) const;
};
struct eqAtom{
    bool operator()(Atom* a1, Atom* a2) const;
};
typedef hash_set<Atom*, hashAtom, eqAtom> AtomHashSet;

class HandleEntry;
class HandleIterator;
class SavingLoading;

class PredicateEvaluator;

/**
 * This class provides mechanisms to store atoms and keep indices for
 * efficient lookups. It implements the local storage data structure of
 * OpenCog. It contains methods to add and remove atoms, as well as to
 * retrieve specific sets according to different criteria.
 */
class AtomTable {

    friend class ImportanceUpdatingAgent;
    friend class SavingLoading;
    friend class AtomSpace;

private:

    int size;

    AtomHashSet* atomSet;

    /**
     * Indicates whether DynamicStatisticsAgent should be used
     * for atoms inserted in this table or not.
     */
    bool useDSA;

    // linked lists for each kind of index
    std::vector<Handle> typeIndex;
    std::vector<Handle> targetTypeIndex;
    std::vector<Handle> nameIndex;
    std::vector<Handle> importanceIndex;
    std::vector<Handle> predicateIndex;
    std::vector<Handle> predicateHandles;
    std::vector<PredicateEvaluator*> predicateEvaluators;

    // Number of predicate indices.
    int numberOfPredicateIndices;
    // Map from each Tree PredicateNode Handle to its corresponding index
    HandleMap<int>* predicateHandles2Indices;

    unsigned int strHash(const char*) const;
    inline unsigned int getNameHash(Atom* atom) const;

    std::vector<HandleIterator*> iterators;

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_t iteratorsLock;
#endif

    void removeFromIndex(Atom *, std::vector<Handle>&, int, int)
        throw (RuntimeException);
    void removeFromTargetTypeIndex(Atom *);
    void removeFromPredicateIndex(Atom *);
    void removeFromIterator(Atom *, HandleIterator *);
    void lockIterators();
    void unlockIterators();
    void decayAtomShortTermImportance(Atom *) __attribute__ ((deprecated));

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

public:

    // JUST FOR TESTS:
    bool isCleared() const;

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
     *        subclass of the given type.
     * If false, matches only atoms of the exact type.
     */
    void print(std::ostream& output = std::cout,
               Type type = ATOM,
               bool subclass = true) const;

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
     * Builds a set of atoms of a given type or with targets of a given
     * type depending on the given index head function.
     *
     * @param The desired type criterion.
     * @param Whether the given type must also consider its subclasses.
     * @param A method pointer to either the function that returns a type
     * index head, or the function that returns a target type index head.
     * @param Which index to be followed.
     * @return The set of atoms created based of the given criteria.
     */
    HandleEntry* buildSet(Type,
                          bool,
                          Handle(AtomTable::*)(Type) const,
                          int) const;

    /**
     * Returns the index head for the given type.
     *
     * @param The type whose index head will be returned.
     * @return The index head for the given type.
     */
    Handle getTypeIndexHead(Type) const;

    /**
     * Returns the index head for the given target type.
     *
     * @param The target type whose index head will be returned.
     * @return The index head for the given target type.
     */
    Handle getTargetTypeIndexHead(Type) const;

    /**
     * Returns the index head for the given name.
     *
     * @param The name whose index head will be returned.
     * @return The index head for the given name.
     */
    Handle getNameIndexHead(const char*) const;

    /**
     * Returns the index head for the given importance bin.
     *
     * @param The importance bin whose index head will be returned.
     * @return The index head for the given importance bin.
     */
    Handle getImportanceIndexHead(int) const;

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
    void addPredicateIndex(Handle, PredicateEvaluator*)
        throw (InvalidParamException);

    /**
     * Returns the index head for the given Functionspredicate index.
     *
     * @param The predicate index whose index head will be returned.
     * @return The index head for the given importance bin.
     */
    Handle getPredicateIndexHead(int index) const;

    /**
     * Returns the Predicate evaluator for a given
     * GroundedPredicateNode Handle, if it is being used as a
     * lookup index. Otherwise, returns NULL.
     */
    PredicateEvaluator* getPredicateEvaluator(Handle gpnHandle) const;

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
    HandleEntry* findHandlesByGPN(Handle,
          VersionHandle = NULL_VERSION_HANDLE) const;

    /**
     * Returns the exact atom for the given name and type.
     * Note: Type must inherit from NODE. Otherwise, it returns
     * UNDEFINED_HANDLE.
     *
     * @param The name of the desired atom.
     * @param The type of the desired atom.
     * @return The handle of the desired atom if found.
     */
    Handle getHandle(const char*, Type) const;

    /**
     * Returns the set of atoms of a given type (subclasses optionally).
     *
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @return The set of atoms of a given type (subclasses optionally).
     */
    HandleEntry* getHandleSet(Type, bool subclass = false) const;

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
    HandleEntry* getHandleSet(Type, Type,
          bool subclass = false, bool targetSubclass = false) const;

    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     *
     * @param The handle that must be in the outgoing set of the atom.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type with the given handle in
     * their outgoing set.
     */
    HandleEntry* getHandleSet(Handle,
                              Type type = ATOM,
                              bool subclass = true) const;

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
    HandleEntry* getHandleSet(const char*,
                Type type = ATOM, bool subclass = true) const;

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
    HandleEntry* getHandleSet(AttentionValue::sti_t,
                              AttentionValue::sti_t upperBound = 32767) const;


    /**
     * Updates the importance index for the given atom. According to the
     * new importance of the atom, it may change importance bins.
     *
     * @param The atom whose importance index will be updated.
     * @param The old importance bin where the atom originally was.
     */
     bool updateImportanceIndex(Atom*, int);

    /**
     * Adds a new atom to the table, checking for duplicates and merging
     * when necessary.
     *
     * @param The new atom to be added.
     * @return The handle of the newly added atom.
     */
    Handle add(Atom*) throw (RuntimeException);

    /**
     * Merges two equivalent atoms into one, and deletes the copy. Atoms
     * must be of same type.
     * @param Original atom.
     * @param Copy to be merged to original atom.
     */
    void merge(Atom*, Atom*);

    /**
     * Removes atoms from the table.
     *
     * @param The atom to be removed.
     * @param Recursive-removal flag; if set, the links in the incoming
     *        set will also be removed.
     * @return True if the removal operation was successful. False, otherwise.
     */
    bool remove(Handle, bool recursive = false);

    /**
     * This method returns which importance bin an atom with the given
     * importance should be placed.
     *
     * @param Importance value to be mapped.
     * @return The importance bin which an atom of the given importance
     * should be placed.
     */
    static unsigned int importanceBin(short);

    /**
     * Returns the mean importance value for the given importance bin (the
     * average between the lower and upper importance bounds for the bin).
     *
     * @param Importance bin to be mapped.
     * @return The mean importance value for the given importance bin.
     */
    static float importanceBinMeanValue(unsigned int);

    /**
     * Decays importance of all atoms in the table, reindexing
     * importanceIndex accordingly.
     */
    void decayShortTermImportance() throw (RuntimeException) __attribute__ ((deprecated));

    /**
     * Returns whether DynamicsStatisticsAgent is to be used with
     * this table or not.
     */
    bool usesDSA() const;

    HandleEntry* getHandleSet(Type type, bool subclass, VersionHandle vh) const;
    HandleEntry* getHandleSet(Type type, Type targetType, bool subclass, bool targetSubclass, VersionHandle vh, VersionHandle targetVh) const;
    HandleEntry* getHandleSet(Handle handle, Type type, bool subclass, VersionHandle vh) const;
    HandleEntry* getHandleSet(const std::vector<Handle>& handles, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const;
    HandleEntry* getHandleSet(const char* name, Type type, bool subclass, VersionHandle vh) const;
    HandleEntry* getHandleSet(const char* targetName, Type targetType, Type type, bool subclass, VersionHandle vh, VersionHandle targetVh) const;
    HandleEntry* getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const;
    HandleEntry* getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const;

    /*
    * If this method is needed it needs to be refactored to use
    * AttentionValue instead of floats
    HandleEntry* getHandleSet(float lowerBound, float upperBound, VersionHandle vh) const;
    */


};

#endif
