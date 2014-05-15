/*
 * opencog/atomspace/AtomTable.h
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2013 Linas Vepstas <linasvepstas@gmail.com>
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
#include <set>
#include <vector>

#include <boost/signals2.hpp>

#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/FixedIntegerIndex.h>
#include <opencog/atomspace/ImportanceIndex.h>
#include <opencog/atomspace/IncomingIndex.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/LinkIndex.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/NodeIndex.h>
#include <opencog/atomspace/PredicateEvaluator.h>
#include <opencog/atomspace/PredicateIndex.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/TypeIndex.h>
#include <opencog/atomspace/TargetTypeIndex.h>
#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/exceptions.h>

class AtomTableUTest;

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

typedef std::set<AtomPtr> AtomPtrSet;

typedef boost::signals2::signal<void (const Handle&)> AtomSignal;
typedef boost::signals2::signal<void (const AtomPtr&)> AtomPtrSignal;
typedef boost::signals2::signal<void (const Handle&,
                                      const AttentionValuePtr&,
                                      const AttentionValuePtr&)> AVCHSigl;
typedef boost::signals2::signal<void (const Handle&, 
                                      const TruthValuePtr&,
                                      const TruthValuePtr&)> TVCHSigl;

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
    friend class ::AtomTableUTest;

private:

    // Single, global mutex for locking the indexes.
    // Its recursive because we need to lock twice during atom insertion
    // and removal: we need to keep the indexes stable while we search
    // them during add/remove.
    static std::recursive_mutex _mtx;

    size_t size;

    // Holds all atoms in the table.  Provides lookup between numeric
    // handle uuid and the actual atom pointer (since they are stored
    // together).  To some degree, this info is duplicated in the Node
    // and LinkIndex below; we have this here for convenience.
    std::set<Handle, handle_less> _atom_set;

    //!@{
    //! Index for quick retreival of certain kinds of atoms.
    TypeIndex typeIndex;
    NodeIndex nodeIndex;
    LinkIndex linkIndex;
    ImportanceIndex importanceIndex;
    TargetTypeIndex targetTypeIndex;
    PredicateIndex predicateIndex;
	//!@}
	
    /**
     * signal connection used to find out about atom type additions in the
     * ClassServer
     */
    boost::signals2::connection addedTypeConnection;

    /** Handler of the 'type added' signal from ClassServer */
    void typeAdded(Type);

    /** Provided signals */
    AtomSignal _addAtomSignal;
    AtomPtrSignal _removeAtomSignal;

    /** Signal emitted when the TV changes. */
    TVCHSigl _TVChangedSignal;

    /** Signal emitted when the AV changes. */
    AVCHSigl _AVChangedSignal;

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
    AtomTable();
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
    size_t getSize() const;
    size_t getNumNodes() const;
    size_t getNumLinks() const;

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
     * \note Does not apply the new predicate index to the atoms
     * inserted previously in the AtomTable.
     */
    void addPredicateIndex(Handle& h, PredicateEvaluator *pe)
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        predicateIndex.addPredicateIndex(h, pe);
    }

    /**
     * Returns the Predicate evaluator for a given
     * GroundedPredicateNode Handle, if it is being used as a
     * lookup index. Otherwise, returns NULL.
     */
    PredicateEvaluator* getPredicateEvaluator(Handle& h) const
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        return predicateIndex.getPredicateEvaluator(h);
    }

    /**
     * Returns a list of handles that matches the GroundedPredicateNode
     * with the given name (id).
     * @param the id of the predicate node.
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByGPN(OutputIterator result,
                    const std::string& gpnNodeName) const
    {
        Handle gpnHandle(getHandle(GROUNDED_PREDICATE_NODE, gpnNodeName));
        return getHandlesByGPN(result, gpnHandle);
    }

    /**
     * Returns a list of handles that matches the GroundedPredicateNode
     * with the given Handle.
     * @param the Handle of the predicate node.
     **/
    template <typename OutputIterator> OutputIterator
    getHandlesByGPN(OutputIterator result,
                    Handle& h) const
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        const UnorderedHandleSet& hs = predicateIndex.findHandlesByGPN(h);
        return std::copy(hs.begin(), hs.end(), result);
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
    Handle getHandle(Type, const std::string&) const;
    Handle getHandle(NodePtr) const;

    Handle getHandle(Type, const HandleSeq&) const;
    Handle getHandle(LinkPtr) const;
    Handle getHandle(AtomPtr) const;
    Handle getHandle(Handle&) const;

protected:
    /* Some basic predicates */
    static bool isDefined(Handle h) { return h != Handle::UNDEFINED; }
    bool hasNullName(Handle& h) const
    {
        if (LinkCast(h)) return true;
        if (NodeCast(h)->getName().c_str()[0] == 0) return true;
        return false;
    }

public:
    /**
     * Returns the set of atoms of a given type (subclasses optionally).
     *
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @return The set of atoms of a given type (subclasses optionally).
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByType(OutputIterator result,
                       Type type,
                       bool subclass = false) const
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        return std::copy_if(typeIndex.begin(type, subclass),
                            typeIndex.end(),
                            result,
                            isDefined);
    }

    /** Calls function 'func' on all atoms */
    template <typename Function> void
    foreachHandleByType(Function func,
                        Type type,
                        bool subclass = false) const
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        std::for_each(typeIndex.begin(type, subclass),
                      typeIndex.end(),
             [&](Handle h)->void {
                  if (not isDefined(h)) return;
                  (func)(h);
             });
    }

    /**
     * Returns all atoms satisfying the predicate
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByTypePred(OutputIterator result,
                         Type type,
                         bool subclass,
                         AtomPredicate* pred) const
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        return std::copy_if(typeIndex.begin(type, subclass),
                            typeIndex.end(),
                            result,
             [&](Handle h)->bool {
                  return isDefined(h) and (*pred)(h);
             });
    }

    template <typename OutputIterator> OutputIterator
    getHandlesByPred(OutputIterator result,
                     AtomPredicate* pred) const
    {
        return getHandlesByTypePred(result, ATOM, true, pred);
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
    template <typename OutputIterator> OutputIterator
    getHandlesByTargetType(OutputIterator result,
                             Type type,
                             Type targetType,
                             bool subclass,
                             bool targetSubclass) const
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        return std::copy_if(targetTypeIndex.begin(targetType, targetSubclass),
                            targetTypeIndex.end(),
                            result,
             [&](Handle h)->bool{
                 return isDefined(h) and h->isType(type, subclass);
             });
    }

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
    UnorderedHandleSet getHandlesByOutgoing(const std::vector<Handle>&,
                              Type*, bool*, Arity,
                              Type type = ATOM,
                              bool subclass = true) const;

    /**
     * Returns the set of atoms of a given name (atom type and subclasses
     * optionally).  If the name is not null or the empty string, then
     * this returns Nodes ONLY (of the requested name, of course). However,
     * if the name is null (or empty string) then Links might be included!
     * This behaviour is surprising, but is explicilty tested for in the
     * AtomSpaceImplUTest. I don't know why its done like this.
     *
     * @param The desired name of the atoms.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type and name.
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByName(OutputIterator result,
                     const std::string& name,
                     Type type = ATOM,
                     bool subclass = true) const
    {
        if (name.c_str()[0] == 0)
            return getHandlesByType(result, type, subclass);

        std::lock_guard<std::recursive_mutex> lck(_mtx);
        UnorderedHandleSet hs = nodeIndex.getHandleSet(type, name.c_str(), subclass);
        return std::copy(hs.begin(), hs.end(), result);
    }

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
    UnorderedHandleSet getHandlesByNames(const char**, Type*, bool*, Arity,
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
    UnorderedHandleSet getHandlesByTypes(Type* types, bool* subclasses, Arity arity,
                              Type type = ATOM, bool subclass = true) const
    { return getHandlesByNames((const char**) NULL, types, subclasses, arity, type, subclass); }

    /**
     * Returns the set of atoms within the given importance range.
     *
     * @param Importance range lower bound (inclusive).
     * @param Importance range upper bound (inclusive).
     * @return The set of atoms within the given importance range.
     */
    UnorderedHandleSet getHandlesByAV(AttentionValue::sti_t lowerBound,
                              AttentionValue::sti_t upperBound = AttentionValue::MAXSTI) const
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        return importanceIndex.getHandleSet(this, lowerBound, upperBound);
    }

    /**
     * Updates the importance index for the given atom. According to the
     * new importance of the atom, it may change importance bins.
     *
     * @param The atom whose importance index will be updated.
     * @param The old importance bin where the atom originally was.
     */
    void updateImportanceIndex(AtomPtr a, int bin)
    {
        std::lock_guard<std::recursive_mutex> lck(_mtx);
        importanceIndex.updateImportance(a, bin);
    }

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
    Handle add(AtomPtr) throw (RuntimeException);

    /**
     * Return true if the atom table holds this handle, else return false.
     */
    bool holds(Handle& h) const {
        return (NULL != h) and h->getAtomTable() == this;
    }

    /** Get Node object already in the AtomTable.
     *
     * @param h Handle of the node to retrieve.
     * @return pointer to Node object, NULL if no atom within this AtomTable is
     * associated with handle or if the atom is a link.
     */
    inline NodePtr getNode(Handle& h) const {
        h = getHandle(h); // force resolution of uuid into atom pointer.
        return NodeCast(h);
    }

    /** Get Link object already in the AtomTable.
     *
     * @param h Handle of the link to retrieve.
     * @return pointer to Link object, NULL if no atom within this AtomTable is
     * associated with handle or if the atom is a node.
     */
    inline LinkPtr getLink(Handle& h) const {
        h = getHandle(h); // force resolution of uuid into atom pointer.
        return LinkCast(h);
    }

    /**
     * Extracts atoms from the table. Table will not contain the
     * extracted atoms anymore.
     *
     * Note that if the recursive flag is set to false, and the atom
     * appears in the incoming set of some other atom, then extraction
     * will fail.  Thus, it is generally recommended that extraction
     * be recursive, unless you can guarentee that the atom is not in
     * someone else's outgoing set.
     *
     * @param handle The atom to be extracted.
     * @param recursive Recursive-removal flag; if set, the links in the
     *        incoming set will also be extracted.
     * @return A set of the extracted atoms.
     */
    AtomPtrSet extract(Handle& handle, bool recursive = true);

    /**
     * Return a random atom in the AtomTable.
     */
    Handle getRandom(RandGen* rng) const;

    AtomSignal& addAtomSignal() { return _addAtomSignal; }
    AtomPtrSignal& removeAtomSignal() { return _removeAtomSignal; }

    /** Provide ability for others to find out about AV changes */
    AVCHSigl& AVChangedSignal() { return _AVChangedSignal; }

    /** Provide ability for others to find out about TV changes */
    TVCHSigl& TVChangedSignal() { return _TVChangedSignal; }
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_ATOMTABLE_H
