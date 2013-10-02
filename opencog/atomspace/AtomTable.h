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
#include <vector>
#include <unordered_set>

#include <boost/signal.hpp>

#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CompositeTruthValue.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/FixedIntegerIndex.h>
#include <opencog/atomspace/ImportanceIndex.h>
#include <opencog/atomspace/IncomingIndex.h>
#include <opencog/atomspace/Intersect.h>
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
/** \addtogroup grp_atomspace
 *  @{
 */

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
    friend class ::AtomTableUTest;

private:

    int size;

    //!@{
    //! Index for quick retreival of certain kinds of atoms.
    TypeIndex typeIndex;
    NodeIndex nodeIndex;
    LinkIndex linkIndex;
    IncomingIndex incomingIndex;
    ImportanceIndex importanceIndex;
    TargetTypeIndex targetTypeIndex;
    PredicateIndex predicateIndex;
	//!@}
	
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

    /** \warning this should only be called by decayShortTermImportance */
    void clearIndexesAndRemoveAtoms(const UnorderedHandleSet&);

    /**
     * Extracts atoms from the table. Table will not contain the
     * extracted atoms anymore, but they will not be deleted.
     * Instead, they are returned by this method.
     *
     * Note: The caller is responsible for releasing the memory of
     * both the returned list and the refered Atoms inside it.
     *
     * @param handle The atom to be extracted.
     * @param recursive Recursive-removal flag; if set, the links in the
     *        incoming set will also be extracted.
     * @return A list with the Handles of all extracted Atoms.
     */
    UnorderedHandleSet extract(Handle handle, bool recursive = false);

    /**
     * Removes the previously extracted Handles (using the extract
     * method) from this table.
     * @param The list of the Handles previously extracted.
     *
     * \note This method also frees the memory of the Atom objects!
     */
    void removeExtractedHandles(const UnorderedHandleSet&);

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
    int getSize() const;

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
    template <typename OutputIterator> OutputIterator
    getHandlesByGPN(OutputIterator result,
                    const std::string& gpnNodeName,
                    VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        Handle gpnHandle = getHandle(gpnNodeName, GROUNDED_PREDICATE_NODE);
        return getHandlesByGPN(result, gpnHandle, vh);
    }

    /**
     * Returns a list of handles that matches the GroundedPredicateNode
     * with the given Handle.
     * @param the Handle of the predicate node.
     * @param VersionHandle for filtering the resulting atoms by
     *       context. NULL_VERSION_HANDLE indicates no filtering
     **/
    template <typename OutputIterator> OutputIterator
    getHandlesByGPN(OutputIterator result,
                    Handle h,
                    VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        const UnorderedHandleSet& hs = predicateIndex.findHandlesByGPN(h);
        return std::copy_if(hs.begin(), hs.end(), result,
                 [&](Handle h)->bool{ return containsVersionedTV(h, vh); });
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
    Handle getHandle(const std::string&, Type) const;
    Handle getHandle(NodePtr) const;

    Handle getHandle(Type, const HandleSeq&) const;
    Handle getHandle(LinkPtr) const;
    Handle getHandle(AtomPtr) const;

protected:
    /* Some basic predicates */
    static bool isDefined(Handle h) { return h != Handle::UNDEFINED; }
    bool isType(Handle h, Type t, bool subclass) const
    {
        Type at = getAtom(h)->getType();
        if (not subclass) return t == at;
        return classserver().isA(at, t);
    }
    bool containsVersionedTV(Handle h, VersionHandle vh) const
    {
        if (isNullVersionHandle(vh)) return true;
        const TruthValue& tv = getAtom(h)->getTruthValue();
        return (not tv.isNullTv())
               and (tv.getType() == COMPOSITE_TRUTH_VALUE)
               and (not (((const CompositeTruthValue&) tv).getVersionedTV(vh).isNullTv()));
    }
    bool hasNullName(Handle h) const
    {
        AtomPtr a(getAtom(h));
        if (LinkCast(a)) return true;
        if (NodeCast(a)->getName().c_str()[0] == 0) return true;
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
        return std::copy_if(typeIndex.begin(type, subclass),
                            typeIndex.end(),
                            result,
                            isDefined);
    }

    template <typename OutputIterator> OutputIterator
    getHandlesByTypeVH(OutputIterator result,
                       Type type,
                       bool subclass,
                       VersionHandle vh) const
    {
        return std::copy_if(typeIndex.begin(type, subclass),
                            typeIndex.end(),
                            result,
             [&](Handle h)->bool{
                  return isDefined(h) and containsVersionedTV(h, vh);
             });
    }

    /** Calls function 'func' on all atoms */
    template <typename Function> void
    foreachHandleByType(Function func,
                        Type type,
                        bool subclass = false) const
    {
        std::for_each(typeIndex.begin(type, subclass),
                      typeIndex.end(),
             [&](Handle h)->void { 
                  if (not isDefined(h)) return;
                  (func)(h);
             });
    }

    template <typename Function> void
    foreachHandleByTypeVH(Function func,
                        Type type,
                        bool subclass,
                        VersionHandle vh) const
    {
        std::for_each(typeIndex.begin(type, subclass),
                      typeIndex.end(),
             [&](Handle h)->void { 
                  if (not isDefined(h)) return;
                  if (not containsVersionedTV(h, vh)) return;
                  (func)(h);
             });
    }

    /**
     * Returns all atoms satisfying the predicate
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByTypePredVH(OutputIterator result,
                           Type type,
                           bool subclass,
                           AtomPredicate* pred,
                           VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        return std::copy_if(typeIndex.begin(type, subclass),
                            typeIndex.end(),
                            result,
             [&](Handle h)->bool { 
                  return isDefined(h)
                      and (*pred)(*getAtom(h))
                      and containsVersionedTV(h, vh);
             });
    }

    template <typename OutputIterator> OutputIterator
    getHandlesByPredVH(OutputIterator result,
                       AtomPredicate* pred,
                       VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        return getHandlesByTypePredVH(result, ATOM, true, pred, vh);
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
    getHandlesByTargetTypeVH(OutputIterator result,
                             Type type,
                             Type targetType,
                             bool subclass,
                             bool targetSubclass,
                             VersionHandle vh = NULL_VERSION_HANDLE,
                             VersionHandle targetVh = NULL_VERSION_HANDLE) const
    {
        return std::copy_if(targetTypeIndex.begin(targetType, targetSubclass),
                            targetTypeIndex.end(),
                            result,
             [&](Handle h)->bool{
                 return isDefined(h)
                    and isType(h, type, subclass)
                    and containsVersionedTV(h, vh)
                    and containsVersionedTV(h, targetVh);
             });
    }

    /**
     * Return the incoming set associated with handle h.
     */
    const UnorderedHandleSet& getIncomingSet(Handle h) const
        { return incomingIndex.getIncomingSet(h); }

    template <typename OutputIterator> OutputIterator
    getIncomingSet(OutputIterator result,
                   Handle h) const
    {
        return std::copy(incomingIndex.begin(h),
                         incomingIndex.end(),
                         result);
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
    template <typename OutputIterator> OutputIterator
    getIncomingSetByType(OutputIterator result,
                         Handle h,
                         Type type,
                         bool subclass = false) const
    {
        return std::copy_if(incomingIndex.begin(h),
                            incomingIndex.end(),
                            result,
             [&](Handle h)->bool{
                     return isDefined(h)
                        and isType(h, type, subclass); });
    }

    template <typename OutputIterator> OutputIterator
    getIncomingSetByTypeVH(OutputIterator result,
                           Handle h,
                           Type type,
                           bool subclass,
                           VersionHandle vh) const
    {
        return std::copy_if(incomingIndex.begin(h),
                            incomingIndex.end(),
                            result,
             [&](Handle h)->bool{
                   return isDefined(h)
                      and isType(h, type, subclass)
                      and containsVersionedTV(h, vh); });
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
    template <typename OutputIterator> OutputIterator
    getIncomingSetByName(OutputIterator result,
                         const std::string& targetName,
                         Type targetType,
                         Type type = ATOM,
                         bool subclass = true) const
    {
        // Gets the exact atom with the given name and type, in any AtomTable.
        Handle targh = getHandle(targetName, targetType);
        return getIncomingSetByType(result, targh, type, subclass);
    }

    template <typename OutputIterator> OutputIterator
    getIncomingSetByNameVH(OutputIterator result,
                           const std::string& targetName,
                           Type targetType,
                           Type type,
                           bool subclass,
                           VersionHandle vh,
                           VersionHandle targetVh) const
    {
        // Gets the exact atom with the given name and type, in any AtomTable.
        Handle targh = getHandle(targetName, targetType);
        // XXX TODO what the heck with targetVH ?? Are we supposed to
        // check if targh above has it ?? And if not, I guess return
        // empty set ... Who needs this stuff, anyway?
        return getIncomingSetByTypeVH(result, targh, type, subclass, vh);
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
                              bool subclass = true,
                              VersionHandle vh = NULL_VERSION_HANDLE) const;

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

        UnorderedHandleSet hs = nodeIndex.getHandleSet(type, name.c_str(), subclass);
        return std::copy(hs.begin(), hs.end(), result);
    }

    template <typename OutputIterator> OutputIterator
    getHandlesByNameVH(OutputIterator result,
                       const std::string& name,
                       Type type,
                       bool subclass,
                       VersionHandle vh) const
    {
        if (name.c_str()[0] == 0)
            return getHandlesByTypeVH(result, type, subclass, vh);

        UnorderedHandleSet hs = nodeIndex.getHandleSet(type, name.c_str(), subclass);
        return std::copy_if(hs.begin(), hs.end(), result,
             [&](Handle h)->bool{ return containsVersionedTV(h, vh); });
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
                              Type type = ATOM, bool subclass = true,
                              VersionHandle vh = NULL_VERSION_HANDLE) const
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
                              Type type = ATOM, bool subclass = true,
                              VersionHandle vh = NULL_VERSION_HANDLE) const
    { return getHandlesByNames((const char**) NULL, types, subclasses, arity, type, subclass, vh); }

    /**
     * Returns the set of atoms within the given importance range.
     *
     * @param Importance range lower bound (inclusive).
     * @param Importance range upper bound (inclusive).
     * @return The set of atoms within the given importance range.
     */
    UnorderedHandleSet getHandleSet(AttentionValue::sti_t lowerBound,
                              AttentionValue::sti_t upperBound = 32767) const
        { return importanceIndex.getHandleSet(this, lowerBound, upperBound); }

    /**
     * Decays importance of all atoms in the table, reindexing
     * importanceIndex accordingly and extracting the atoms that fall
     * below the "LOWER_STI_VALUE" threshold.
     * @return the list of the handles that should be removed.
     */
    UnorderedHandleSet decayShortTermImportance()
        { return importanceIndex.decayShortTermImportance(this); }


    /**
     * Updates the importance index for the given atom. According to the
     * new importance of the atom, it may change importance bins.
     *
     * @param The atom whose importance index will be updated.
     * @param The old importance bin where the atom originally was.
     */
    void updateImportanceIndex(AtomPtr a, int bin)
    {
        importanceIndex.updateImportance(a, bin);
    }

    /**
     * Merge the existing atom with the given handle with the given truth value.
     * If the handle is valid, emits atom merged signal.
     * @param h     Handle of the Atom to be merged
     * @param tvn   TruthValue to be merged to current atom's truth value.  
     */
    void merge(Handle, const TruthValue&);

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
    bool holds(Handle h) const { return (NULL != getAtom(h)); }

    /** Get Atom object already in the AtomTable.
     *
     * @param h Handle of the atom to retrieve.
     * @return pointer to Atom object, NULL if no atom within this AtomTable is
     * associated with handle.
     */
    inline AtomPtr getAtom(Handle h) const {
        if (h == Handle::UNDEFINED) return NULL;
        AtomPtr atom(TLB::getAtom(h));
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
    inline NodePtr getNode(Handle h) const {
        return NodeCast(getAtom(h));
    }

    /** Get Link object already in the AtomTable.
     *
     * @param h Handle of the link to retrieve.
     * @return pointer to Link object, NULL if no atom within this AtomTable is
     * associated with handle or if the atom is a node.
     */
    inline LinkPtr getLink(Handle h) const {
        return LinkCast(getAtom(h));
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
     */
    Handle getRandom(RandGen* rng) const;
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_ATOMTABLE_H
