/*
 * opencog/atomspace/AtomSpaceImpl.h
 *
 * Copyright (C) 2010-2011 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@opencog.org>
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

#ifndef _OPENCOG_ATOMSPACE_IMPL_H
#define _OPENCOG_ATOMSPACE_IMPL_H

#include <algorithm>
#include <list>
#include <vector>

#include <boost/signal.hpp>

#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/AttentionBank.h>
#include <opencog/atomspace/BackingStore.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/recent_val.h>

class AtomSpaceImplUTest;

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/** 
 * \warning The AtomSpaceImpl class contains methods that are only to be called by
 * AtomSpace requests that are running within the AtomSpaceAsync event loop.
 */
class AtomSpaceImpl
{
    friend class SavingLoading;
    friend class SaveRequest;
    friend class ::AtomSpaceImplUTest;

public:
    AtomSpaceImpl(void);
    ~AtomSpaceImpl();

    AttentionBank& getAttentionBank() {return bank; }
    const AttentionBank& getAttentionBankconst() const {return bank; }

    /**
     * Register a provider of backing storage.
     */
    void registerBackingStore(BackingStore *);
    void unregisterBackingStore(BackingStore *);

    /**
     * Recursively store the atom to the backing store.
     * I.e. if the atom is a link, then store all of the atoms
     * in its outgoing set as well, recursively.
     */
    void storeAtom(Handle h) {
        if (backing_store) backing_store->storeAtom(h);
    }

    /**
     * Return the atom with the indicated handle. This method will
     * explicitly use the backing store to obtain an instance of the
     * atom. If an atom corresponding to the handle cannot be found,
     * then an undefined handle is returned. If the atom is found, 
     * then the corresponding atom is guaranteed to have been
     * instantiated in the atomspace.
     */
    Handle fetchAtom(Handle);

    /**
     * Use the backing store to load the entire incoming set of the atom.
     * If the flag is true, then the load is done recursively. 
     * This method queries the backing store to obtain all atoms that 
     * contain this one in their outgoing sets. All of these atoms are
     * then loaded into this atomtable/atomspace.
     */
    Handle fetchIncomingSet(Handle, bool);

    /**
     * @return a const reference to the AtomTable
     */
    const AtomTable& getAtomTable() const { return atomTable; }

    //helpers for GDB debugging, because trying to get std::cout is annoying
    void printGDB() const;
    void printTypeGDB(Type t) const;

    /** Add a new node to the Atom Table,
     * if the atom already exists then the old and the new truth value is merged
     *  @param t     Type of the node
     *  @param name  Name of the node
     *  @param tvn   Optional TruthValue of the node. If not provided, uses the
     *  DEFAULT_TV (see TruthValue.h)
     */
    Handle addNode(Type t, const std::string& name = "", TruthValuePtr tvn = TruthValue::DEFAULT_TV());

    /**
     * Add a new link to the Atom Table
     * If the atom already exists then the old and the new truth value
     * is merged
     * @param t         Type of the link
     * @param outgoing  a const reference to a HandleSeq containing
     *                  the outgoing set of the link
     * @param tvn       Optional TruthValue of the node. If not
     *                  provided, uses the DEFAULT_TV (see TruthValue.h)
     */
    Handle addLink(Type t, const HandleSeq& outgoing,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV());

    /**
     * Removes an atom from the atomspace
     *
     * @param h The Handle of the atom to be removed.
     * @param recursive Recursive-removal flag; the removal will
     *       fail if this flag is not set, and the atom has incoming
     *       links (that are in the atomspace).  Set to false only if
     *       you can guarantee that this atom does not appear in the
     *       outgoing set of any link in the atomspace.
     * @return True if the Atom for the given Handle was successfully
     *         removed. False, otherwise.
     */
    bool removeAtom(Handle h, bool recursive = true) {
        return 0 < atomTable.extract(h, recursive).size();
    }

    /** Retrieve the incoming set of a given atom */
    HandleSeq getIncoming(Handle);

    /**
     * Returns neighboring atoms, following links and returning their
     * target sets.
     * @param h Get neighbours for the atom this handle points to.
     * @param fanin Whether directional links point to this node should be
     * considered.
     * @param fanout Whether directional links point from this node to
     * another should be considered.
     * @param linkType Follow only these types of links.
     * @param subClasses Follow subtypes of linkType too.
     */
    HandleSeq getNeighbors(Handle h, bool fanin=true, bool fanout=true,
            Type linkType=LINK, bool subClasses=true) const;


    /**
     * Returns the set of atoms with the given target handles and types
     * (order is considered) in their outgoing sets, where the type and
     * subclasses of the atoms are optional.
     *
     * @param result An output iterator.
     * @param handles An array of handles to match the outgoing sets of the searched
     * atoms. This array can be empty (or each of its elements can be null), if
     * the handle value does not matter or if it does not apply to the
     * specific search.
     * Note that if this array is not empty, it must contain "arity" elements.
     * @param types An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms.
     * @param subclasses An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Note that if this
     * array is not null, it must contains "arity" elements.
     * @param arity The length of the outgoing set of the atoms being searched.
     * @param type The type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * @note The matched entries are appended to a container whose OutputIterator
     * is passed as the first argument. Example of call to this method, which
     * would return all entries in AtomSpace:
     * @code
     *     std::list<Handle> ret;
     *     atomSpaceImpl.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const HandleSeq& handles,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        UnorderedHandleSet hs = atomTable.getHandlesByOutgoing(handles,
                types, subclasses, arity, type, subclass, vh);
        return std::copy(hs.begin(), hs.end(), result);
    }

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param result An output iterator.
     * @param names An array of names to match the outgoing sets of the searched
     * atoms. This array (or each of its elements) can be null, if
     * the names do not matter or if do not apply to the specific search.
     * Note that if this array is not null, it must contain "arity" elements.
     * @param types An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. If array of names is not null,
     * this parameter *cannot* be null as well. Besides, if an element in a
     * specific position in the array of names is not null, the corresponding
     * type element in this array *cannot* be NOTYPE as well.
     * @param subclasses An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param arity The length of the outgoing set of the atoms being searched.
     * @param type The optional type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh return only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * @note  The matched entries are appended to a container whose
     *        OutputIterator is passed as the first argument.  Example
     *        of call to this method, which would return all entries in
     *        AtomSpace:
     * @code
     *        std::list<Handle> ret;
     *        atomSpaceImpl.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const char** names,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        UnorderedHandleSet hs = atomTable.getHandlesByNames(names, types, subclasses, arity, type, subclass, vh);
        return std::copy(hs.begin(), hs.end(), result);
    }

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param result An output iterator.
     * @param types An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. This parameter can be null (or any of
     * its elements can be NOTYPE), what means that the type doesnt matter.
     * Not that if this array is not null, it must contains "arity" elements.
     * @param subclasses An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param arity The length of the outgoing set of the atoms being searched.
     * @param type The optional type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh returns only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * @note  The matched entries are appended to a container whose
     *        OutputIterator is passed as the first argument.  Example
     *        of call to this method, which would return all entries in
     *        AtomSpace:
     * @code
     *        std::list<Handle> ret;
     *        atomSpaceImpl.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        UnorderedHandleSet hs = atomTable.getHandlesByTypes(types, subclasses, arity, type, subclass, vh);
        return std::copy(hs.begin(), hs.end(), result);
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally), sorted according to the given comparison
     * structure.
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param compare The comparison struct to use in the sort.
     * @param vh returns only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * @note  The matched entries are appended to a container whose
     *        OutputIterator is passed as the first argument.  Example
     *        of call to this method, which would return all entries in
     *        AtomSpace sorted by STI:
     * @code
     *        std::list<Handle> ret;
     *        AttentionValue::STISort stiSort;
     *        atomSpaceImpl.getHandleSet(back_inserter(ret), ATOM, true, stiSort);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getSortedHandleSet(OutputIterator result,
                 Type type,
                 bool subclass,
                 AtomComparator* compare,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        // get the handle set as a vector and sort it.
        std::vector<Handle> hs;

        atomTable.getHandlesByTypeVH(back_inserter(hs), type, subclass, vh);
        sort(hs.begin(), hs.end(), compareAtom<AtomComparator>(&atomTable, compare));

        // copy the vector and return the iterator.
        return std::copy(hs.begin(), hs.end(), result);
    }

    // Wrapper for comparing atoms from a HandleSeq
    template <typename Compare>
    struct compareAtom{
        Compare* c;
        const AtomTable* table;
        compareAtom(const AtomTable* _table, Compare* _c) : c(_c), table(_table) {}

        bool operator()(Handle h1, Handle h2) {
            return (*c)(h1, h2);
        }
    };

    template <typename Compare>
    struct filterAtom{
        Compare *c;
        const AtomTable* table;
        filterAtom(const AtomTable* _table, Compare *_c) : c(_c), table(_table) {}

        bool operator()(const Handle& h1) {
            return (*c)(h1);
        }
    };

    /* ----------------------------------------------------------- */
    /* The foreach routines offer an alternative interface
     * to the getHandleSet API.
     */
    /**
     * Invoke the callback on each handle of the given type.
     */
    template<class T>
    inline bool foreach_handle_of_type(Type atype,
                                       bool (T::*cb)(Handle), T *data,
                                       bool subclass = false) {
        std::list<Handle> handle_set;
        // The intended signatue is
        // getHandleSet(OutputIterator result, Type type, bool subclass)
        getAtomTable().getHandlesByType(back_inserter(handle_set), atype, subclass);

        // Loop over all handles in the handle set.
        std::list<Handle>::iterator i;
        for (i = handle_set.begin(); i != handle_set.end(); i++) {
            Handle h = *i;
            bool rc = (data->*cb)(h);
            if (rc) return rc;
        }
        return false;
    }

    template<class T>
    inline bool foreach_handle_of_type(const char * atypename,
                                       bool (T::*cb)(Handle), T *data,
                                       bool subclass = false) {
        Type atype = classserver().getType(atypename);
        return foreach_handle_of_type(atype, cb, data, subclass);
    }

    /* ----------------------------------------------------------- */

    /**
     * Decays STI of all atoms (one cycle of importance decay).
     * Deprecated, importance updating should be done by ImportanceUpdating
     * Agent. Still used by Embodiment.
     */
    void decayShortTermImportance() { atomTable.decayShortTermImportance(); }

    bool containsVersionedTV(Handle h, VersionHandle vh) const
        { return atomTable.containsVersionedTV(h, vh); }

    //! Clear the atomspace, remove all atoms
    void clear();

// ---- filter templates

    HandleSeq filter(AtomPredicate* pred, VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeq result;
        atomTable.getHandlesByPredVH(back_inserter(result), pred, vh);
        return result;
    }

    template<typename OutputIterator>
    OutputIterator filter(OutputIterator it, AtomPredicate* pred, VersionHandle vh = NULL_VERSION_HANDLE)
        { return atomTable.getHandlesByPredVH(it, pred, vh); }

    /**
     * Filter handles from a sequence according to the given criterion.
     *
     * @param begin iterator for the sequence
     * @param end iterator for the sequence
     * @param struct or function embodying the criterion
     * @return The handles in the sequence that match the criterion.
     */
    template<typename InputIterator>
    HandleSeq filter(InputIterator begin, InputIterator end, AtomPredicate* compare) const {
        HandleSeq result;
        for (; begin != end; begin++)
            if (filterAtom<AtomPredicate>(&atomTable,compare)(*begin))
                result.push_back(*begin);

        return result;
    }

    template<typename InputIterator, typename OutputIterator>
    OutputIterator filter(InputIterator begin, InputIterator end,
            OutputIterator it, AtomPredicate* compare) const {
        for (; begin != end; begin++)
            if (filterAtom<AtomPredicate>(&atomTable,compare)(*begin))
                * it++ = *begin;
        return it;
    }

    template<typename InputIterator>
    HandleSeq filter_InAttentionalFocus(InputIterator begin, InputIterator end) const {
        STIAboveThreshold stiAbove(bank.getAttentionalFocusBoundary());
        return filter(begin, end, &stiAbove);
    }

    struct STIAboveThreshold : public AtomPredicate {
        STIAboveThreshold(const AttentionValue::sti_t t) : threshold (t) {}

        virtual bool test(AtomPtr a) {
            return a->getAttentionValue()->getSTI() > threshold;
        }
        AttentionValue::sti_t threshold;
    };

    struct LTIAboveThreshold : public AtomPredicate {
        LTIAboveThreshold(const AttentionValue::lti_t t) : threshold (t) {}

        virtual bool test(AtomPtr a) {
            return a->getAttentionValue()->getLTI() > threshold;
        }
        AttentionValue::lti_t threshold;
    };

private:

    AtomTable atomTable;
    AttentionBank bank;

    /**
     * Used to fetch atoms from disk.
     */
    BackingStore *backing_store;

    /**
     * signal connections used to keep track of atom removal in the AtomTable
     */
    boost::signals::connection removedAtomConnection; 
    boost::signals::connection addedAtomConnection; 

    /** Handler for the 'atom removed' signal */
    void atomRemoved(AtomPtr);

    /** Handler for the 'atom added' signal */
    void atomAdded(Handle);

public:
    // pass on the signals from the Atom Table
    AtomSignal& addAtomSignal() { return atomTable._addAtomSignal; }
    AtomPtrSignal& removeAtomSignal() { return atomTable._removeAtomSignal; }
    TVCHSigl& TVChangedSignal() { return atomTable._TVChangedSignal; }
    AVCHSigl& AVChangedSignal() { return atomTable._AVChangedSignal; }

    /**
     * Overrides and declares copy constructor and equals operator as private 
     * for avoiding large object copying by mistake.
     */
    AtomSpaceImpl& operator=(const AtomSpaceImpl&);
    AtomSpaceImpl(const AtomSpaceImpl&);
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_IMPL_H
