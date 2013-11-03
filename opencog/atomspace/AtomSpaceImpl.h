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
#include <set>
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

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class AtomSpaceImpl;

/** 
 * \warning The AtomSpaceImpl class contains methods that are only to be called by
 * AtomSpace requests that are running within the AtomSpaceAsync event loop.
 */
class AtomSpaceImpl
{
    friend class SavingLoading;
    friend class SaveRequest;

public:
    AtomSpaceImpl(void);
    ~AtomSpaceImpl();

    AttentionBank& getAttentionBank() {return bank; }

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

    /**
     * Return the number of atoms contained in the space.
     */
    int getSize() const { return atomTable.getSize(); }

    /**
     * Prints atoms of this AtomTable to the given output stream.
     * @param output  the output stream where the atoms will be printed.
     * @param type  the type of atoms that should be printed.
     * @param subclass  if true, matches all atoms whose type is
     *              subclass of the given type. If false, matches
     *              only atoms of the exact type.
     */
    void print(std::ostream& output = std::cout,
               Type type = ATOM, bool subclass = true) const;

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

    /**
     * Retrieve from the Atom Table the Handle of a given node
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    Handle getHandle(Type t, const std::string& str) const
        { return atomTable.getHandle(t, str); }

    /**
     * Retrieve from the Atom Table the Handle of a given link
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    Handle getHandle(Type t, const HandleSeq& outgoing) const
        { return atomTable.getHandle(t, outgoing); }

    /** Retrieve the name of a given Handle */
    const std::string& getName(Handle h) const
    {
        static std::string emptyName;
        NodePtr nnn(NodeCast(h));
        if (nnn) return nnn->getName();
        return emptyName;
    }

    /** Retrieve the outgoing set of a given link */
    const HandleSeq getOutgoing(Handle h) const
    {
        static HandleSeq hs;
        LinkPtr link(LinkCast(h));
        if (link) return link->getOutgoingSet();
        return hs;
    }

    /** Retrieve a single Handle from the outgoing set of a given link */
    Handle getOutgoing(Handle h, int idx) const
    {
        LinkPtr link(LinkCast(h));
        if (link) return link->getOutgoingAtom(idx);
        return Handle::UNDEFINED;
    }

    /** Retrieve the arity of a given link */
    size_t getArity(Handle h) const
    {
        LinkPtr link(LinkCast(h));
        if (link) return link->getArity();
        return 0;
    }

    /** Retrieve the type of a given Handle */
    Type getType(Handle h) const
    {
        if (h) return h->getType();
        else return NOTYPE;
    }

    /** Return whether s is the source handle in a link l
     * @note Only ORDERED_LINKs have a source handle.
     */ 
    bool isSource(Handle source, Handle link) const
    {
        LinkPtr l(LinkCast(link));
        if (l) return l->isSource(source);
        return false;
    }

    /** Retrieve the incoming set of a given atom */
    HandleSeq getIncoming(Handle);

    /** Convenience functions... */
    bool isNode(Handle h) const
        { return classserver().isA(getType(h), NODE); }

    bool isLink(Handle h) const
        { return classserver().isA(getType(h), LINK); }

    /** Retrieve the TruthValue of a given Handle */
    TruthValuePtr getTV(Handle, VersionHandle = NULL_VERSION_HANDLE) const;

    /** Change the TruthValue of a given Handle
     * @return whether TV was successfully set
     */
    bool setTV(Handle, TruthValuePtr, VersionHandle = NULL_VERSION_HANDLE);

    /** Change the primary TV's mean of a given Handle */
    void setMean(Handle, float mean) throw (InvalidParamException);

    /** Retrieve the doubly normalised Short-Term Importance between -1..1
     * for a given AttentionValue. STI above and below threshold
     * normalised separately and linearly.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to -1..1? Outside this
     * range can be return if average=true
     * @return normalised STI between -1..1
     */
    float getNormalisedSTI(AttentionValuePtr avh, bool average=true, bool clip=false) const;

    /** Retrieve the linearly normalised Short-Term Importance between 0..1
     * for a given AttentionValue.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to 0..1? Outside this
     * range can be return if average=true
     * @return normalised STI between 0..1
     */
    float getNormalisedZeroToOneSTI(AttentionValuePtr avh, bool average=true, bool clip=false) const;

    /** Retrieve the AttentionValue of a given Handle */
    AttentionValuePtr getAV(Handle h) const {
        return h->getAttentionValue();
    }

    /** Change the AttentionValue of a given Handle */
    void setAV(Handle h, AttentionValuePtr av) {
        h->setAttentionValue(av);
    }

    /** Change the Short-Term Importance of a given Handle */
    void setSTI(Handle h, AttentionValue::sti_t stiValue) {
        /* Make a copy */
        AttentionValuePtr old_av = h->getAttentionValue();
        AttentionValuePtr new_av = createAV(
            stiValue,
            old_av->getLTI(),
            old_av->getVLTI());
        h->setAttentionValue(new_av);
    }

    /** Change the Long-term Importance of a given Handle */
    void setLTI(Handle h, AttentionValue::lti_t ltiValue) {
        AttentionValuePtr old_av = h->getAttentionValue();
        AttentionValuePtr new_av = createAV(
            old_av->getSTI(),
            ltiValue,
            old_av->getVLTI());
        h->setAttentionValue(new_av);
    }

    /** Increase the Very-Long-Term Importance of a given Handle by 1 */
    void incVLTI(Handle h) {
        AttentionValuePtr old_av = h->getAttentionValue();
        AttentionValuePtr new_av = createAV(
            old_av->getSTI(),
            old_av->getLTI(),
            old_av->getVLTI() + 1);
        h->setAttentionValue(new_av);
    }

    /** Decrease the Very-Long-Term Importance of a given Handle by 1 */
    void decVLTI(Handle h) {
        AttentionValuePtr old_av = h->getAttentionValue();
        //we only want to decrement the vlti if it's not already disposable.
        if (old_av->getVLTI() == AttentionValue::DISPOSABLE) return;
        AttentionValuePtr new_av = createAV(
            old_av->getSTI(),
            old_av->getLTI(),
            old_av->getVLTI() - 1);
        h->setAttentionValue(new_av);
    }

    /** Retrieve the Short-Term Importance of a given Handle */
    AttentionValue::sti_t getSTI(Handle h) const {
        return h->getAttentionValue()->getSTI();
    }

    /** Retrieve the Long-term Importance of a given Handle */
    AttentionValue::lti_t getLTI(Handle h) const {
        return h->getAttentionValue()->getLTI();
    }

    /** Retrieve the Very-Long-Term Importance of a given Handle */
    AttentionValue::vlti_t getVLTI(Handle h) const {
        return h->getAttentionValue()->getVLTI();
    }

    bool isValidHandle(Handle h) const {
        return atomTable.holds(h);
    }

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
     * Gets a set of handles that matches with the given arguments.
     *
     * @param result An output iterator.
     * @param type the type of the atoms to be searched
     * @param name the name of the atoms to be searched.
     *        For searching only links, use "" or a search by type.
     * @param subclass if sub types of the given type are accepted in this search
     * @param vh only atoms that contains versioned TVs with
     *        the given VersionHandle are returned. If NULL_VERSION_HANDLE is given,
     *        it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument. Example of a
     * call to this method, which would return all entries in AtomSpace:
     * @code
     *      std::list<Handle> ret;
     *      atomSpaceImpl.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 const std::string& name,
                 bool subclass = true) const
    {
        return atomTable.getHandlesByName(result, name, type, subclass);
    }

    /** Same as above, but includes check for VH */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const std::string& name,
                 Type type,
                 bool subclass,
                 VersionHandle vh) const
    {
        return atomTable.getHandlesByNameVH(result, name, type, subclass, vh);
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally).
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given
     *        VersionHandle are returned.  If NULL_VERSION_HANDLE is
     *        given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * @note  The matched entries are appended to a container whose
     *        OutputIterator is passed as the first argument.
     *        Example of call to this method, which would return all
     *        entries in AtomSpace:
     * @code
     *        std::list<Handle> ret;
     *        atomSpaceImpl.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 bool subclass = false) const 
    {
        return atomTable.getHandlesByType(result, type, subclass);
    }

    /* Same as above */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 bool subclass,
                 VersionHandle vh) const
    {
        return atomTable.getHandlesByTypeVH(result, type, subclass, vh);
    }

    /**
     * Returns the set of atoms of a given type which have atoms of a
     * given target type in their outgoing set (subclasses optionally).
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param targetType The desired target type.
     * @param subclass Whether type subclasses should be considered.
     * @param targetSubclass Whether target type subclasses should be
     *        considered.
     * @param vh only atoms that contains versioned TVs with the given
     *        VersionHandle are returned.  If NULL_VERSION_HANDLE is
     *        given, it does not restrict the result.
     * @param targetVh only atoms whose target contains versioned TVs
     *        with the given VersionHandle are returned.  If
     *        NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of a given type and target type (subclasses
     *        optionally).
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
                 Type type,
                 Type targetType,
                 bool subclass,
                 bool targetSubclass,
                 VersionHandle vh = NULL_VERSION_HANDLE,
                 VersionHandle targetVh = NULL_VERSION_HANDLE) const
    {
        return atomTable.getHandlesByTargetTypeVH(result,
               type, targetType, subclass, targetSubclass, vh, targetVh);
    }

    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     *
     * @param result An output iterator.
     * @param handle The handle that must be in the outgoing set of the atom.
     * @param type The type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the given handle in
     * their outgoing set.
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
                 Handle handle,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        return atomTable.getIncomingSetByTypeVH(result, handle, type, subclass, vh);
    }

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
     * Returns the set of atoms whose outgoing set contains at least one
     * atom with the given name and type (atom type and subclasses
     * optionally).
     *
     * @param result An output iterator.
     * @param targetName The name of the atom in the outgoing set of the searched
     * atoms.
     * @param targetType The type of the atom in the outgoing set of the searched
     * atoms.
     * @param type type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh return only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type and name whose outgoing
     * set contains at least one atom of the given type and name.
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
                 const std::string& targetName,
                 Type targetType,
                 Type type,
                 bool subclass) const
    {
        return atomTable.getIncomingSetByName(result,
                 targetName, targetType, type, subclass);
    }

    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const std::string& targetName,
                 Type targetType,
                 Type type,
                 bool subclass,
                 VersionHandle vh,
                 VersionHandle targetVh) const
    {
        return atomTable.getIncomingSetByNameVH(result,
                 targetName, targetType, type, subclass, vh, targetVh);
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
     * Gets a set of handles in the Attentional Focus that matches with the given type
     * (subclasses optionally).
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param vh returns only atoms that contains versioned TVs with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
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
    getHandleSetInAttentionalFocus(OutputIterator result,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        //return getHandleSet(result, type, subclass, InAttentionalFocus(), vh);
        STIAboveThreshold s(bank.getAttentionalFocusBoundary());
        return atomTable.getHandlesByTypePredVH(result, type, subclass, &s, vh);
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

        getHandleSet(back_inserter(hs), type, subclass, vh);
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
        getHandleSet(back_inserter(handle_set), atype, subclass);

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

    size_t Nodes(VersionHandle = NULL_VERSION_HANDLE) const;
    size_t Links(VersionHandle = NULL_VERSION_HANDLE) const;

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
