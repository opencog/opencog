/*
 * opencog/atomspace/AtomSpace.h
 *
 * Copyright (C) 2008-2011 OpenCog Foundation
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

#ifndef _OPENCOG_ATOMSPACE_H
#define _OPENCOG_ATOMSPACE_H

#include <algorithm>
#include <list>
#include <set>
#include <vector>

#include <opencog/atomspace/AtomSpaceImpl.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/util/exceptions.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */
/**
 * The AtomSpace class exposes the public API of the OpenCog AtomSpace
 *
 * @code
 *  // Create an atomspace
 *  AtomSpace atomspace;
 * @endcode
 *
 * If one were to add atoms to via AtomTable or AtomSpace, they would both be
 * in the same "AtomSpace".
 */
class AtomSpace
{
    friend class SavingLoading;
    friend class ::AtomTableUTest;
    friend class SaveRequest;

    /**
     * Overrides and declares equals operator as private
     * for avoiding large object copying by mistake.
     */
    AtomSpace& operator=(const AtomSpace&);

    /**
     * The AtomSpace class is essentially just be a wrapper of the AtomTable
     */
    mutable AtomSpaceImpl* _atomSpaceImpl;
    bool _ownsAtomSpaceImpl;

public:
    AtomSpace(void);
    /**
     * Create an atomspace that will send requests to an existing AtomSpace
     * event-loop.
     */
    AtomSpace(AtomSpaceImpl* a);

    AtomSpace(const AtomSpace&);
    ~AtomSpace();

    inline AttentionBank& getAttentionBank()
    { return _atomSpaceImpl->bank; }

    inline const AttentionBank& getAttentionBankconst() const
    { return _atomSpaceImpl->bank; }

    inline AtomSpaceImpl& getImpl()
    { return *_atomSpaceImpl; }

    inline const AtomSpaceImpl& getImplconst() const
    { return *_atomSpaceImpl; }

    inline const AtomTable& getAtomTable() const
    { return getImplconst().atomTable; }

    /**
     * Return the number of atoms contained in the space.
     */
    inline int getSize() const { return getAtomTable().getSize(); }
    inline int getNumNodes() const { return getAtomTable().getNumNodes(); }
    inline int getNumLinks() const { return getAtomTable().getNumLinks(); }

    /**
     * Prints atoms of this AtomSpace to the given output stream.
     * @param output  the output stream where the atoms will be printed.
     * @param type  the type of atoms that should be printed.
     * @param subclass  if true, matches all atoms whose type is
     *              subclass of the given type. If false, matches
     *              only atoms of the exact type.
     */
    void print(std::ostream& output = std::cout,
               Type type = ATOM, bool subclass = true) const {
        getAtomTable().print(output, type, subclass);
    }

    /**
     * Add a new node to the Atom Table.  If the atom already exists
     * then the old and the new truth value is merged.
     *
     * \param t     Type of the node
     * \param name  Name of the node
     * \param tvn   Optional TruthValue of the node. If not provided,
     *              uses the DEFAULT_TV (see TruthValue.h)
     */
    inline Handle addNode(Type t, const std::string& name = "",
                          TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        return getImpl().addNode(t, name, tvn);
    }

    /**
     * Add a new node to the AtomTable. A random 16-character string
     * will be appended to the provided name.
     *
     * @todo: Later on, the names can include server/time info to decrease
     * the probability of collisions and be more informative.
     **/
    Handle addPrefixedNode(Type t, const std::string& prefix = "",
                       TruthValuePtr tvn = TruthValue::DEFAULT_TV());

    /**
     * Add a new link to the Atom Table.
     * If the atom already exists, then the old and the new truth value
     * is merged.
     *
     * @param t         Type of the link
     * @param outgoing  a const reference to a HandleSeq containing
     *                  the outgoing set of the link
     * @param tvn       Optional TruthValue of the node. If not
     *                  provided, uses the DEFAULT_TV (see TruthValue.h)
     */
    inline Handle addLink(Type t, const HandleSeq& outgoing,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        return getImpl().addLink(t,outgoing,tvn);
    }

    inline Handle addLink(Type t, Handle h,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(h);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc, Handle hd,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc, Handle hd, Handle he,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        oset.push_back(hf);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf, Handle hg,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        oset.push_back(hf);
        oset.push_back(hg);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf, Handle hg,
                          Handle hh,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        oset.push_back(hf);
        oset.push_back(hg);
        oset.push_back(hh);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf, Handle hg,
                          Handle hh, Handle hi,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        oset.push_back(hf);
        oset.push_back(hg);
        oset.push_back(hh);
        oset.push_back(hi);
        return addLink(t, oset, tvn);
    }

    /**
     * Make sure all atom writes have completed, before returning.
     * This only has an effect when the atomspace is backed by some
     * sort of storage, or is sending atoms to some remote location
     * asynchronously. This simply guarantees that the asynch
     * operations have completed.
     * NB: at this time, we don't distinguish barrier and flush.
     */
    void barrier(void) { getImpl().barrier(); }

    /**
     * Purge an atom from the atomspace.  This only removes the atom
     * from the AtomSpace; it may still remain in persistent storage.
     * To also delete from persistant storage, use the removeAtom()
     * method.  The atom remains valid as long as there are Handles
     * or AtomPptr's that reference it; it is deleted only when the
     * last reference goes away.
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
    bool purgeAtom(Handle h, bool recursive = true) {
        return getImpl().purgeAtom(h, recursive);
    }

    /**
     * Removes an atom from the atomspace, and any attached storage.
     * The atom remains valid as long as there are Handles or AtomPtr's
     * that reference it; it is deleted only when the last reference
     * goes away.
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
        return getImpl().removeAtom(h, recursive);
    }

    /**
     * Retrieve from the Atom Table the Handle of a given node
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    Handle getHandle(Type t, const std::string& str) {
        return getImpl().getNode(t, str);
    }

    /**
     * Retrieve from the Atom Table the Handle of a given link
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    Handle getHandle(Type t, const HandleSeq& outgoing) {
        return getImpl().getLink(t, outgoing);
    }

    /** Get the atom referred to by Handle h represented as a string. */
    std::string atomAsString(Handle h, bool terse = true) const {
        if (terse) return h->toShortString();
        return h->toString();
    }

    /** Retrieve the name of a given Handle */
    const std::string& getName(Handle h) const {
        static std::string noname;
        NodePtr nnn(NodeCast(h));
        if (nnn) return nnn->getName();
        return noname;
    }

    /** Change the Short-Term Importance of a given Handle */
    void setSTI(Handle h, AttentionValue::sti_t stiValue) const {
        h->setSTI(stiValue);
    }

    /** Change the Long-term Importance of a given Handle */
    void setLTI(Handle h, AttentionValue::lti_t ltiValue) const {
        h->setLTI(ltiValue);
    }

    /** Increase the Very-Long-Term Importance of a given Handle by 1 */
    void incVLTI(Handle h) const { h->incVLTI(); }

    /** Decrease the Very-Long-Term Importance of a given Handle by 1 */
    void decVLTI(Handle h) const { h->decVLTI(); }

    /** Retrieve the Short-Term Importance of a given Handle */
    AttentionValue::sti_t getSTI(Handle h) const {
        return h->getAttentionValue()->getSTI();
    }

    /** Retrieve the Long-term Importance of a given atom */
    AttentionValue::lti_t getLTI(Handle h) const {
        return h->getAttentionValue()->getLTI();
    }

    /** Retrieve the Very-Long-Term Importance of a given atom */
    AttentionValue::vlti_t getVLTI(Handle h) const {
        return h->getAttentionValue()->getVLTI();
    }

    /** Retrieve the outgoing set of a given link */
    const HandleSeq& getOutgoing(Handle h) const {
        static HandleSeq empty;
        LinkPtr lll(LinkCast(h));
        if (lll) return lll->getOutgoingSet();
        return empty;
    }

    /** Retrieve a single Handle from the outgoing set of a given link */
    Handle getOutgoing(Handle h, Arity idx) const {
        LinkPtr lll = LinkCast(h);
        if (lll) return lll->getOutgoingAtom(idx);
        return Handle::UNDEFINED;
    }

    /** Retrieve the arity of a given link */
    Arity getArity(Handle h) const {
        LinkPtr lll(LinkCast(h));
        if (lll) return lll->getArity();
        return 0;
    }

    /** Return whether s is the source handle in a link l */
    bool isSource(Handle source, Handle link) const
    {
        LinkPtr l(LinkCast(link));
        if (l) return l->isSource(source);
        return false;
    }

    /** Retrieve the AttentionValue of a given Handle */
    AttentionValuePtr getAV(Handle h) const {
        return h->getAttentionValue();
    }

    /** Change the AttentionValue of a given Handle */
    void setAV(Handle h, AttentionValuePtr av) const {
        h->setAttentionValue(av);
    }

    /** Retrieve the type of a given Handle */
    Type getType(Handle h) const {
        return h->getType();
    }

    /** Retrieve the TruthValue of a given Handle */
    TruthValuePtr getTV(Handle h) const
    {
        return h->getTruthValue();
    }

    strength_t getMean(Handle h) const {
        return h->getTruthValue()->getMean();
    }

    confidence_t getConfidence(Handle h) const {
        return h->getTruthValue()->getConfidence();
    }

    /** Change the TruthValue of a given Handle */
    void setTV(Handle h, TruthValuePtr tv) {
        h->setTruthValue(tv);
    }

    /**
     * Return true if the handle belongs to *this* atomspace; else
     * return false.  Note that the handle might still be valid in
     * some other atomsapce. */
    bool isValidHandle(Handle h) const {
        // The h->getHandle() maneuver below is a trick to get at the
        // UUID of the actual atom, rather than the cached UUID in the
        // handle. Technically, this is not quite right, since perhaps
        // handles with valid UUID's but unresolved atom pointers are
        // "valid".  This call is essentially forcing resolution :-(
        // We should also be confirming that the UUID is OK, by asking
        // the TLB about it.  Anyway, this check is not entirely technically
        // correct ... worse, its a potentially performance-critical check,
        // as its called fairly often (I think).
        // return (NULL != h) and TLB::isValidHandle(h->getHandle());
        return (NULL != h) and (h->getHandle() != Handle::UNDEFINED);
    }

    /** Retrieve the doubly normalised Short-Term Importance between -1..1
     * for a given Handle. STI above and below threshold normalised separately
     * and linearly.
     *
     * @param h The atom handle to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to -1..1? Outside this
     * range can be returned if average=true
     * @return normalised STI between -1..1
     */
    float getNormalisedSTI(Handle h, bool average=true, bool clip=false) const {
        return getAttentionBankconst().getNormalisedSTI(h->getAttentionValue(), average, clip);
    }

    /** Retrieve the linearly normalised Short-Term Importance between 0..1
     * for a given Handle.
     *
     * @param h The atom handle to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to 0..1? Outside this
     * range can be returned if average=true
     * @return normalised STI between 0..1
     */
    float getNormalisedZeroToOneSTI(Handle h, bool average=true, bool clip=false) const {
        return getAttentionBankconst().getNormalisedZeroToOneSTI(h->getAttentionValue(), average, clip);
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
    HandleSeq getNeighbors(const Handle& h, bool fanin, bool fanout,
            Type linkType=LINK, bool subClasses=true) const
    {
        return getImplconst().getNeighbors(h, fanin, fanout, linkType, subClasses);
    }

    /** Retrieve the incoming set of a given atom */
    HandleSeq getIncoming(Handle h) {
        return getImpl().getIncoming(h);
    }

    /** Convenience functions... */
    bool isNode(Handle h) const { return NodeCast(h) != NULL; }
    bool isLink(Handle h) const { return LinkCast(h) != NULL; }

    /**
     * Gets a set of handles that matches with the given arguments.
     *
     * @param result An output iterator.
     * @param type the type of the atoms to be searched
     * @param name the name of the atoms to be searched.
     *        For searching only links, use "" or a search by type.
     * @param subclass if sub types of the given type are accepted
     *        in this search
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument. Example of a
     * call to this method, which would return all entries in AtomSpace:
     * @code
     *      std::list<Handle> ret;
     *      atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByName(OutputIterator result,
                 const std::string& name,
                 Type type = ATOM,
                 bool subclass = true) const
    {
        return getAtomTable().getHandlesByName(result, name, type, subclass);
    }

    /** Identical to above. Do not use in new code.  */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const char* name,
                 Type type,
                 bool subclass = true) const
    {
        return getAtomTable().getHandlesByName(result, name, type, subclass);
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally).
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * @note The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in AtomSpace:
     * @code
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByType(OutputIterator result,
                 Type type,
                 bool subclass = false) const
    {
        return getAtomTable().getHandlesByType(result, type, subclass);
    }

    /**
     * Returns the set of atoms of a given type which have atoms of a
     * given target type in their outgoing set (subclasses optionally).
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param targetType The desired target type.
     * @param subclass Whether type subclasses should be considered.
     * @param targetSubclass Whether target type subclasses should be considered.
     * @return The set of atoms of a given type and target type (subclasses
     * optionally).
     *
     * @note The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in AtomSpace:
     * @code
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 Type targetType,
                 bool subclass,
                 bool targetSubclass) const
    {
        return getAtomTable().getHandlesByTargetType(result,
               type, targetType, subclass, targetSubclass);
    }

    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     * i.e. returns the incoming set for that handle, but filtered
     * by the Type you specify.
     *
     * @param result An output iterator.
     * @param handle The handle that must be in the outgoing set of the atom.
     * @param type The type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type with the given handle in
     * their outgoing set.
     *
     * @note The matched entries are appended to a container whose 
     *       OutputIterator is passed as the first argument.
     *       Example of call to this method, which would return all
     *       entries in AtomSpace:
     * @code
     *         // Handle h == the Handle for your choice of Atom
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), h, ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Handle handle,
                 Type type,
                 bool subclass) const
    {
        return handle->getIncomingSetByType(result, type, subclass);
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
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * @note The matched entries are appended to a container whose OutputIterator
     * is passed as the first argument. Example of call to this method, which
     * would return all entries in AtomSpace:
     * @code
     *     std::list<Handle> ret;
     *     atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByOutgoing(OutputIterator result,
                 const HandleSeq& handles,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass) const
    {
        UnorderedHandleSet hs = getAtomTable().getHandlesByOutgoing(handles,
                types, subclasses, arity, type, subclass);
        return std::copy(hs.begin(), hs.end(), result);
    }

    /**
     * Returns the set of atoms whose outgoing set contains at least one
     * atom with the given name and type (atom type and subclasses
     * optionally).
     *
     * @param result An output iterator.
     * @param targetName The name of the atom in the outgoing set of
     *        the searched atoms.
     * @param targetType The type of the atom in the outgoing set of
     *        the searched atoms.
     * @param type type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type and name whose outgoing
     * set contains at least one atom of the given type and name.
     *
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument.  Example of call to
     * this method, which would return all entries in AtomSpace:
     *
     * @code
     * std::list<Handle> ret;
     * atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const char* targetName,
                 Type targetType,
                 Type type,
                 bool subclass) const
    {
        Handle targh(getAtomTable().getHandle(targetType, targetName));
        return targh->getIncomingSetByType(result, type, subclass);
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
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * @note The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in AtomSpace:
     * @code
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const char** names,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass) const
    {
        UnorderedHandleSet hs = getAtomTable().getHandlesByNames(names,
            types, subclasses, arity, type, subclass);
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
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument.  Example of call to this
     * method, which would return all entries in AtomSpace:
     *
     * @code
     * std::list<Handle> ret;
     * atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass) const
    {
        UnorderedHandleSet hs = getAtomTable().getHandlesByTypes(types,
            subclasses, arity, type, subclass);
        return std::copy(hs.begin(), hs.end(), result);
    }

    /**
     * Returns the set of atoms within the given importance range.
     *
     * @param Importance range lower bound (inclusive).
     * @param Importance range upper bound (inclusive).
     * @return The set of atoms within the given importance range.
     *
     * @note: This method utilizes the ImportanceIndex
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByAV(OutputIterator result,
                   AttentionValue::sti_t lowerBound,
                   AttentionValue::sti_t upperBound = AttentionValue::MAXSTI) const
    {
        UnorderedHandleSet hs = getAtomTable().getHandlesByAV(lowerBound, upperBound);
        return std::copy(hs.begin(), hs.end(), result);
    }

    /**
     * Gets the set of all handles in the Attentional Focus
     *
     * @return The set of all atoms in the Attentional Focus
     * @note: This method utilizes the ImportanceIndex
     */
    template <typename OutputIterator> OutputIterator
    getHandleSetInAttentionalFocus(OutputIterator result) const
    {
        return getHandlesByAV(result, getAttentionalFocusBoundary(), AttentionValue::AttentionValue::MAXSTI);
    }

    // Wrapper for comparing atoms from a HandleSeq
    template <typename Compare>
    struct compareAtom
    {
        Compare* c;
        compareAtom(Compare* _c) : c(_c) {}

        bool operator()(Handle h1, Handle h2) {
            return (*c)(h1, h2);
        }
    };

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally) and a given criterion.
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param compare A criterion for including atoms. It must be something
     * that returns a bool when called.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument.  Example of call to this
     * method, which would return all entries in AtomSpace beyond 500 LTI:
     * @code
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true, LTIAboveThreshold(500));
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSetFiltered(OutputIterator result,
                 Type type,
                 bool subclass,
                 AtomPredicate* compare) const
    {
        HandleSeq hs;
        getHandlesByType(back_inserter(hs), type, subclass);

        return std::copy_if(hs.begin(), hs.end(), result,
            [&](Handle h)->bool { return (*compare)(h); } );
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
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument.  Example of call to this
     * method, which would return all entries in AtomSpace, sorted by STI:
     * @code
     *         std::list<Handle> ret;
     *         AttentionValue::STISort stiSort;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true, stiSort);
     * @endcode
     */
    template <typename OutputIterator, typename Compare> OutputIterator
    getSortedHandleSet(OutputIterator result,
                 Type type,
                 bool subclass,
                 Compare compare) const
    {
        // get the handle set as a vector and sort it.
        std::vector<Handle> hs;

        getHandlesByType(back_inserter(hs), type, subclass);
        std::sort(hs.begin(), hs.end(), compareAtom<AtomComparator>(compare));
        return std::copy(hs.begin(), hs.end(), result);
    }


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
        getHandlesByType(back_inserter(handle_set), atype, subclass);

        // Loop over all handles in the handle set.
        std::list<Handle>::iterator i = handle_set.begin();
        std::list<Handle>::iterator iend = handle_set.end();
        for (; i != iend; i++) {
            bool rc = (data->*cb)(*i);
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

    /** Get attentional focus boundary
     * Generally atoms below this threshold shouldn't be accessed unless search
     * methods are unsuccessful on those that are above this value.
     *
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t getAttentionalFocusBoundary() const {
        return getAttentionBankconst().getAttentionalFocusBoundary();
    }

    /** Change the attentional focus boundary.
     * Some situations may benefit from less focussed searches.
     *
     * @param s New threshold
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t setAttentionalFocusBoundary(
        AttentionValue::sti_t s) {
        return getAttentionBank().setAttentionalFocusBoundary(s);
    }

    /** Get the maximum STI observed in the AtomSpace.
     * @param average If true, return an exponentially decaying average of
     * maximum STI, otherwise return the actual maximum.
     * @return Maximum STI
     */
    AttentionValue::sti_t getMaxSTI(bool average=true) const
    { return getAttentionBankconst().getMaxSTI(average); }

    /** Get the minimum STI observed in the AtomSpace.
     *
     * @param average If true, return an exponentially decaying average of
     * minimum STI, otherwise return the actual maximum.
     * @return Minimum STI
     */
    AttentionValue::sti_t getMinSTI(bool average=true) const
    { return getAttentionBankconst().getMinSTI(average); }

    /** Update the minimum STI observed in the AtomSpace.
     * Min/max are not updated on setSTI because average is calculate by lobe
     * cycle, although this could potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New minimum STI
     */
    void updateMinSTI(AttentionValue::sti_t m) { getAttentionBank().updateMinSTI(m); }

    /**
     * Update the maximum STI observed in the AtomSpace. Min/max are not updated
     * on setSTI because average is calculate by lobe cycle, although this could
     * potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New maximum STI
     */
    void updateMaxSTI(AttentionValue::sti_t m) { getAttentionBank().updateMaxSTI(m); }

    //! Clear the atomspace, remove all atoms
    void clear() { getImpl().clear(); }

// ---- filter templates

    // @deprecated do not use in new code!
    HandleSeq filter(AtomPredicate* compare)
    {
        HandleSeq hs;
        getHandleSetFiltered(back_inserter(hs), ATOM, true, compare);
        return hs;
    }

    // @deprecated do not use in new code!
    template<typename OutputIterator>
    OutputIterator filter(OutputIterator result, AtomPredicate* compare)
    {
        return getHandleSetFiltered(result, ATOM, true, compare);
    }

    /**
     * Filter handles from a sequence according to the given criterion.
     * XXX Do Not use this in new code FIXME TODO -- use std::copy_if() instead
     *
     * @param begin iterator for the sequence
     * @param end iterator for the sequence
     * @param struct or function embodying the criterion
     * @return The handles in the sequence that match the criterion.
     * @deprecated do not use in new code!
     */
    template<typename InputIterator>
    HandleSeq filter(InputIterator begin, InputIterator end, AtomPredicate* compare) const
    {
        HandleSeq result;
        std::copy_if(begin, end, back_inserter(result),
            [&](Handle h)->bool { return (*compare)(h); } ); 
        return result;
    }

    // XXX FIXME TODO -- don't use this, use std::copy_if
    template<typename InputIterator, typename OutputIterator>
    OutputIterator filter(InputIterator begin, InputIterator end, OutputIterator result, AtomPredicate* compare) const
    {
        return std::copy_if(begin, end, result,
            [&](Handle h)->bool { return (*compare)(h); } ); 
    }

// ---- custom filter templates

    struct TruePredicate : public AtomPredicate {
        TruePredicate(){}
        virtual bool test(AtomPtr atom) { return true; }
    };

    template<typename InputIterator>
    HandleSeq filter_InAttentionalFocus(InputIterator begin, InputIterator end) const {
        STIAboveThreshold sti_above(getAttentionalFocusBoundary());
        return filter(begin, end, &sti_above);
    }

    struct STIAboveThreshold : public AtomPredicate {
        STIAboveThreshold(const AttentionValue::sti_t t) : threshold (t) {}

        virtual bool test(AtomPtr atom) {
            return atom->getAttentionValue()->getSTI() > threshold;
        }
        AttentionValue::sti_t threshold;
    };

    struct LTIAboveThreshold : public AtomPredicate {
        LTIAboveThreshold(const AttentionValue::lti_t t) : threshold (t) {}

        virtual bool test(AtomPtr atom) {
            return atom->getAttentionValue()->getLTI() > threshold;
        }
        AttentionValue::lti_t threshold;
    };

    boost::signals2::connection addAtomSignal(const AtomSignal::slot_type& function)
    {
        return getImpl().atomTable.addAtomSignal().connect(function);
    }
    boost::signals2::connection removeAtomSignal(const AtomPtrSignal::slot_type& function)
    {
        return getImpl().atomTable.removeAtomSignal().connect(function);
    }
    boost::signals2::connection AVChangedSignal(const AVCHSigl::slot_type& function)
    {
        return getImpl().atomTable.AVChangedSignal().connect(function);
    }
    boost::signals2::connection TVChangedSignal(const TVCHSigl::slot_type& function)
    {
        return getImpl().atomTable.TVChangedSignal().connect(function);
    }
    boost::signals2::connection AddAFSignal(const AVCHSigl::slot_type& function)
    {
        return getAttentionBank().AddAFSignal().connect(function);
    }
    boost::signals2::connection RemoveAFSignal(const AVCHSigl::slot_type& function)
    {
        return getAttentionBank().RemoveAFSignal().connect(function);
    }
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_H
