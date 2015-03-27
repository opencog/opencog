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
     * Override and declare copy constructor and equals operator, to
     * prevent the accidental copying of large objects.
     */
    AtomSpace& operator=(const AtomSpace&);
    AtomSpace(const AtomSpace&);

    /**
     * The AtomSpace class is just a wrapper of the AtomTable
     */
    AtomSpaceImpl* _atomSpaceImpl;

public:
    AtomSpace(AtomSpace* parent = NULL);

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
     * Add an atom to the Atom Table.  If the atom already exists
     * then new truth value is ignored, and the existing atom is
     * returned.
     */
    inline Handle addAtom(AtomPtr atom)
    {
        return getImpl().addAtom(atom);
    }

    /**
     * Add a node to the Atom Table.  If the atom already exists
     * then that is returned.
     *
     * \param t     Type of the node
     * \param name  Name of the node
     */
    inline Handle addNode(Type t, const std::string& name = "")
    {
        return getImpl().addNode(t, name);
    }

    /**
     * Add a new node to the AtomTable. A random 16-character string
     * will be appended to the provided name.
     *
     * @todo: Later on, the names can include server/time info to decrease
     * the probability of collisions and be more informative.
     **/
    Handle addPrefixedNode(Type t, const std::string& prefix = "");

    /**
     * Add a link to the Atom Table. If the atom already exists, then
     * that is returned.
     *
     * @param t         Type of the link
     * @param outgoing  a const reference to a HandleSeq containing
     *                  the outgoing set of the link
     */
    inline Handle addLink(Type t, const HandleSeq& outgoing)
    {
        return getImpl().addLink(t, outgoing);
    }

    inline Handle addLink(Type t, Handle h)
    {
        HandleSeq oset;
        oset.push_back(h);
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc, Handle hd)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        oset.push_back(hf);
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf, Handle hg)
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        oset.push_back(he);
        oset.push_back(hf);
        oset.push_back(hg);
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf, Handle hg,
                          Handle hh)
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
        return addLink(t, oset);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                          Handle hd, Handle he, Handle hf, Handle hg,
                          Handle hh, Handle hi)
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
        return addLink(t, oset);
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
     * or AtomPtr's that reference it; it is deleted only when the
     * last reference goes away.
     *
     * @param h The Handle of the atom to be removed.
     * @param recursive Recursive-removal flag. If the flag is set,
     *       then this atom, and *everything* that points to it will
     *       be removed from the atomspace.  This can cause a large
     *       cascade of removals!  If the flag is not set, then the
     *       atom will be removed only if its incoming set is empty.
     *       By default, recursion is disabled.
     * @return True if the Atom for the given Handle was successfully
     *         removed. False, otherwise.
     */
    bool purgeAtom(Handle h, bool recursive = false) {
        return getImpl().purgeAtom(h, recursive);
    }

    /**
     * Removes an atom from the atomspace, and any attached storage.
     * The atom remains valid as long as there are Handles or AtomPtr's
     * that reference it; it is deleted only when the last reference
     * goes away.
     *
     * @param h The Handle of the atom to be removed.
     * @param recursive Recursive-removal flag. If the flag is set,
     *       then this atom, and *everything* that points to it will
     *       be removed from the atomspace.  This can cause a large
     *       cascade of removals!  If the flag is not set, then the
     *       atom will be removed only if its incoming set is empty.
     *       By default, recursion is disabled.
     * @return True if the Atom for the given Handle was successfully
     *         removed. False, otherwise.
     */
    bool removeAtom(Handle h, bool recursive = false) {
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
    Handle getHandle(Type t, const Handle& ha) {
        HandleSeq outgoing;
        outgoing.push_back(ha);
        return getImpl().getLink(t, outgoing);
    }
    Handle getHandle(Type t, const Handle& ha, const Handle& hb) {
        HandleSeq outgoing;
        outgoing.push_back(ha);
        outgoing.push_back(hb);
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
    void setTV(Handle h, TruthValuePtr tv) const {
        h->setTruthValue(tv);
    }

    /**
     * Return true if the handle points to an atom that is in some
     * (any) atomspace; else return false.
     *
     * Currently, this code has the side-effect of resolving the handle.
     * That is, if the handle's atom-pointer was non-null, and the atom
     * can be located based on its UUID, then the atom will be
     * instantiated in the appropriate atomspace.  Thus, this method
     * returns false only if the handles UUID is -1 or if the UUID is
     * positive, but no atomspace can lay claim to it.
     *
     * A UUID can be positive, but not a part of any atomspace, if the
     * atom was recently removed from some atomspace. In this case, the
     * handle is still caching its old UUID, although the atom itself
     * now has a UUID of -1.
     *
     * Note also: a handle that is pointing to a recently-created atom
     * that is not in any atomspace will have a UUID of -1, and thus is
     * considered "invalid", even though it points to an atom that
     * exists in RAM (and is thus usable as a naked atom).
     */
    bool isValidHandle(Handle h) const {
        // The h->getHandle() maneuver below is a trick to get at the
        // UUID of the actual atom, rather than the cached UUID in the
        // handle. Technically, this is not quite right, since, in
        // principle, a handle could have a valid UUID, but the atom
        // pointer is null, because the atom is on disk, in a database,
        // is on a remote server, or has been purged from RAM.  But we
        // have no way of knowing that the situation really is, so it
        // seems like the only thing that can be done here is to resolve
        // the pointer (i.e. make it point to an acutal atom, in RAM,
        // and then check the actual UUID in the atom.
        //
        // The actual resolution is done with the (NULL != h) check,
        // which causes h.operator!=() to run, which fixes up the atom
        // pointer, as needed.
        //
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
     * Returns neighboring atoms, following incoming links and
     * returning their outgoing sets.
     *
     * @param h Get neighbours for the atom this handle points to.
     * @param fanin Whether directional (ordered) links point to this
     *              node should beconsidered.
     * @param fanout Whether directional (ordered) links point from this
     *               node to another should be considered.
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
     * DEPRECATED! DO NOT USE IN NEW CODE!
     * If you need this function, just cut and paste the code below into
     * whatever you are doing!
     */
    template <typename OutputIterator> OutputIterator
    getHandlesByName(OutputIterator result,
                     const std::string& name,
                     Type type = NODE,
                     bool subclass = true)
    {
        if (name.c_str()[0] == 0)
            return getHandlesByType(result, type, subclass);

        if (false == subclass) {
            Handle h(getHandle(type, name));
            if (h) *(result++) = h;
            return result;
        }

        classserver().foreachRecursive(
            [&](Type t)->void {
                Handle h(getHandle(t, name));
                if (h) *(result++) = h; }, type);

        return result;
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
     *         atomSpace.getHandlesByType(back_inserter(ret), ATOM, true);
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
     * DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this function, just copy the one-liner below.
     * XXX ONLY the python bindings use this. XXX kill that code.
     */
    template <typename OutputIterator> OutputIterator
    getIncomingSetByType(OutputIterator result,
                 Handle handle,
                 Type type,
                 bool subclass) const
    {
        return handle->getIncomingSetByType(result, type, subclass);
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
        for (; i != iend; ++i) {
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

// ---- Signals

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
