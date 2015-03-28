/*
 * opencog/atomspace/AtomSpace.h
 *
 * Copyright (C) 2008-2011 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2015 Linas Vepstas
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

#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/AttentionBank.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/BackingStore.h>
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
 */
class AtomSpace
{
    friend class SavingLoading;
    friend class SQLPersistSCM;
    friend class ::AtomTableUTest;

    /**
     * Override and declare copy constructor and equals operator, to
     * prevent the accidental copying of large objects.
     */
    AtomSpace& operator=(const AtomSpace&);
    AtomSpace(const AtomSpace&);

    AtomTable atomTable;
    AttentionBank bank;
    /**
     * Used to fetch atoms from disk.
     */
    BackingStore *backing_store;

    AtomTable& getAtomTable(void) { return atomTable; }
protected:

    /**
     * Register a provider of backing storage.
     */
    void registerBackingStore(BackingStore *);
    void unregisterBackingStore(BackingStore *);

public:
    AtomSpace(AtomSpace* parent = NULL);
    ~AtomSpace();

    /**
     * Return the number of atoms contained in the space.
     */
    inline int getSize() const { return atomTable.getSize(); }
    inline int getNumNodes() const { return atomTable.getNumNodes(); }
    inline int getNumLinks() const { return atomTable.getNumLinks(); }

    //! Clear the atomspace, remove all atoms
    void clear();

    /**
     * Add an atom to the Atom Table.  If the atom already exists
     * then new truth value is ignored, and the existing atom is
     * returned.
     */
    Handle addAtom(AtomPtr atom, bool async=false);

    /**
     * Add a node to the Atom Table.  If the atom already exists
     * then that is returned.
     *
     * \param t     Type of the node
     * \param name  Name of the node
     */
    Handle addNode(Type t, const std::string& name = "",
                   bool async = false);

    /**
     * Add a link to the Atom Table. If the atom already exists, then
     * that is returned.
     *
     * @param t         Type of the link
     * @param outgoing  a const reference to a HandleSeq containing
     *                  the outgoing set of the link
     */
    Handle addLink(Type t, const HandleSeq& outgoing, bool async = false);

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
    void barrier(void) {
        atomTable.barrier();
        if (backing_store) backing_store->barrier();
    }

    /**
     * Unconditionally fetch an atom from the backingstore.
     * If there is no backingstore, then Handle::UNDEINFED is returned.
     * If the atom is found in the backingstore, then it is placed in
     * the atomtable before returning.  If the atom is already in the
     * atomtable, and is also found in the backingstore, then the TV's
     * are merged.
     *
     * The fetch is 'unconditional', in that it is fetched, even if it
     * already is in the atomspace.  Also, the ignored-types of the
     * backing store are not used.
     *
     * To avoid a fetch if the atom already is in the atomtable, use the
     * getAtom() method instead.
     */
    Handle fetchAtom(Handle h);

    /**
     * Get an atom from the AtomTable. If not found there, get it from
     * the backingstore (and add it to the AtomTable).  If the atom is
     * not found in either place, return Handle::UNDEFINED.
     */
    Handle getAtom(Handle);

    /**
     * Load *all* atoms of the given type, but only if they are not
     * already in the AtomTable.
     */
    void fetchAllAtomsOfType(Type t) {
        if (NULL == backing_store)
            throw RuntimeException(TRACE_INFO, "No backing store");
        backing_store->loadType(atomTable, t);
    }


    /**
     * Use the backing store to load the entire incoming set of the
     * atom.
     * If the flag is true, then the load is done recursively.
     * This method queries the backing store to obtain all atoms that
     * contain this one in their outgoing sets. All of these atoms are
     * then loaded into this atomtable/atomspace.
     */
    Handle fetchIncomingSet(Handle, bool);

    /**
     * Recursively store the atom to the backing store.
     * I.e. if the atom is a link, then store all of the atoms
     * in its outgoing set as well, recursively.
     */
    void storeAtom(Handle h);

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
        return 0 < atomTable.extract(h, recursive).size();
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
    bool removeAtom(Handle h, bool recursive = false);

    /**
     * Get a node from the AtomTable, if it's in there. If its not found
     * in the AtomTable, and there's a backing store, then the atom will
     * be fetched from the backingstore (and added to the AtomTable). If
     * the atom can't be found in either place, Handle::UNDEFINED will be
     * returned.
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    Handle getNode(Type t, const std::string& name = "");
    inline Handle getHandle(Type t, const std::string& str) {
        return getNode(t, str);
    }

    /**
     * Get a link from the AtomTable, if it's in there. If its not found
     * in the AtomTable, and there's a backing store, then the atom will
     * be fetched from the backingstore (and added to the AtomTable). If
     * the atom can't be found in either place, Handle::UNDEFINED will be
     * returned.
     *
     * See also the getAtom() method.
     *
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    Handle getLink(Type t, const HandleSeq& outgoing);
    Handle getHandle(Type t, const HandleSeq& outgoing) {
        return getLink(t, outgoing);
    }
    Handle getHandle(Type t, const Handle& ha) {
        HandleSeq outgoing;
        outgoing.push_back(ha);
        return getLink(t, outgoing);
    }
    Handle getHandle(Type t, const Handle& ha, const Handle& hb) {
        HandleSeq outgoing;
        outgoing.push_back(ha);
        outgoing.push_back(hb);
        return getLink(t, outgoing);
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
     * @note The matched entries are appended to a container whose
     *        OutputIterator is passed as the first argument.
     *
     * Example of call to this method, which would return all entries
     * in AtomSpace:
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
        return atomTable.getHandlesByType(result, type, subclass);
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

    /* ----------------------------------------------------------- */
    /* Attentional Focus stuff */

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
        return bank.getNormalisedSTI(h->getAttentionValue(), average, clip);
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
        return bank.getNormalisedZeroToOneSTI(h->getAttentionValue(), average, clip);
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
        UnorderedHandleSet hs = atomTable.getHandlesByAV(lowerBound, upperBound);
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

    /** Get attentional focus boundary
     * Generally atoms below this threshold shouldn't be accessed unless search
     * methods are unsuccessful on those that are above this value.
     *
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t getAttentionalFocusBoundary() const {
        return bank.getAttentionalFocusBoundary();
    }

    /** Change the attentional focus boundary.
     * Some situations may benefit from less focussed searches.
     *
     * @param s New threshold
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t setAttentionalFocusBoundary(
        AttentionValue::sti_t s) {
        return bank.setAttentionalFocusBoundary(s);
    }

    /** Get the maximum STI observed in the AtomSpace.
     * @param average If true, return an exponentially decaying average of
     * maximum STI, otherwise return the actual maximum.
     * @return Maximum STI
     */
    AttentionValue::sti_t getMaxSTI(bool average=true) const
    { return bank.getMaxSTI(average); }

    /** Get the minimum STI observed in the AtomSpace.
     *
     * @param average If true, return an exponentially decaying average of
     * minimum STI, otherwise return the actual maximum.
     * @return Minimum STI
     */
    AttentionValue::sti_t getMinSTI(bool average=true) const
    { return bank.getMinSTI(average); }

    /** Update the minimum STI observed in the AtomSpace.
     * Min/max are not updated on setSTI because average is calculate by lobe
     * cycle, although this could potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New minimum STI
     */
    void updateMinSTI(AttentionValue::sti_t m) { bank.updateMinSTI(m); }

    /**
     * Update the maximum STI observed in the AtomSpace. Min/max are not updated
     * on setSTI because average is calculate by lobe cycle, although this could
     * potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New maximum STI
     */
    void updateMaxSTI(AttentionValue::sti_t m) { bank.updateMaxSTI(m); }
    void updateSTIFunds(AttentionValue::sti_t m) { bank.updateSTIFunds(m); }
    void updateLTIFunds(AttentionValue::lti_t m) { bank.updateLTIFunds(m); }
    long getSTIFunds() const { return bank.getSTIFunds(); }
    long getLTIFunds() const { return bank.getLTIFunds(); }

    /* ----------------------------------------------------------- */
    // ---- Signals

    boost::signals2::connection addAtomSignal(const AtomSignal::slot_type& function)
    {
        return atomTable.addAtomSignal().connect(function);
    }
    boost::signals2::connection removeAtomSignal(const AtomPtrSignal::slot_type& function)
    {
        return atomTable.removeAtomSignal().connect(function);
    }
    boost::signals2::connection AVChangedSignal(const AVCHSigl::slot_type& function)
    {
        return atomTable.AVChangedSignal().connect(function);
    }
    boost::signals2::connection TVChangedSignal(const TVCHSigl::slot_type& function)
    {
        return atomTable.TVChangedSignal().connect(function);
    }
    boost::signals2::connection AddAFSignal(const AVCHSigl::slot_type& function)
    {
        return bank.AddAFSignal().connect(function);
    }
    boost::signals2::connection RemoveAFSignal(const AVCHSigl::slot_type& function)
    {
        return bank.RemoveAFSignal().connect(function);
    }

    /* ----------------------------------------------------------- */
    /* Deprecated and obsolete code */

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

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    HandleSeq getIncoming(Handle h) const {
        HandleSeq hs;
        h->getIncomingSet(back_inserter(hs));
        return hs;
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    bool isNode(Handle h) const { return NodeCast(h) != NULL; }
    bool isLink(Handle h) const { return LinkCast(h) != NULL; }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    std::string atomAsString(Handle h, bool terse = true) const {
        if (terse) return h->toShortString();
        return h->toString();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    const std::string& getName(Handle h) const {
        static std::string noname;
        NodePtr nnn(NodeCast(h));
        if (nnn) return nnn->getName();
        return noname;
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    void setSTI(Handle h, AttentionValue::sti_t stiValue) const {
        h->setSTI(stiValue);
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    void setLTI(Handle h, AttentionValue::lti_t ltiValue) const {
        h->setLTI(ltiValue);
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    void incVLTI(Handle h) const { h->incVLTI(); }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    void decVLTI(Handle h) const { h->decVLTI(); }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    AttentionValue::sti_t getSTI(Handle h) const {
        return h->getAttentionValue()->getSTI();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    AttentionValue::lti_t getLTI(Handle h) const {
        return h->getAttentionValue()->getLTI();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    AttentionValue::vlti_t getVLTI(Handle h) const {
        return h->getAttentionValue()->getVLTI();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    const HandleSeq& getOutgoing(Handle h) const {
        static HandleSeq empty;
        LinkPtr lll(LinkCast(h));
        if (lll) return lll->getOutgoingSet();
        return empty;
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    Handle getOutgoing(Handle h, Arity idx) const {
        LinkPtr lll = LinkCast(h);
        if (lll) return lll->getOutgoingAtom(idx);
        return Handle::UNDEFINED;
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    Arity getArity(Handle h) const {
        LinkPtr lll(LinkCast(h));
        if (lll) return lll->getArity();
        return 0;
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    bool isSource(Handle source, Handle link) const
    {
        LinkPtr l(LinkCast(link));
        if (l) return l->isSource(source);
        return false;
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    AttentionValuePtr getAV(Handle h) const {
        return h->getAttentionValue();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    void setAV(Handle h, AttentionValuePtr av) const {
        h->setAttentionValue(av);
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    Type getType(Handle h) const {
        return h->getType();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    TruthValuePtr getTV(Handle h) const
    {
        return h->getTruthValue();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    strength_t getMean(Handle h) const {
        return h->getTruthValue()->getMean();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    confidence_t getConfidence(Handle h) const {
        return h->getTruthValue()->getConfidence();
    }

    /** DEPRECATED! Do NOT USE IN NEW CODE!
     * If you need this, just copy the code below into your app! */
    void setTV(Handle h, TruthValuePtr tv) const {
        h->setTruthValue(tv);
    }
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_H
