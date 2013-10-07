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

#include <boost/scoped_ptr.hpp>

#include <opencog/atomspace/AtomSpaceAsync.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/BackingStore.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/util/exceptions.h>

// Whether to wrap certain functions in lru_cache_threaded<>
#define USE_ATOMSPACE_LOCAL_THREAD_CACHE 1

#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
#include <opencog/util/lru_cache.h>
#endif


namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

typedef boost::shared_ptr<TruthValue> TruthValuePtr;

/**
 * The AtomSpace class is a legacy interface to OpenCog's AtomSpace and
 * provide standard functions that return results immediately.
 * The AtomSpace is a wrapper around opencog::AtomSpaceAsync which submits
 * requests to a queue.
 * These requests (defined in ASRequest.h) access the core
 * opencog::AtomSpaceImpl class. For an asynchronous interface that allows
 * AtomSpace requests to be fired and forgot (mostly useful for setting TVs,
 * queuing handleset queries or creating Atoms), see AtomSpaceAsync.
 *
 * The default constructor will create a new AtomSpaceAsync. To connect to
 * an existing AtomSpaceAsync and send requests to it, pass a reference.
 *
 * @code
 *  // Create an atomspace with it's own internal AtomSpace event-loop.
 *  AtomSpace atomspace;
 *
 *  // create AtomSpaceAsync and start event loop
 *  AtomSpaceAsync& atomSpaceAsync;
 *  // wrap it in this legacy AtomSpace API
 *  AtomSpace atomspace2(atomSpaceAsync);
 *
 *  // Share the event-loop of atomspace
 *  AtomSpace atomspace3(*atomspace.atomSpaceAsync);
 * @endcode
 *
 * If one were to add atoms to via atomspace3 or atomspace, they would both be
 * in the same "AtomSpace". Adding atoms via atomSpaceAsync or atomspace2 would
 * place atoms in another "AtomSpace" separate from atomspace and atomspace3.
 */
class AtomSpace
{
    friend class SavingLoading;
    friend class ::AtomTableUTest;
    friend class SaveRequest;

    //! Indicates whether the AtomSpace should delete AtomSpaceAsync on destruction
    bool ownsAtomSpaceAsync;

    /**
     * Overrides and declares equals operator as private 
     * for avoiding large object copying by mistake.
     */
    AtomSpace& operator=(const AtomSpace&);
public:
    void do_merge_tv(Handle, const TruthValue&);

    /** 
     * The AtomSpace class is essentially just be a wrapper of the asynchronous
     * AtomSpaceAsync which returns ASRequest "futures" as well as allowing
     * thread-local caching of some requests. Functions in this
     * class will block until notified that they've been fulfilled by the
     * AtomSpaceAsync event loop.
     */
    mutable AtomSpaceAsync* atomSpaceAsync;

    AtomSpace(void);
    /**
     * Create an atomspace that will send requests to an existing AtomSpace
     * event-loop.
     */
    AtomSpace(AtomSpaceAsync& a);

    AtomSpace(const AtomSpace&);
    ~AtomSpace();

    /**
     * Recursively store the atom to the backing store.
     * I.e. if the atom is a link, then store all of the atoms
     * in its outgoing set as well, recursively.
     * @deprecated Use AtomSpaceAsync::storeAtom in new code.
     */
    inline void storeAtom(Handle h) {
        atomSpaceAsync->storeAtom(h)->get_result();
    }

    /**
     * Return the atom with the indicated handle. This method will
     * explicitly use the backing store to obtain an instance of the
     * atom. If an atom corresponding to the handle cannot be found,
     * then an undefined handle is returned. If the atom is found, 
     * then the corresponding atom is guaranteed to have been
     * instantiated in the atomspace.
     * @deprecated Use AtomSpaceAsync::fetchAtom in new code.
     */
    inline Handle fetchAtom(Handle h) {
        return atomSpaceAsync->fetchAtom(h)->get_result();
    }

    /**
     * Use the backing store to load the entire incoming set of the atom.
     * If the flag is true, then the load is done recursively. 
     * This method queries the backing store to obtain all atoms that 
     * contain this one in their outgoing sets. All of these atoms are
     * then loaded into this AtomSpace's AtomTable.
     * @deprecated Use AtomSpaceAsync::fetchIncomingSet in new code.
     */
    inline Handle fetchIncomingSet(Handle h, bool recursive) {
        return atomSpaceAsync->fetchIncomingSet(h,recursive)->get_result();
    };

    inline AttentionBank& getAttentionBank()
    { return atomSpaceAsync->getAttentionBank(); }

    /**
     * Return the number of atoms contained in the space.
     */
    inline int getSize() const { return atomSpaceAsync->getSize()->get_result(); }

    /**
     * Prints atoms of this AtomSpace to the given output stream.
     * @param output  the output stream where the atoms will be printed.
     * @param type  the type of atoms that should be printed.
     * @param subclass  if true, matches all atoms whose type is
     *              subclass of the given type. If false, matches
     *              only atoms of the exact type.
     */
    void print(std::ostream& output = std::cout,
               Type type = ATOM, bool subclass = true) const;

    /** Add a new node to the Atom Table,
     * if the atom already exists then the old and the new truth value is merged
     * \param t     Type of the node
     * \param name  Name of the node
     * \param tvn   Optional TruthValue of the node. If not provided, uses the DEFAULT_TV (see TruthValue.h) 
     * @deprecated New code should directly use the AtomSpaceAsync::addNode method.
     */
    inline Handle addNode(Type t, const std::string& name = "", const TruthValue& tvn = TruthValue::DEFAULT_TV())
    {
        return atomSpaceAsync->addNode(t,name,tvn)->get_result();
    }

    /**
     * Add a new node to the AtomTable. A random 16-character string
     * will be appended to the provided name.
     * @todo: Later on, the names can include server/time info to decrease
     * the probability of collisions and be more informative.
     **/
    Handle addPrefixedNode(Type t, const std::string& prefix = "", const TruthValue& tvn = TruthValue::DEFAULT_TV());

    /**
     * Add a new link to the Atom Table
     * If the atom already exists then the old and the new truth value
     * is merged
     * @param t         Type of the link
     * @param outgoing  a const reference to a HandleSeq containing
     *                  the outgoing set of the link
     * @param tvn       Optional TruthValue of the node. If not
     *                  provided, uses the DEFAULT_TV (see TruthValue.h)
     * @deprecated New code should directly use the AtomSpaceAsync::addLink method.
     */
    inline Handle addLink(Type t, const HandleSeq& outgoing,
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
    { 
        return atomSpaceAsync->addLink(t,outgoing,tvn)->get_result();
    }

    inline Handle addLink(Type t, Handle h,
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(h);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb,
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc,
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc, Handle hd,
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
    {
        HandleSeq oset;
        oset.push_back(ha);
        oset.push_back(hb);
        oset.push_back(hc);
        oset.push_back(hd);
        return addLink(t, oset, tvn);
    }

    inline Handle addLink(Type t, Handle ha, Handle hb, Handle hc, Handle hd, Handle he,
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
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
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
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
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
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
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
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
                   const TruthValue& tvn = TruthValue::DEFAULT_TV())
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
        return atomSpaceAsync->removeAtom(h,recursive)->get_result();
    }

    /**
     * Retrieve from the Atom Table the Handle of a given node
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    Handle getHandle(Type t, const std::string& str) const {
        return atomSpaceAsync->getHandle(t,str)->get_result();
    }

    /**
     * Retrieve from the Atom Table the Handle of a given link
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    Handle getHandle(Type t, const HandleSeq& outgoing) const {
        return atomSpaceAsync->getHandle(t,outgoing)->get_result();
    }

    /** Get the atom referred to by Handle h represented as a string. */
    std::string atomAsString(Handle h, bool terse = true) const {
        return atomSpaceAsync->atomAsString(h,terse)->get_result();
    }

    /** Retrieve the name of a given Handle */
    std::string getName(Handle h) const {
        return atomSpaceAsync->getName(h)->get_result();
    }

    /** Change the Short-Term Importance of a given Handle */
    void setSTI(Handle h, AttentionValue::sti_t stiValue) {
        atomSpaceAsync->setSTI(h, stiValue)->get_result();
    }

    /** Change the Long-term Importance of a given Handle */
    void setLTI(Handle h, AttentionValue::lti_t ltiValue) {
        atomSpaceAsync->setLTI(h, ltiValue)->get_result();
    }

    /** Increase the Very-Long-Term Importance of a given Handle by 1 */
    void incVLTI(Handle h) {
        atomSpaceAsync->incVLTI(h)->get_result();
    }

    /** Decrease the Very-Long-Term Importance of a given Handle by 1 */
    void decVLTI(Handle h) {
        atomSpaceAsync->decVLTI(h)->get_result();
    }
    
    /** Retrieve the Short-Term Importance of a given Handle */
    AttentionValue::sti_t getSTI(Handle h) const {
        return atomSpaceAsync->getSTI(h)->get_result();
    }

    /** Retrieve the Long-term Importance of a given AttentionValueHolder */
    AttentionValue::lti_t getLTI(Handle h) const {
        return atomSpaceAsync->getLTI(h)->get_result();
    }

    /** Retrieve the Very-Long-Term Importance of a given
     * AttentionValueHolder */
    AttentionValue::vlti_t getVLTI(Handle h) const {
        return atomSpaceAsync->getVLTI(h)->get_result();
    }

    /** Retrieve the outgoing set of a given link */
    HandleSeq getOutgoing(Handle h) const {
        return atomSpaceAsync->getOutgoing(h)->get_result();
    }

    /** Retrieve a single Handle from the outgoing set of a given link */
    Handle getOutgoing(Handle h, int idx) const {
        return atomSpaceAsync->getOutgoing(h,idx)->get_result();
    }

    /** Retrieve the arity of a given link */
    int getArity(Handle h) const {
        return atomSpaceAsync->getArity(h)->get_result();
    }

    /** Return whether s is the source handle in a link l */ 
    bool isSource(Handle source, Handle link) const {
        return atomSpaceAsync->isSource(source, link)->get_result();
    }

    /** Retrieve the AttentionValue of a given Handle */
    AttentionValue getAV(Handle h) const;

    /** Change the AttentionValue of a given Handle */
    void setAV(Handle h, const AttentionValue &av);

    /** Retrieve the type of a given Handle */
    Type getType(Handle h) const;

#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    // Experimental code for speeding up TV retrieval...
    /** Cached get type function */
    class _getType : public std::unary_function<Handle, Type> {
        AtomSpace* a;
        public:
        _getType(AtomSpace* _a) : a(_a) { };
        Type operator()(const Handle& h) const {
            return a->atomSpaceAsync->getType(h)->get_result();
        }
    };
    _getType* __getType;
    // dummy get type version which is cached using lru_cache
    lru_cache_threaded<AtomSpace::_getType> *getTypeCached;
#endif // USE_ATOMSPACE_LOCAL_THREAD_CACHE


    /* Retrieve the TruthValue summary of a given Handle
     */
    //tv_summary_t getTV(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) const;

    /** Retrieve the TruthValue of a given Handle
     * @note This is an unpleasant hack which is unsafe as it returns a pointer
     * to the AtomSpace TV which may be lost if the Atom the TV belongs to is
     * removed. It's much faster than having to copy the value and use smart
     * pointers though. Garbage collection should solve this.
     */
    TruthValuePtr getTV(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) const;

    strength_t getMean(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) const;  
    confidence_t getConfidence(Handle h, VersionHandle vh = NULL_VERSION_HANDLE) const;  

    /** Change the TruthValue of a given Handle */
    void setTV(Handle h, const TruthValue& tv, VersionHandle vh = NULL_VERSION_HANDLE);

    /** Change the primary TV's mean of a given Handle
     * @note By Joel: this makes no sense to me, how can you generally set a mean
     * across all TV types. I think a better solution would be to remove this
     * enforce the use of setTV.
     */
    void setMean(Handle h, float mean) {
        atomSpaceAsync->setMean(h, mean)->get_result();
    }

    /** Clone an atom from the AtomSpace, replaces the public access to TLB::getAtom
     * that many modules were doing.
     * @param h Handle of atom to clone
     * @return A smart pointer to the atom
     * @note Any changes to the atom object must be committed using
     * AtomSpace::commitAtom for them to be merged with the AtomSpace.
     * Otherwise changes are lost.
     */
    AtomPtr cloneAtom(const Handle& h) const;

    /** Commit an atom that has been cloned from the AtomSpace.
     *
     * @param a Atom to commit
     * @return whether the commit was successful
     */
    bool commitAtom(const AtomPtr a);

    bool isValidHandle(const Handle& h) const;

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
        return atomSpaceAsync->getNormalisedSTI(h, average, clip, false)->get_result();
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
        return atomSpaceAsync->getNormalisedSTI(h, average, clip, true)->get_result();
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
            Type linkType=LINK, bool subClasses=true) const {
        return atomSpaceAsync->getNeighbors(h,fanin,fanout,linkType,subClasses)->get_result();
    }

    /** Retrieve the incoming set of a given atom */
    HandleSeq getIncoming(Handle h) {
        return atomSpaceAsync->getIncoming(h)->get_result();
    }

    /** Convenience functions... */
    bool isNode(const Handle& h) const;
    bool isLink(const Handle& h) const;

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
     *      atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 const std::string& name,
                 bool subclass = true,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getHandlesByName(
                std::string(name), type, subclass, vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
    }

    /**
     * Returns the set of atoms of a given name (atom type and subclasses
     * optionally).
     *
     * @param result An output iterator.
     * @param name The desired name of the atoms.
     * @param type The type of the atom.
     * @param subclass Whether atom type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type and name.
     *
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument.
     *
     * @note Example of call to this method, which would return all entries in AtomSpace:
     * @code
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const char* name,
                 Type type,
                 bool subclass = true,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getHandlesByName(
                name, type, subclass, vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally).
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
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
    getHandleSet(OutputIterator result,
                 Type type,
                 bool subclass = false,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getHandlesByType(type, subclass, vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
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
     * @param vh only atoms that contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @param targetVh only atoms whose target contains versioned TVs with the given VersionHandle are returned.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
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
                 bool targetSubclass,
                 VersionHandle vh = NULL_VERSION_HANDLE,
                 VersionHandle targetVh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getHandlesByTarget(type, targetType,
                subclass, targetSubclass, vh, targetVh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
    }

    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     * i.e. returns the incoming set for that handle, but filtered by the Type you specify.
     * Uses a special index, so it's more efficient than filtering it yourself.
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
     * @note The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in AtomSpace:
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
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getHandlesByTargetHandle(handle,
                type, subclass, vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
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
     * @param vh only atoms that contains versioned TVs with the given
     * VersionHandle are returned. If NULL_VERSION_HANDLE is given, it does not
     * restrict the result.
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
    getHandleSet(OutputIterator result,
                 const HandleSeq& handles,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getHandlesByOutgoingSet(
                handles,types,subclasses,arity,type,subclass,vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
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
     * @param vh return only atoms that contains versioned TVs with the given
     * VersionHandle.  If NULL_VERSION_HANDLE is given, it does not restrict
     * the result.
     * @return The set of atoms of the given type and name whose outgoing
     * set contains at least one atom of the given type and name.
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
                 const char* targetName,
                 Type targetType,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE,
                 VersionHandle targetVh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getHandlesByTargetName(
               targetName, targetType, type, subclass, vh, targetVh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
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
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        HandleSeq result_set = atomSpaceAsync->getHandlesByTargetNames(
                names, types, subclasses, arity, type, subclass, vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
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
     * @param vh returns only atoms that contains versioned TVs with the given
     * VersionHandle.  If NULL_VERSION_HANDLE is given, it does not restrict
     * the result.
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
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        HandleSeq result_set = atomSpaceAsync->getHandlesByTargetTypes(
                types, subclasses, arity, type, subclass, vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
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
     * @note The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument.  Example of call to this
     * method, which would return all entries in AtomSpace in the
     * AttentionalFocus:
     * @code
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    template <typename OutputIterator> OutputIterator
    getHandleSetInAttentionalFocus(OutputIterator result,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        STIAboveThreshold stiAbove(getAttentionalFocusBoundary());
        return getHandleSetFiltered(result, type, subclass, &stiAbove, vh);

    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally) and a given criterion.
     *
     * @param result An output iterator.
     * @param type The desired type.
     * @param subclass Whether type subclasses should be considered.
     * @param compare A criterion for including atoms. It must be something
     * that returns a bool when called.
     * @param vh returns only atoms that contains versioned TVs with the given
     * VersionHandle.  If NULL_VERSION_HANDLE is given, it does not restrict
     * the result.
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
                 AtomPredicate* compare,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleSeq hs = atomSpaceAsync->filter(compare, type, subclass, vh)->get_result();
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
     * @param vh returns only atoms that contains versioned TVs with the given
     * VersionHandle.  If NULL_VERSION_HANDLE is given, it does not restrict
     * the result.
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
                 Compare compare,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleSeq result_set = atomSpaceAsync->getSortedHandleSet(
                type, subclass, compare, vh)->get_result();
        return std::copy(result_set.begin(), result_set.end(), result);
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
     * Decays STI of all atoms (one cycle of importance decay).  Deprecated,
     * importance updating should be done by ImportanceUpdating Agent.
     * @deprecated ECAN should be used, but this method is still used by
     * embodiment.
     */
    void decayShortTermImportance();

    /** Get attentional focus boundary
     * Generally atoms below this threshold shouldn't be accessed unless search
     * methods are unsuccessful on those that are above this value.
     *
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t getAttentionalFocusBoundary() const;

    /** Change the attentional focus boundary.
     * Some situations may benefit from less focussed searches.
     *
     * @param s New threshold
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t setAttentionalFocusBoundary(
        AttentionValue::sti_t s);

    /** Get the maximum STI observed in the AtomSpace.
     * @param average If true, return an exponentially decaying average of
     * maximum STI, otherwise return the actual maximum.
     * @return Maximum STI
     */
    AttentionValue::sti_t getMaxSTI(bool average=true)
    { return getAttentionBank().getMaxSTI(average); } 

    /** Get the minimum STI observed in the AtomSpace.
     *
     * @param average If true, return an exponentially decaying average of
     * minimum STI, otherwise return the actual maximum.
     * @return Minimum STI
     */
    AttentionValue::sti_t getMinSTI(bool average=true)
    { return getAttentionBank().getMinSTI(average); } 

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

    int Nodes(VersionHandle = NULL_VERSION_HANDLE) const;
    int Links(VersionHandle = NULL_VERSION_HANDLE) const;

    //! Clear the atomspace, remove all atoms
    void clear();

// ---- filter templates

    HandleSeq filter(AtomPredicate* compare, VersionHandle vh = NULL_VERSION_HANDLE) {
        return atomSpaceAsync->filter(compare,ATOM,true,vh)->get_result();
    }

    template<typename OutputIterator>
    OutputIterator filter(OutputIterator it, AtomPredicate* compare, VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeq result = atomSpaceAsync->filter(compare,ATOM,true,vh)->get_result();
        foreach(Handle h, result) 
            * it++ = h;
        return it;
    }

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
        for (; begin != end; begin++) {
            //std::cout << "evaluating atom " << atomAsString(*begin) << std::endl;
            if ((*compare)(*cloneAtom(*begin))) {
                //std::cout << "passed! " <<  std::endl;
                result.push_back(*begin);
            }
        }
        return result;
    }

    template<typename InputIterator, typename OutputIterator>
    OutputIterator filter(InputIterator begin, InputIterator end, OutputIterator it, AtomPredicate* compare) const {
        for (; begin != end; begin++)
            if (compare(*begin))
                * it++ = *begin;
        return it;
    }

// ---- custom filter templates

    struct TruePredicate : public AtomPredicate {
        TruePredicate(){}
        virtual bool test(const Atom& atom) { return true; }
    };

    template<typename InputIterator>
    HandleSeq filter_InAttentionalFocus(InputIterator begin, InputIterator end) const {
        STIAboveThreshold sti_above(getAttentionalFocusBoundary());
        return filter(begin, end, &sti_above);
    }

    struct STIAboveThreshold : public AtomPredicate {
        STIAboveThreshold(const AttentionValue::sti_t t) : threshold (t) {}

        virtual bool test(const Atom& atom) {
            return atom.getAttentionValue().getSTI() > threshold;
        }
        AttentionValue::sti_t threshold;
    };

    struct LTIAboveThreshold : public AtomPredicate {
        LTIAboveThreshold(const AttentionValue::lti_t t) : threshold (t) {}

        virtual bool test(const Atom& atom) {
            return atom.getAttentionValue().getLTI() > threshold;
        }
        AttentionValue::lti_t threshold;
    };

    // Provide access to the atom-added signal in Python, but using a queue instead
    // of callbacks. It's accessible via the Cython wrapper.
    std::list<Handle> addAtomSignalQueue;


    static bool isHandleInSeq(Handle h, HandleSeq &seq);

private:
    /**
     * Remove stimulus from atom, only should be used when Atom is deleted.
     */
    void removeStimulus(Handle h);

    bool handleAddSignal(AtomSpaceImpl *, Handle);

#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    /** For monitoring removals to the AtomSpace so that cache entries can be
     * invalidated as necessary
     */
    bool atomRemoveSignal(AtomSpaceImpl *, AtomPtr);

    //! Whether AtomSpaceWrapper is listening for AtomSpace signals.
    bool watchingAtomSpace;

    boost::signals::connection c_add; //! Connection to add atom signals
    boost::signals::connection c_remove; //! Connection to remove atom signals

    void setUpCaching();
#endif


};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_H
