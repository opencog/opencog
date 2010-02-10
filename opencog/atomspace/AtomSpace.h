/*
 * opencog/atomspace/AtomSpace.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Carlos Lopes <dlopes@vettalabs.com>
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
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/BackingStore.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/HandleSet.h>
#include <opencog/atomspace/TimeServer.h>
#include <opencog/atomspace/SpaceServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/recent_val.h>


namespace opencog
{

typedef std::vector<HandleSet*> HandleSetSeq;

//XXX FIXME: do NOT inherit from SpaceServerContainer;
//instead used atom table signals to accomplish work.
class AtomSpace : public SpaceServerContainer
{
    friend class SavingLoading;

    /**
     * Used to fetch atoms from disk.
     */
    BackingStore *backing_store;
    void do_merge_tv(Handle, const TruthValue&);

public:
    // USED TO SEEK MEMORY LEAK
    //std::set<std::string> uniqueTimestamp;

    AtomSpace(void);
    ~AtomSpace();

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
     * @return a const reference to the AtomTable object of this AtomSpace
     */
    const AtomTable& getAtomTable() const;

    /**
     * @return a const reference to the TimeServer object of this AtomSpace
     */
    const TimeServer& getTimeServer() const;

    /**
     * XXX TODO eliminate this function
     * @return a reference to the SpaceServer object of this AtomSpace
     */
    SpaceServer& getSpaceServer() const;

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

    //helper for GDB, because print seems to mess things up
    void printGDB() const {
        print();
    }

    /**
     * Adds both the AtTime(TimeNode <timestamp>, atom) atom
     * representation into the AtomTable and the entry (atom,
     * timestamp) into the TimeServer of the given AtomSpace.
     *
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param timestamp The timestamp to be associated to the atom.
     * @param tv Truth value for the AtTimeLink created (optional)
     * @return the Handle of the AtTimeLink added into AtomSpace.
     */
    Handle addTimeInfo(Handle atom, unsigned long timestamp,
                       const TruthValue& tv = TruthValue::NULL_TV());

    /**
     * Adds both the AtTime(TimeNode <t>, atom) atom representation
     * into the AtomTable and the entry (atom, t) into the TimeServer
     * of the given AtomSpace.
     *
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param t The Temporal object to be associated to the atom.
     * @param tv Truth value for the AtTimeLink created (optional)
     * @return the Handle of the AtTimeLink added into AtomSpace.
     */
    Handle addTimeInfo(Handle atom, const Temporal& t,
                       const TruthValue& tv = TruthValue::NULL_TV());

    /**
     * Removes both the AtTime(TimeNode <timestamp>, atom) atom
     * representation from the AtomTable and the entry (atom,
     * timestamp) from the TimeServer of the given AtomSpace.
     *
     * NOTE1: All handles in the incoming set of the corresponding
     * AtTimeLink Atom will also be removed recursively (unless the
     * recursive argument is explicitely set to false).
     *
     * NOTE2: The TimeNode that corresponds to the given removed
     * time info is also removed if its incoming set becomes empty
     * after the removal of an AtTimeLink link (unless the
     * removeDisconnectedTimeNodes argument is explicitly set to false).
     *
     * @param atom the Handle of the atom to be associated to
     *        the timestamp. This argument cannot be an Handle::UNDEFINED.
     *        If it is, a RuntimeException is thrown.
     * @param timestamp The timestamp to be associated to the atom.
     *        This argument cannot be an UNDEFINED_TEMPORAL. If
     *        so, it throws a RuntimeException.
     * @param the Temporal relationship criteria to be used for
     *        this removal operation.
     *
     *        This method only removes the time info related to
     *        the HandleTemporalPair objects whose Temporal matches with
     *        this argument (search criteria) applied to the given
     *        timestamp argument.
     *
     *        The default temporal relationship is "exact match".
     *        See the definition of TemporalRelationship enumeration
     *        to see other possible values for it.
     * @param removeDisconnectedTimeNodes Flag to indicate if any
     *        TimeNode whose incoming set becomes empty after the
     *        removal of the AtTimeLink link must be removed.
     * @param recursive Flag to indicate if all atoms in the
     *        incoming set of the AtTimeLink link must be
     *        removed recursively.
     *
     * @return True if the matching pairs (Handle, Temporal) were
     *        successfully removed. False, otherwise (i.e., no
     *        mathing pair or any of them were not removed)
     */
    bool removeTimeInfo(Handle atom,
                        unsigned long timestamp,
                        TemporalTable::TemporalRelationship = TemporalTable::EXACT,
                        bool removeDisconnectedTimeNodes = true,
                        bool recursive = true);

    /**
     * Removes both the AtTime(TimeNode <t>, atom) atom representation
     * from the AtomTable and the entry (atom, t) from the TimeServer
     * of the given AtomSpace.
     *
     * NOTE1: All handles in the incoming set of the corresponding
     *        AtTimeLink Atom will also be removed recursively
     *        (unless the recursive argument is explicitely set to false).
     * NOTE2: The TimeNode that corresponds to the given removed
     *        time info is also removed if its incoming set becomes
     *        empty after the removal of an AtTimeLink link (unless the
     *        removeDisconnectedTimeNodes argument is explicitly set to
     *        false).
     *
     * @param atom the Handle of the atom to be associated to the
     *        timestamp. This argument cannot be an Handle::UNDEFINED.
     *        If so, it throws a RuntimeException.
     * @param t The Temporal object to be associated to the atom. This
     *        argument cannot be an UNDEFINED_TEMPORAL. If so, it throws
     *        a RuntimeException.
     * @param removeDisconnectedTimeNode Flag to indicate if the
     *        TimeNode that corresponds to the given timestamp should
     *        be removed, if its incoming set becomes empty after the
     *        removal of the AtTimeLink link.
     * @param the Temporal relationship criteria to be used for this
     *        removal operation, if the given Temporal object argument
     *        is not UNDEFINED_TEMPORAL. This method only removes the
     *        time info related to the HandleTemporalPair objects whose
     *        Temporal matches with this argument (search criteria)
     *        applied to the given Temporal object argument. The default
     *        temporal relationship is "exact match". See the definition
     *        of TemporalRelationship enumeration to see other possible
     *        values for it.
     * @param removeDisconnectedTimeNodes Flag to indicate if any
     *        TimeNode whose incoming set becomes empty after the removal
     *        of the AtTimeLink link must be removed.
     * @param recursive Flag to indicate if all atoms in the incoming set
     *        of the AtTimeLink link must be removed recursively.
     *
     * @return True if the matching pairs (Handle, Temporal) were
     *         successfully removed. False, otherwise (i.e., no mathing
     *         pair or any of them were not removed)
     */
    bool removeTimeInfo(Handle atom,
                        const Temporal& t = UNDEFINED_TEMPORAL,
                        TemporalTable::TemporalRelationship = TemporalTable::EXACT,
                        bool removeDisconnectedTimeNodes = true,
                        bool recursive = true);

    /**
     * Gets the corresponding AtTimeLink for the given HandleTemporalPair value
     * @param the pair (Handle, Temporal) that defines an AtTimeLink instance.
     * @return the Handle of the corresponding AtTimeLink, if it exists.
     */
    Handle getAtTimeLink(const HandleTemporalPair& htp) const;

    /**
     * Gets a list of HandleTemporalPair objects given an Atom Handle.
     *
     * \param outIt The outputIterator to
     * \param h The Atom Handle
     * \param t The temporal object
     * \param c The Temporal pair removal criterion
     *
     * \return An OutputIterator list
     *
     * NOTE: The matched entries are appended to a container whose
     *       OutputIterator is passed as the first argument. Example
     *       of call to this method, which would return all entries
     *       in TimeServer:
     *           std::list<HandleTemporalPair> ret;
     *           timeServer.get(back_inserter(ret), Handle::UNDEFINED);
     */
    template<typename OutputIterator> OutputIterator
    getTimeInfo(OutputIterator outIt,
                Handle h,
                const Temporal& t = UNDEFINED_TEMPORAL,
                TemporalTable::TemporalRelationship criterion = TemporalTable::EXACT) const
    {
        return timeServer.get(outIt, h, t, criterion);
    }

    /**
     * XXX TODO: Move this function out of here, and into its own
     * class, somewhere not in the atomspace dirctory. The SpaceServer
     * should be using the AtomTable addAtom, RemoveAtom() signals 
     * to acomplish its work.
     *
     * Adds space information about an object represented by a Node.
     * @param objectNode the Handle of the node that represents the object to be associated to the space info
     * @param timestamp The timestamp to be associated to this operation.
     * @param the remaining arguments are related to object's spatial information
     * @return true if any property of the object has changed (or it's a new object). False, otherwise.
     */
    bool addSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp,
                              double objX, double objY, double objZ,
                              double objLength, double objWidth, double objHeight,
                              double objYaw, bool isObstacle = true);
    /**
     * XXX TODO: Move this function out of here, and into its own
     * class, somewhere not in the atomspace dirctory. The SpaceServer
     * should be using the AtomTable addAtom, RemoveAtom() signals 
     * to acomplish its work.
     *
     * Add a whole space map into the SpaceServer.
     * NOTE: This is just used when a whole space map is received
     * from a remote SpaceServer (in LearningServer, for instance).
     */
    Handle addSpaceMap(unsigned long timestamp, SpaceServer::SpaceMap * spaceMap);

    /**
     * XXX TODO: Move this function out of here, and into its own
     * class, somewhere not in the atomspace dirctory. The SpaceServer
     * should be using the AtomTable addAtom, RemoveAtom() signals 
     * to acomplish its work.
     *
     * Removes space information about an object from the latest map (object is no longer at map's range)
     * @param objectNode the Handle of the node that represents the object to be removed from space map
     * @param timestamp The timestamp to be associated to this operation.
     * @return handle of the atom that represents the SpaceMap (at the given timestamp) where the object was removed
     */
    Handle removeSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp);

     /**
     * XXX TODO: Move this function out of here, and into its own
     * class, somewhere not in the atomspace dirctory. The SpaceServer
     * should be using the AtomTable addAtom, RemoveAtom() signals 
     * to acomplish its work.
     *
     * Gets all SpaceMap handles that would be needed inside the given interval.
     * For getting the SpaceMap of each handle returned,
     * use the spaceServer.getMap(Handle spaceMapHandle) method.
     * @param out  the output iterator where the resulting handles will be added.
     * @param startMoment the start of the time interval for searching the maps
     * @param endMoment the end of the time interval for searching the maps
     *
     * Example of usage:
     *     HandleSeq result;
     *     spaceServer.getMapHandles(back_inserter(result),start,end);
     *     foreach(Handle h, result) {
     *         const SpaceMap& map = spaceServer().getMap(h);
     *         ...
     *     }
     */

    template<typename OutputIterator>
    OutputIterator getMapHandles(   OutputIterator outIt,
                                    unsigned long startMoment, unsigned long endMoment) const {
        Temporal t(startMoment, endMoment);
        std::vector<HandleTemporalPair> pairs;
        Handle spaceMapNode = getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME);
        if (spaceMapNode != Handle::UNDEFINED) {
            // Gets the first map before the given interval, if any
            getTimeInfo(back_inserter(pairs), spaceMapNode, t, TemporalTable::PREVIOUS_BEFORE_START_OF);
            // Gets all maps inside the given interval, if any
            getTimeInfo(back_inserter(pairs), spaceMapNode, t, TemporalTable::STARTS_WITHIN);
            for(unsigned int i = 0; i < pairs.size(); i++) {
                HandleTemporalPair pair = pairs[i];
                *(outIt++) = getAtTimeLink(pair);
            }
        }
        return outIt;
    }

    /**
     * XXX TODO: Move this function out of here, and into its own
     * class, somewhere not in the atomspace dirctory. The SpaceServer
     * should be using the AtomTable addAtom, RemoveAtom() signals 
     * to acomplish its work.
     *
     * Remove old maps from SpaceServer in order to save memory. SpaceMaps
     * associated with exemplar sections, i.e., marked as persistent and the
     * latest (newest) space map are preserved.
     *
     * IMPORTANT: This function cannot be called while any trick exemplar is in progress.
     */
    void cleanupSpaceServer();


    /** Add a new node to the Atom Table,
    if the atom already exists then the old and the new truth value is merged
        \param t     Type of the node
        \param name  Name of the node
        \param tvn   Optional TruthValue of the node. If not provided, uses the DEFAULT_TV (see TruthValue.h) */
    Handle addNode(Type t, const std::string& name = "", const TruthValue& tvn = TruthValue::DEFAULT_TV());

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
                   const TruthValue& tvn = TruthValue::DEFAULT_TV());
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
     * DEPRECATED!
     *
     * Add an atom an optional TruthValue object to the Atom Table
     * This is a deprecated function; do not use it in new code,
     * if at all possible.
     *
     * @param the handle of the Atom to be added
     * @param the TruthValue object to be associated to the added
     *        atom. NULL if the own atom's tv must be used.
     * @deprecated This is a legacy code left-over from when one could
     * have non-real atoms, i.e. those whose handles were
     * less than 500, and indicated types, not atoms.
     * Instead of using that method, one should use
     * addNode or addLink should be used (which is a bit faster too).
     */
    Handle addRealAtom(const Atom& atom,
                       const TruthValue& tvn = TruthValue::NULL_TV());

    /**
     * Removes an atom from the atomspace
     *
     * @param The Handle of the atom to be removed.
     * @param Recursive-removal flag; if set, the links in the
     *        incoming set of the atom to be removed will also be
     *        removed.
     * @return True if the Atom for the given Handle was successfully
     *         removed. False, otherwise.
     *
     * When the atom is removed from the atomspace, all memory associated
     * with it is also deleted; in particular, the atom is removed from
     * the TLB as well, so that future TLB lookups will now return pointers
     * to freed memory.
     */
    bool removeAtom(Handle h, bool recursive = false);

    /**
     * Retrieve from the Atom Table the Handle of a given node
     *
     * @param t     Type of the node
     * @param str   Name of the node
    */
    Handle getHandle(Type t, const std::string& str) const {
        return atomTable.getHandle(str.c_str(), t);
    }

    /**
     * Retrieve from the Atom Table the Handle of a given link
     * @param t        Type of the node
     * @param outgoing a reference to a HandleSeq containing
     *        the outgoing set of the link.
    */
    Handle getHandle(Type t, const HandleSeq& outgoing) const {
        return atomTable.getHandle(t, outgoing);
    }

    /** Retrieve the name of a given Handle */
    const std::string& getName(Handle) const;

    /** Change the name of a given Handle */
    void setName(Handle, const std::string& name);

    /** Retrieve the type of a given Handle */
    Type getType(Handle) const;

    /** Retrieve the TruthValue of a given Handle */
    const TruthValue& getTV(Handle, VersionHandle = NULL_VERSION_HANDLE) const;

    /** Change the TruthValue of a given Handle */
    void setTV(Handle, const TruthValue&, VersionHandle = NULL_VERSION_HANDLE);

    /** Change the primary TV's mean of a given Handle */
    void setMean(Handle, float mean) throw (InvalidParamException);

    /** Retrieve the AttentionValue of an attention value holder */
    const AttentionValue& getAV(AttentionValueHolder *avh) const;

    /** Change the AttentionValue of an attention value holder */
    void setAV(AttentionValueHolder *avh, const AttentionValue &av);

    /** Change the Short-Term Importance of an attention value holder */
    void setSTI(AttentionValueHolder *avh, AttentionValue::sti_t);

    /** Change the Long-term Importance of an attention value holder */
    void setLTI(AttentionValueHolder *avh, AttentionValue::lti_t);

    /** Change the Very-Long-Term Importance of an attention value holder */
    void setVLTI(AttentionValueHolder *avh, AttentionValue::vlti_t);

    /** Retrieve the Short-Term Importance of an attention value holder */
    AttentionValue::sti_t getSTI(AttentionValueHolder *avh) const;

    /** Retrieve the doubly normalised Short-Term Importance between -1..1
     * for a given AttentionValueHolder. STI above and below threshold
     * normalised separately and linearly.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to -1..1? Outside this
     * range can be return if average=true
     * @return normalised STI between -1..1
     */
    float getNormalisedSTI(AttentionValueHolder *avh, bool average=true, bool clip=false) const;

    /** Retrieve the linearly normalised Short-Term Importance between 0..1
     * for a given AttentionValueHolder.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to 0..1? Outside this
     * range can be return if average=true
     * @return normalised STI between 0..1
     */
    float getNormalisedZeroToOneSTI(AttentionValueHolder *avh, bool average=true, bool clip=false) const;

    /** Retrieve the Long-term Importance of a given AttentionValueHolder */
    AttentionValue::lti_t getLTI(AttentionValueHolder *avh) const;

    /** Retrieve the Very-Long-Term Importance of a given
     * AttentionValueHolder */
    AttentionValue::vlti_t getVLTI(AttentionValueHolder *avh) const;

    /** Retrieve the AttentionValue of a given Handle */
    const AttentionValue& getAV(Handle h) const {
        return getAV(TLB::getAtom(h));
    }

    /** Change the AttentionValue of a given Handle */
    void setAV(Handle h, const AttentionValue &av) {
        setAV(TLB::getAtom(h), av);
    }

    /** Change the Short-Term Importance of a given Handle */
    void setSTI(Handle h, AttentionValue::sti_t stiValue) {
        setSTI(TLB::getAtom(h), stiValue);
    }

    /** Change the Long-term Importance of a given Handle */
    void setLTI(Handle h, AttentionValue::lti_t ltiValue) {
        setLTI(TLB::getAtom(h), ltiValue);
    }

    /** Change the Very-Long-Term Importance of a given Handle */
    void setVLTI(Handle h, AttentionValue::vlti_t vltiValue) {
        setVLTI(TLB::getAtom(h), vltiValue);
    }

    /** Retrieve the Short-Term Importance of a given Handle */
    AttentionValue::sti_t getSTI(Handle h) const {
        return getSTI(TLB::getAtom(h));
    }

    /** Retrieve the doubly normalised Short-Term Importance between -1..1
     * for a given Handle. STI above and below threshold normalised separately
     * and linearly.
     *
     * @param h The atom handle to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to -1..1? Outside this
     * range can be return if average=true
     * @return normalised STI between -1..1
     */
    float getNormalisedSTI(Handle h, bool average=true, bool clip=false) const {
        return getNormalisedSTI(TLB::getAtom(h), average, clip);
    }

    /** Retrieve the linearly normalised Short-Term Importance between 0..1
     * for a given Handle.
     *
     * @param h The atom handle to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to 0..1? Outside this
     * range can be return if average=true
     * @return normalised STI between 0..1
     */
    float getNormalisedZeroToOneSTI(Handle h, bool average=true, bool clip=false) const {
        return getNormalisedZeroToOneSTI(TLB::getAtom(h), average, clip);
    }

    /** Retrieve the Long-term Importance of a given Handle */
    AttentionValue::lti_t getLTI(Handle h) const {
        return getLTI(TLB::getAtom(h));
    }

    /** Retrieve the Very-Long-Term Importance of a given Handle */
    AttentionValue::vlti_t getVLTI(Handle h) const {
        return getVLTI(TLB::getAtom(h));
    }

    /** Retrieve a single Handle from the outgoing set of a given link */
    Handle getOutgoing(Handle, int idx) const;

    /** Retrieve the arity of a given link */
    int getArity(Handle) const;

    /** Retrieve the outgoing set of a given link */
    const HandleSeq& getOutgoing(Handle h) const;

    /** Retrieve the incoming set of a given atom */
    HandleSeq getIncoming(Handle);

    /** Retrieve the Count of a given Handle */
    float getCount(Handle) const;

    /** Returns the default TruthValue */
    static const TruthValue& getDefaultTV();

    //type properties
    Type getAtomType(const std::string& typeName) const;
    bool isNode(Type t) const;
    /** Does t1 inherit from t2 */
    bool inheritsType(Type t1, Type t2) const;
    std::string getName(Type t) const;

    /**
     * Gets a set of handles that matches with the given arguments.
     *
     * @param An output iterator.
     * @param the type of the atoms to be searched
     * @param the name of the atoms to be searched.
     *        For searching only links, use "" or a search by type.
     * @param if subTypes of the given type are accepted in this search
     * @param if returns only atoms that contains versioned TVS with
     *        the given VersionHandle. If NULL_VERSION_HANDLE is given,
     *        it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * NOTE: The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument. Example of a
     * call to this method, which would return all entries in TimeServer:
     *
     *      std::list<Handle> ret;
     *      atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 const std::string& name,
                 bool subclass = true,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        HandleEntry * handleEntry = atomTable.getHandleSet(name.c_str(), type, subclass, vh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally).
     *
     * @param An output iterator.
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 bool subclass = false,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        HandleEntry * handleEntry = atomTable.getHandleSet(type, subclass, vh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Returns the set of atoms of a given type which have atoms of a
     * given target type in their outgoing set (subclasses optionally).
     *
     * @param An output iterator.
     * @param The desired type.
     * @param The desired target type.
     * @param Whether type subclasses should be considered.
     * @param Whether target type subclasses should be considered.
      * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of a given type and target type (subclasses
     * optionally).
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type type,
                 Type targetType,
                 bool subclass,
                 bool targetSubclass,
                 VersionHandle vh = NULL_VERSION_HANDLE,
                 VersionHandle targetVh = NULL_VERSION_HANDLE) const {

        HandleEntry * handleEntry = atomTable.getHandleSet(type, targetType, subclass, targetSubclass, vh, targetVh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Returns the set of atoms with a given target handle in their
     * outgoing set (atom type and its subclasses optionally).
     *
     * @param An output iterator.
     * @param The handle that must be in the outgoing set of the atom.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the given handle in
     * their outgoing set.
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Handle handle,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        HandleEntry * handleEntry = atomTable.getHandleSet(handle, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Returns the set of atoms with the given target handles and types
     * (order is considered) in their outgoing sets, where the type and
     * subclasses of the atoms are optional.
     *
     * @param An output iterator.
     * @param An array of handles to match the outgoing sets of the searched
     * atoms. This array can be empty (or each of its elements can be null), if
     * the handle value does not matter or if it does not apply to the
     * specific search.
     * Note that if this array is not empty, it must contain "arity" elements.
     * @param An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms.
     * @param An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Note that if this
     * array is not null, it must contains "arity" elements.
     * @param The length of the outgoing set of the atoms being searched.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * @note The matched entries are appended to a container whose OutputIterator
     * is passed as the first argument. Example of call to this method, which
     * would return all entries in AtomSpace:
     *     std::list<Handle> ret;
     *     atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
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

        HandleEntry * handleEntry = atomTable.getHandleSet(handles, types,
                subclasses, arity, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Returns the set of atoms of a given name (atom type and subclasses
     * optionally).
     *
     * @param An output iterator.
     * @param The desired name of the atoms.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type and name.
     *
     * NOTE: The matched entries are appended to a container whose
     * OutputIterator is passed as the first argument.
     *
     * Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const char* name,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        HandleEntry * handleEntry = atomTable.getHandleSet(name, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Returns the set of atoms whose outgoing set contains at least one
     * atom with the given name and type (atom type and subclasses
     * optionally).
     *
     * @param An output iterator.
     * @param The name of the atom in the outgoing set of the searched
     * atoms.
     * @param The type of the atom in the outgoing set of the searched
     * atoms.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type and name whose outgoing
     * set contains at least one atom of the given type and name.
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 const char* targetName,
                 Type targetType,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE,
                 VersionHandle targetVh = NULL_VERSION_HANDLE) const {

        HandleEntry * handleEntry = atomTable.getHandleSet(targetName, targetType, type, subclass, vh, targetVh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param An output iterator.
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
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
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

        HandleEntry * handleEntry = atomTable.getHandleSet(names, types, subclasses, arity, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Returns the set of atoms with the given target names and/or types
     * (order is considered) in their outgoing sets, where the type
     * and subclasses arguments of the searched atoms are optional.
     *
     * @param An output iterator.
     * @param An array of target types to match the types of the atoms in
     * the outgoing set of searched atoms. This parameter can be null (or any of
     * its elements can be NOTYPE), what means that the type doesnt matter.
     * Not that if this array is not null, it must contains "arity" elements.
     * @param An array of boolean values indicating whether each of the
     * above types must also consider subclasses. This array can be null,
     * what means that subclasses will not be considered. Not that if this
     * array is not null, it must contains "arity" elements.
     * @param The length of the outgoing set of the atoms being searched.
     * @param The optional type of the atom.
     * @param Whether atom type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     * @return The set of atoms of the given type with the matching
     * criteria in their outgoing set.
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSet(OutputIterator result,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        HandleEntry * handleEntry = atomTable.getHandleSet(types, subclasses, arity, type, subclass, vh);
        return (toOutputIterator(result, handleEntry));
    }

    /**
     * Gets a set of handles in the Attentional Focus that matches with the given type
     * (subclasses optionally).
     *
     * @param An output iterator.
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer in the AttentionalFocus:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     */
    template <typename OutputIterator> OutputIterator
    getHandleSetInAttentionalFocus(OutputIterator result,
                 Type type,
                 bool subclass,
                 VersionHandle vh = NULL_VERSION_HANDLE) const
    {
        //return getHandleSet(result, type, subclass, InAttentionalFocus(), vh);
        return getHandleSetFiltered(result, type, subclass, STIAboveThreshold(getAttentionalFocusBoundary()), vh);

    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally) and a given criterion.
     *
     * @param An output iterator.
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @param A criterion for including atoms. It must be something that returns a bool when called.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer beyond 500 LTI:
     *         std::list<Handle> ret;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true, LTIAboveThreshold(500));
     */
    template <typename OutputIterator, typename Predicate> OutputIterator
    getHandleSetFiltered(OutputIterator result,
                 Type type,
                 bool subclass,
                 Predicate compare,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {

        HandleEntry * handleEntry = atomTable.getHandleSet(type, subclass, vh);
        std::vector<Handle> hs;
        // these two lines could be replaced using a function that filters
        // a handleEntry list into an arbitrary sequence
        toOutputIterator(back_inserter(hs), handleEntry);
        return filter(hs.begin(), hs.end(), result, compare);
    }

    /**
     * Gets a set of handles that matches with the given type
     * (subclasses optionally), sorted according to the given comparison
     * structure.
     *
     * @param An output iterator.
     * @param The desired type.
     * @param Whether type subclasses should be considered.
     * @param The comparison struct to use in the sort.
     * @param if returns only atoms that contains versioned TVS with the given VersionHandle.
     *        If NULL_VERSION_HANDLE is given, it does not restrict the result.
     *
     * @return The set of atoms of a given type (subclasses optionally).
     *
     * NOTE: The matched entries are appended to a container whose OutputIterator is passed as the first argument.
     *          Example of call to this method, which would return all entries in TimeServer, sorted by STI:
     *         std::list<Handle> ret;
     *         AttentionValue::STISort stiSort;
     *         atomSpace.getHandleSet(back_inserter(ret), ATOM, true, stiSort);
     */
    template <typename OutputIterator, typename Compare> OutputIterator
    getSortedHandleSet(OutputIterator result,
                 Type type,
                 bool subclass,
                 Compare compare,
                 VersionHandle vh = NULL_VERSION_HANDLE) const {
        // get the handle set as a vector and sort it.
        std::vector<Handle> hs;

        getHandleSet(back_inserter(hs), type, subclass, vh);
        sort(hs.begin(), hs.end(), compare);

        // copy the vector and return the iterator.
        return copy(hs.begin(), hs.end(), result);
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
     * Decays STI of all atoms (one cycle of importance decay).
     * Deprecated, importance updating should be done by ImportanceUpdating
     * Agent. Still used by Embodiment.
     */
    void decayShortTermImportance();

    /**
     * Get the total amount of STI in the AtomSpace, sum of
     * STI across all atoms.
     *
     * @return total STI in AtomSpace
     */
    long getTotalSTI() const;

    /**
     * Get the total amount of LTI in the AtomSpace, sum of
     * all LTI across atoms.
     *
     * @return total LTI in AtomSpace
     */
    long getTotalLTI() const;

    /**
     * Get the STI funds available in the AtomSpace pool.
     *
     * @return STI funds available
     */
    long getSTIFunds() const;

    /**
     * Get the LTI funds available in the AtomSpace pool.
     *
     * @return LTI funds available
     */
    long getLTIFunds() const;

    /**
     * Get attentional focus boundary, generally atoms below
     * this threshold won't be accessed unless search methods
     * are unsuccessful on those that are above this value.
     *
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t getAttentionalFocusBoundary() const;

    /**
     * Change the attentional focus boundary. Some situations
     * may benefit from less focussed searches.
     *
     * @param s New threshold
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t setAttentionalFocusBoundary(
        AttentionValue::sti_t s);

    /**
     * Get the maximum STI observed in the AtomSpace.
     *
     * @param average If true, return an exponentially decaying average of
     * maximum STI, otherwise return the actual maximum.
     * @return Maximum STI
     */
    AttentionValue::sti_t getMaxSTI(bool average=true) const;

    /**
     * Get the minimum STI observed in the AtomSpace.
     *
     * @param average If true, return an exponentially decaying average of
     * minimum STI, otherwise return the actual maximum.
     * @return Minimum STI
     */
    AttentionValue::sti_t getMinSTI(bool average=true) const;

    /**
     * Update the minimum STI observed in the AtomSpace. Min/max are not updated
     * on setSTI because average is calculate by lobe cycle, although this could
     * potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New minimum STI
     */
    void updateMinSTI(AttentionValue::sti_t m);

    /**
     * Update the maximum STI observed in the AtomSpace. Min/max are not updated
     * on setSTI because average is calculate by lobe cycle, although this could
     * potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New maximum STI
     */
    void updateMaxSTI(AttentionValue::sti_t m);

    // For convenience
    // bool isNode(Handle) const;
    bool isVar(Handle) const;
    bool isList(Handle) const;
    bool containsVar(Handle) const;

    Handle createHandle(Type t, const std::string& str, bool managed = false);
    Handle createHandle(Type t, const HandleSeq& outgoing, bool managed = false);

    int Nodes(VersionHandle = NULL_VERSION_HANDLE) const;
    int Links(VersionHandle = NULL_VERSION_HANDLE) const;

    bool containsVersionedTV(Handle h, VersionHandle vh) const;

    //! Clear the atomspace, remove all atoms
    void clear();

// ---- filter templates

    template<typename Predicate>
    HandleSeq filter(Predicate compare, VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeq result;
        _getNextAtomPrepare();
        Handle next;
        while ((next = _getNextAtom()) != Handle::UNDEFINED)
            if (compare(next) && containsVersionedTV(next, vh))
                result.push_back(next);
        return result;
    }

    template<typename OutputIterator, typename Predicate>
    OutputIterator filter(OutputIterator it, Predicate compare, VersionHandle vh = NULL_VERSION_HANDLE) {
        _getNextAtomPrepare();
        Handle next;
        while ((next = _getNextAtom()) != Handle::UNDEFINED)
            if (compare(next) && containsVersionedTV(next, vh))
                * it++ = next;
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
    template<typename Predicate, typename InputIterator>
    HandleSeq filter(InputIterator begin, InputIterator end, Predicate compare) const {
        HandleSeq result;
        for (; begin != end; begin++)
            if (compare(*begin))
                result.push_back(*begin);

        return result;
    }

    template<typename InputIterator, typename OutputIterator, typename Predicate>
    OutputIterator filter(InputIterator begin, InputIterator end, OutputIterator it, Predicate compare) const {
        for (; begin != end; begin++)
            if (compare(*begin))
                * it++ = *begin;

        return it;
    }


// ---- custom filter templates

    HandleSeq filter_type(Type type, VersionHandle vh = NULL_VERSION_HANDLE) {
        HandleSeq result;
        _getNextAtomPrepare_type(type);
        Handle next;
        while ((next = _getNextAtom_type(type)) != Handle::UNDEFINED)
            if (containsVersionedTV(next, vh))
                result.push_back(next);

        return result;
    }

    template<typename OutputIterator>
    OutputIterator filter_type(OutputIterator it, Type type, VersionHandle vh = NULL_VERSION_HANDLE) {
        _getNextAtomPrepare_type(type);
        Handle next;
        while ((next = _getNextAtom_type(type)) != Handle::UNDEFINED)
            if (containsVersionedTV(next, vh))
                * it++ = next;

        return it;
    }

    template<typename InputIterator>
    HandleSeq filter_InAttentionalFocus(InputIterator begin, InputIterator end) const {
        return filter(begin, end, STIAboveThreshold(getAttentionalFocusBoundary()));
    }

    struct STIAboveThreshold {
        STIAboveThreshold(const AttentionValue::sti_t t) : threshold (t) {}

        bool operator()(const Handle& h) {
            return TLB::getAtom(h)->getAttentionValue().getSTI() > threshold;
        }
        AttentionValue::sti_t threshold;
    };

    struct LTIAboveThreshold {
        LTIAboveThreshold(const AttentionValue::lti_t t) : threshold (t) {}

        bool operator()(const Handle& h) {
            return TLB::getAtom(h)->getAttentionValue().getLTI() > threshold;
        }
        AttentionValue::lti_t threshold;
    };

protected:

    HandleIterator* _handle_iterator;
    TypeIndex::iterator type_itr;
    // these methods are used by the filter_* templates
    void _getNextAtomPrepare();
    Handle _getNextAtom();
    void _getNextAtomPrepare_type(Type type);
    Handle _getNextAtom_type(Type type);

private:

    TimeServer timeServer;
    AtomTable atomTable;
    std::string emptyName;
    SpaceServer* spaceServer;

    /**
     * signal connections used to keep track of atom removal in the AtomTable
     */
    boost::signals::connection removedAtomConnection; 
    boost::signals::connection addedAtomConnection; 

    /* Boundary at which an atom is considered within the attentional
     * focus of opencog. Atom's with STI less than this value are
     * not charged STI rent */
    AttentionValue::sti_t attentionalFocusBoundary;

    opencog::recent_val<AttentionValue::sti_t> maxSTI;
    opencog::recent_val<AttentionValue::sti_t> minSTI;

    /* These indicate the amount importance funds available in the
     * AtomSpace */
    long fundsSTI;
    long fundsLTI;

    /*
     * Remove stimulus from atom, only should be used when Atom is deleted.
     */
    void removeStimulus(Handle h);

    template <typename OutputIterator> OutputIterator
    toOutputIterator(OutputIterator result, HandleEntry * handleEntry) const {

        HandleEntry * toRemove = handleEntry;
        while (handleEntry) {
            *(result++) = handleEntry->handle;
            handleEntry = handleEntry->next;
        }
        // free memory
        if (toRemove) delete toRemove;
        return result;
    }

    /*
     * Adds both the AtTime(TimeNode <timeNodeName>, atom) atom representation into the AtomTable and the
     * corresponding entry (atom, t) into the TimeServer of the given AtomSpace.
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param timeNodeName the name of the TimeNode to be associated to the atom via an AtTimeLink.
     * @param tv Truth value for the AtTimeLink created (optional)
     * @return the Handle of the AtTimeLink added into the AtomSpace.
     */
    Handle addTimeInfo(Handle h, const std::string& timeNodeName, const TruthValue& tv = TruthValue::NULL_TV());

    /**
     * Creates the space map node, if not created yet.
     * returns the handle of the node.
     */
    Handle getSpaceMapNode(void);

    /**
     * Handler of the 'atom removed' signal from AtomTable
     */
    void atomRemoved(Handle h);

    /**
     * Handler of the 'atom added' signal from AtomTable
     */
    void atomAdded(Handle h);

public:
    // pass on the signals from the Atom Table
    boost::signal<void (Handle)>& addAtomSignal()
        { return atomTable.addAtomSignal(); }
    boost::signal<void (Handle)>& removeAtomSignal()
        { return atomTable.removeAtomSignal(); }
    boost::signal<void (Handle)>& mergeAtomSignal()
        { return atomTable.mergeAtomSignal(); }

    // SpaceServerContainer virtual methods:
    void mapRemoved(Handle mapId);
    void mapPersisted(Handle mapId);
    std::string getMapIdString(Handle mapId);

    /**
     * Overrides and declares copy constructor and equals operator as private 
     * for avoiding large object copying by mistake.
     */
    AtomSpace& operator=(const AtomSpace&);
    AtomSpace(const AtomSpace&);

};

} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_H
