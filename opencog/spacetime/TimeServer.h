/*
 * opencog/spacetime/TimeServer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _OPENCOG_TIME_SERVER_H
#define _OPENCOG_TIME_SERVER_H

#include <mutex>
#include <set>
#include <map>
#include <string>
#include <boost/signals2.hpp>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/TemporalTable.h>

// TODO: Wrap this up in a module and add a scheme binding with options of
// indexing with/without concurrent entry in the atomspace.
// * Why not enter an atom's timestamp in the atomspace by using AtTimeLink?
// -> There maybe cases where the timestamp could be entered in the atomsapce
//    but for a very large data set we maynot need that, and one can retreive
//    that data from TemporalTable.
// * How to keep the handle in multiple copy of atomspaces?

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

/**
 * Add a TimeDomain inside the TimeServer
 * For example, in the game embodiment we may want to
 * save the both time in the game world and in the real system.
 * ex. in Minecraft we need the game world time to judge if the monster will appear
 * but we also need real system time to do calculation since the tick rate of Minecraft is slow(once per sec)
 * For this we add a time domain to mark what time domain the timestamp belongs to.
 * and we use a map<TimeDomain, TemporalTable> to save all corresponding tables
 * Also, we change the AtTimeLink structure to:
 * AtTimeLink
 * __TimeNode "12:00:00"
 * __ConceptNode "Lunch"
 * __TimeDomainNode "GameWorld"
 * by adding a TimeDomainNode in the outgoings.
 * If we don't set our own TimeDomain when adding time info, TimeServer will set a default time domain.
 * And the AtTimeLink will be 
 * AtTimeLink
 * __TimeNode "12:00:00"
 * __ConceptNode "Lunch"
 * skip the TimeDomainNode.
 */
typedef std::string TimeDomain;
extern TimeDomain DEFAULT_TIMEDOMAIN;
class TimeServerSavable;

/**
 * This class is used to associate temporal information (timestamps or timelags) to
 * atom handles.
 * All information added to this object should also be present in the
 * corresponding AtomSpace hypergraph in the following way:
 *     AtTimeLink ( TimeNode "<timestamp or timelag>", Handle )
 *
 * See also http://www.opencog.org/wiki/TimeServer
 */
class TimeServer
{
    friend class TimeServerSavable;

    /**
     * Initializes the TimeServer
     */
    void init();

    AtomSpace* atomspace;
    SpaceServer* spaceServer;

    // TS-wide mutex, since add/remove atom signals are called from AtomSpace event-loop
    mutable std::mutex ts_mutex;

public:


    // USED TO SEEK MEMORY LEAK
    //static int timeServerEntries;
    //static std::set<Temporal> temporalSet;

    // TODO: Why does the it need SpaceServer? Add constructor that takes
    // AtomSpace reference.
    TimeServer(AtomSpace& a, SpaceServer* ss);

    // A temporary hack for working with OpenPsi, because it doesn't need
    // a SpaceTimeCogserver
    TimeServer(AtomSpace& a);
    virtual ~TimeServer();

    /**
     * Adds into this TimeServer an entry composed by the given Atom Handle and Temporal object 
     * in the given time domain.
     */
    void add(Handle h, const Temporal& t, const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);

    /**
     * Gets a list of HandleTemporalPair objects given an Atom Handle and a time domain.
     * If the passed Handle object is Handle::UNDEFINED, it matches any Handle.
     * If the optional Temporal object argument is not UNDEFINED_TEMPORAL, it will be used
     * to restrict the return to only HandleTemporalPair objects whose Temporal
     * matches with it, according to the temporal relationship (search criteria) argument to
     * be applied with this given Temporal argument.
     * See the definition of TemporalRelationship enumeration to see the possible values for it.
     *
     * NOTE: The matched entries are appended to a container 
     * whose OutputIterator is passed as the first argument.
     * Example of call to this method, which return all entries associated to time domain in TimeServer:
     *         std::list<HandleTemporalPair> ret;
     *         timeServer->get(back_inserter(ret), Handle::UNDEFINED, "game world");
     */
    template<typename OutputIterator> OutputIterator
        get(OutputIterator outIt, Handle h,
            const Temporal& t = UNDEFINED_TEMPORAL,
            TemporalTable::TemporalRelationship criterion = TemporalTable::EXACT,
            const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN) const {

        auto temporalTableIter = temporalTableMap.find(timeDomain);        
        if (temporalTableIter == temporalTableMap.end()) {
            return outIt;            
        }

        std::unique_lock<std::mutex> lock(ts_mutex);
        HandleTemporalPairEntry* hte = (temporalTableIter->second).get(h, t, criterion);
        HandleTemporalPairEntry* toRemove = hte;

        while (hte) {
            *(outIt++) = hte->handleTemporalPair;
            hte = hte->next;
        }
        if (toRemove) delete toRemove;

        return outIt;
    }

    /**
     * Removes HandleTemporalPair objects related to a given Atom Handle and given time domain.
     * If the passed Handle object is Handle::UNDEFINED, it matches any Handle.
     * If the optional Temporal object argument is not UNDEFINED_TEMPORAL, it will be used
     * to restrict the removal to only HandleTemporalPair objects whose Temporal
     * matches with it, according to the temporal relationship (search criteria) argument to
     * be applied with this given Temporal argument.
     * See the definition of TemporalRelationship enumeration to see the possible values for it.
     * @return True if any entry corresponding to the given arguments was removed. False, otherwise.
     */
    bool remove(Handle, 
                const Temporal& = UNDEFINED_TEMPORAL, 
                TemporalTable::TemporalRelationship = TemporalTable::EXACT,
                const TimeDomain& = DEFAULT_TIMEDOMAIN);

    /**
     * Get the timestamp of the more recent upper bound of Temporal object 
     * already inserted into this TimeServer.
     */
    octime_t getLatestTimestamp() const;

    void clear();

    /**
     * Adds both the AtTime(TimeNode <timestamp>, atom) atom representation into the AtomTable
     * and the entry (atom, * timestamp) into the TimeServer of the given AtomSpace.
     *
     * @param atom       the Handle of the atom to be associated to the timestamp
     * @param timeDomain the time domain which the timestamp belongs to
     * @param timestamp  The timestamp to be associated to the atom.
     * @param tv         Truth value for the AtTimeLink created (optional)
     *
     * @return the Handle of the AtTimeLink added into AtomSpace.
     */
    Handle addTimeInfo(Handle atom,
		       const octime_t& timestamp,
                       const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN,
                       TruthValuePtr tv = TruthValue::TRUE_TV());

    /**
     * Adds both the AtTime(TimeNode <t>, atom) atom representation
     * into the AtomTable and the entry (atom, t) into the TimeServer
     * of the given AtomSpace.
     *
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param timeDomain the timedomian which the timestamp belongs to
     * @param t The Temporal object to be associated to the atom.
     * @param tv Truth value for the AtTimeLink created (optional)
     * @return the Handle of the AtTimeLink added into AtomSpace.
     */
    Handle addTimeInfo(Handle atom, 
		       const Temporal& t,
                       const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN,
                       TruthValuePtr tv = TruthValue::TRUE_TV());

    /**
     * Adds both the AtTime(TimeNode <timeNodeName>, atom) atom representation into the AtomTable 
     * and the corresponding entry (atom, t) into the TimeServer of the given AtomSpace.
     * @param atom the Handle of the atom to be associated to the timestamp
     * @param timeNodeName the name of the TimeNode to be associated to the atom via an AtTimeLink.
     * @param timeDomain the time domain which the timestamp belongs to
     * @param tv Truth value for the AtTimeLink created (optional)
     * @return the Handle of the AtTimeLink added into the AtomSpace.
     */
    Handle addTimeInfo(Handle h,
		       const std::string& timeNodeName,
                       const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN, 
		       TruthValuePtr tv = TruthValue::TRUE_TV());

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
     * @param h the Handle of the atom to be associated to
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
     * @param timeDomain the time domain which the timestamp belongs to
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
    bool removeTimeInfo(Handle h,
                        const octime_t& timestamp,
                        TemporalTable::TemporalRelationship = TemporalTable::EXACT,
                        const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN,
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
     * @param h the Handle of the atom to be associated to the
     *        timestamp. This argument cannot be an Handle::UNDEFINED.
     *        If so, it throws a RuntimeException.
     * @param t The Temporal object to be associated to the atom. This
     *        argument cannot be an UNDEFINED_TEMPORAL. If so, it throws
     *        a RuntimeException.
     * @param the Temporal relationship criteria to be used for this
     *        removal operation, if the given Temporal object argument
     *        is not UNDEFINED_TEMPORAL. This method only removes the
     *        time info related to the HandleTemporalPair objects whose
     *        Temporal matches with this argument (search criteria)
     *        applied to the given Temporal object argument. The default
     *        temporal relationship is "exact match". See the definition
     *        of TemporalRelationship enumeration to see other possible
     *        values for it.
     * @param timeDomain the timedomain which temporal t belongs to
     * @param removeDisconnectedTimeNode Flag to indicate if the
     *        TimeNode that corresponds to the given timestamp should
     *        be removed, if its incoming set becomes empty after the
     *        removal of the AtTimeLink link.
     * @param recursive Flag to indicate if all atoms in the incoming set
     *        of the AtTimeLink link must be removed recursively.
     *
     * @return True if the matching pairs (Handle, Temporal) were
     *         successfully removed. False, otherwise (i.e., no mathing
     *         pair or any of them were not removed)
     */

    bool removeTimeInfo(Handle h,
                        const Temporal& t = UNDEFINED_TEMPORAL,
                        TemporalTable::TemporalRelationship = TemporalTable::EXACT,
			const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN,
                        bool removeDisconnectedTimeNodes = true,
                        bool recursive = true);

    /**
     * Gets the corresponding AtTimeLink for the given HandleTemporalPair value
     * @param the pair (Handle, Temporal) that defines an AtTimeLink instance.
     * @param timeDomain the time domain htp belongs to
     * @return the Handle of the corresponding AtTimeLink, if it exists.
     */
    Handle getAtTimeLink(const HandleTemporalPair& htp, 
                         const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN) const;

    /**
     * Gets a list of HandleTemporalPair objects given an Atom Handle.
     *
     * @param outIt The outputIterator to
     * @param h The Atom Handle
     * @param t The temporal object
     * @param c The Temporal pair removal criterion
     * @param timeDomain The time domain which the temporal object belongs to
     *
     * @return An OutputIterator list
     *
     * @note The matched entries are appended to a container whose
     *       OutputIterator is passed as the first argument. Example
     *       of call to this method, which would return all entries
     *       in TimeServer:
     *           std::list<HandleTemporalPair> ret;
     *           timeServer.get(back_inserter(ret), Handle::UNDEFINED);
     *
     * @todo This is redundant! Why not just use get()?
     */
    template<typename OutputIterator> OutputIterator
		getTimeInfo(OutputIterator outIt,
			    Handle h,
			    const Temporal& t = UNDEFINED_TEMPORAL,
			    TemporalTable::TemporalRelationship criterion = TemporalTable::EXACT,
                            const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN) const
    {
        return get(outIt, h, t, criterion, timeDomain);
    }

    /**
     * Gets all SpaceMap handles that would be needed inside the given interval.
     * For getting the SpaceMap of each handle returned,
     * use the spaceServer.getMap(Handle spaceMapHandle) method.
     * @param out  the output iterator where the resulting handles will be added.
     * @param startMoment the start of the time interval for searching the maps
     * @param endMoment the end of the time interval for searching the maps
     * @param timeDomain the time domain which the time interval belongs to
     *
     * Example of usage:
     *     HandleSeq result;
     *     TimeServer.getMapHandles(back_inserter(result),"utc-8",start,end);
     *     for (Handle h : result) {
     *         const SpaceMap& map = spaceServer().getMap(h);
     *         ...
     *     }
     */
    template<typename OutputIterator>
		OutputIterator getMapHandles( OutputIterator outIt, 
                                              octime_t startMoment, 
                                              octime_t endMoment,
                                              const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN) const
    {
	Temporal t(startMoment, endMoment);
	std::vector<HandleTemporalPair> pairs;
        Handle spaceMapNode = spaceServer->getLatestMapHandle();
        if (spaceMapNode != Handle::UNDEFINED) {
	    // Gets the first map before the given interval, if any
            getTimeInfo(back_inserter(pairs), spaceMapNode, t, TemporalTable::PREVIOUS_BEFORE_START_OF, timeDomain);
            // Gets all maps inside the given interval, if any
            getTimeInfo(back_inserter(pairs), spaceMapNode, t, TemporalTable::STARTS_WITHIN, timeDomain);
            for(unsigned int i = 0; i < pairs.size(); i++) {
		HandleTemporalPair pair = pairs[i];
                *(outIt++) = getAtTimeLink(pair, timeDomain);
            }
        }
        return outIt;
    }

    //For the old code which assume there's only one time domain in the system.
    //TimeDomain getTimeDomain() const;
    vector<TimeDomain> getTimeDomains() const;
    bool existTimeDomain(const TimeDomain timeDomain) const
    {
        return temporalTableMap.find(timeDomain) != temporalTableMap.end();
    }

private:

    /**
     * signal connections used to keep track of atom removal in the SpaceMap
     */
    boost::signals2::connection removedAtomConnection;
    boost::signals2::connection addedAtomConnection;

    void atomAdded(const Handle&);
    void atomRemoved(const AtomPtr&);

    /**
     * The temporal table used by this TimeServer
     */
    map<TimeDomain, TemporalTable> temporalTableMap;

    /**
     * The timestamp of the most recent upper bound of Temporal object already inserted into TimeServer.
     */
    octime_t latestTimestamp;

    /**
     * Overrides and declares copy constructor and equals operator as private
     * for avoiding large object copying by mistake.
     */
    TimeServer& operator=(const TimeServer&);
    TimeServer(const TimeServer&);

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_TIME_SERVER_H
