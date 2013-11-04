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

    /**
     * Decays STI of all atoms (one cycle of importance decay).
     * Deprecated, importance updating should be done by ImportanceUpdating
     * Agent. Still used by Embodiment.
     */
    void decayShortTermImportance() { atomTable.decayShortTermImportance(); }

    //! Clear the atomspace, remove all atoms
    void clear();

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
