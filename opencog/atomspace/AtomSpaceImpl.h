/*
 * opencog/atomspace/AtomSpaceImpl.h
 *
 * Copyright (c) 2010-2011 OpenCog Foundation
 * Copyright (c) 2009, 2013 Linas Vepstas
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
 * the AtomSpace
 */
class AtomSpaceImpl
{
    friend class AtomSpace;
    friend class SavingLoading;
    friend class SaveRequest;
    friend class ::AtomSpaceImplUTest;

public:
    AtomSpaceImpl(void);
    ~AtomSpaceImpl();

    /**
     * Register a provider of backing storage.
     */
    void registerBackingStore(BackingStore *);
    void unregisterBackingStore(BackingStore *);

    /**
     * Read-write synchronization barrier.
     * If there is a backing store, then make sure all writes have
     * been completed.
     * NB: at this time,we don't distinguish barrier and flush.
     * Although this is named 'barrier', its actually implemented
     * as a flush.  This may change in the future.
     */
    void barrier(void) { if (backing_store) backing_store->barrier(); }

    /**
     * Recursively store the atom to the backing store.
     * I.e. if the atom is a link, then store all of the atoms
     * in its outgoing set as well, recursively.
     */
    void storeAtom(Handle h);

    /**
     * Load *all* atoms of the given type, but only if they are not
     * already in the AtomTable. 
     */
    void loadType(Type t) {
        if (backing_store) backing_store->loadType(atomTable, t);
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
    Handle fetchAtom(Handle);

    /**
     * Get an atom from the AtomTable. If not found there, get it from
     * the backingstore (and add it to the AtomTable).  If the atom is
     * not found in either place, return Handle::UNDEFINED.
     */
    Handle getAtom(Handle);

    /**
     * Use the backing store to load the entire incoming set of the atom.
     * If the flag is true, then the load is done recursively. 
     * This method queries the backing store to obtain all atoms that 
     * contain this one in their outgoing sets. All of these atoms are
     * then loaded into this atomtable/atomspace.
     */
    Handle fetchIncomingSet(Handle, bool);

    /** Add a new node to the Atom Table,
     * if the atom already exists then the old and the new truth value is merged
     *  @param t     Type of the node
     *  @param name  Name of the node
     *  @param tvn   Optional TruthValue of the node. If not provided, uses the
     *  DEFAULT_TV (see TruthValue.h)
     */
    Handle addNode(Type t, const std::string& name = "", TruthValuePtr tvn = TruthValue::DEFAULT_TV());

    /**
     * Get a node from the AtomTable, if it's in there. If its not found
     * in the AtomTable, and there's a backing store, then the atom will
     * be fetched from the backingstore (and added to the AtomTable). If
     * the atom can't be found in either place, Handle::UNDEFINED will be
     * returned.
     *
     * See also the getAtom() method.
     */
    Handle getNode(Type t, const std::string& name = "");

    /**
     * Add a new link to the AtomTable.
     * If the atom already exists in the AtomTable, then the old and
     * the new truth values are merged.
     * @param t         Type of the link
     * @param outgoing  a const reference to a HandleSeq containing
     *                  the outgoing set of the link
     * @param tvn       Optional TruthValue of the node. If not
     *                  provided, uses the DEFAULT_TV (see TruthValue.h)
     */
    Handle addLink(Type t, const HandleSeq& outgoing,
                   TruthValuePtr tvn = TruthValue::DEFAULT_TV());

    /**
     * Get a link from the AtomTable, if it's in there. If its not found
     * in the AtomTable, and there's a backing store, then the atom will
     * be fetched from the backingstore (and added to the AtomTable). If
     * the atom can't be found in either place, Handle::UNDEFINED will be
     * returned.
     *
     * See also the getAtom() method.
     */
    Handle getLink(Type t, const HandleSeq& outgoing);

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
     * Purges an atom from the atomtable. Attached storage is not
     * affected.  The removed atom stays valid (is not deleted) until
     * all handles and pointers referencing it are deleted.
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
        return 0 < atomTable.extract(h, recursive).size();
    }

    /**
     * Remove an atom from the atomtable and from attached storage.
     * The removed atom stays valid (is not deleted) until all
     * handles and pointers referencing it are deleted.
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
    bool removeAtom(Handle h, bool recursive = true);

    //! Clear the atomspace, remove all atoms
    void clear();

    AtomTable atomTable;
    AttentionBank bank;

private:
    /**
     * Used to fetch atoms from disk.
     */
    BackingStore *backing_store;

public:
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
