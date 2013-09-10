/*
 * opencog/hypertable/AtomspaceHTabler.h
 *
 * Copyright (C) 2009-2011 OpenCog Foundation
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

#ifndef _OPENCOG_ATOMSPACE_HTABLER_H
#define _OPENCOG_ATOMSPACE_HTABLER_H

#include "Common/Compat.h"
#include "Hypertable/Lib/Client.h"

#include "opencog/atomspace/Handle.h"
#include "opencog/atomspace/Node.h"
#include "opencog/atomspace/Link.h"
#include "opencog/atomspace/BackingStore.h"

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

/**
 * Persistent Atom storage, backed by Hypertable.
 */
class AtomspaceHTabler : public BackingStore
{
    private:
        ClientPtr c;
        TablePtr m_handle_table;
        TableMutatorPtr m_handle_mutator;
        TablePtr m_name_table;
        TableMutatorPtr m_name_mutator;
        TablePtr m_outset_table;
        TableMutatorPtr m_outset_mutator;
        
        KeySpec make_key(Handle);
        KeySpec make_key(Type, const std::vector<Handle>&);
        KeySpec make_key(Type, const char*);
        
        void initTables();
    public:

        /**
         * Initializes the object, connects to Hypertable.
         * @throws NetworkException if the connection times out,
         * usually indicating that the Hypertable servers are not running.
         */
        AtomspaceHTabler(void);
        virtual ~AtomspaceHTabler(){}
        
        /**
         * Erases everything stored in the Hypertable.
         */
        void clearData();
        
        /** 
         * Return a pointer to a link of the indicated type and outset,
         * if it exists; else return NULL.
         */
        virtual Link * getLink(Type, const std::vector<Handle>&) const;

        /** 
         * Return a pointer to a node of the indicated type and name,
         * if it exists; else return NULL.
         */
        virtual Node * getNode(Type, const char *) const;

        /**
         * Recursively store the atom and anything in its outgoing set.
         * If the atom is already in storage, this will update it's 
         * truth value, etc. 
         */
        virtual void storeAtom(Handle);

        /**
         * Return a vector containing the handles of the entire incoming
         * set of the indicated handle. 
         */
        virtual std::vector<Handle> getIncomingSet(Handle) const;

        /** 
         * Return a pointer to an Atom associated with the given
         * handle, if it exists; else return NULL.
         */
        virtual Atom * getAtom(Handle) const;
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_HTABLER_H
