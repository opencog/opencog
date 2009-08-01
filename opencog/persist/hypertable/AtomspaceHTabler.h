/*
 * opencog/hypertable/AtomspaceHTabler.h
 *
 * Copyright (C) 2009 Jeremy Schlatter <jeremy.schlatter@gmail.com>
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


        const String attribute_schema = "\
<Schema>\n\
  <AccessGroup name=\"default\">\n\
    <ColumnFamily>\n\
      <Name>name</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
    <ColumnFamily>\n\
      <Name>type</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
    <ColumnFamily>\n\
      <Name>stv</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
    <ColumnFamily>\n\
      <Name>incoming</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
    <ColumnFamily>\n\
      <Name>outgoing</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
    <ColumnFamily>\n\
      <Name>sti</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
    <ColumnFamily>\n\
      <Name>lti</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
    <ColumnFamily>\n\
      <Name>vlti</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
  </AccessGroup>\n\
</Schema>";

        const String handle_schema = "\
<Schema>\n\
  <AccessGroup name=\"default\">\n\
    <ColumnFamily>\n\
      <Name>handle</Name>\n\
      <deleted>false</deleted>\n\
    </ColumnFamily>\n\
  </AccessGroup>\n\
</Schema>";
namespace opencog
{

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
public:
        #ifdef HYPERTABLE_INSTALL_DIR
        AtomspaceHTabler(void) {
            c = new Client(HYPERTABLE_INSTALL_DIR, HYPERTABLE_CONFIG_FILE);
            std::vector<String> tables;
            c->get_tables(tables);
            if (find(tables.begin(), tables.end(), "Atomtable") == tables.end()) {
                c->create_table("Atomtable", attribute_schema);
            }
            if (find(tables.begin(), tables.end(), "Nametable") == tables.end()) {
                c->create_table("Nametable", handle_schema);
            }
            if (find(tables.begin(), tables.end(), "Outsettable") == tables.end()) {
                c->create_table("Outsettable", handle_schema);
            }
            m_handle_table = c->open_table("Atomtable");
            m_handle_mutator = m_handle_table->create_mutator();
            m_name_table = c->open_table("Nametable");
            m_name_mutator = m_name_table->create_mutator();
            m_outset_table = c->open_table("Outsettable");
            m_outset_mutator = m_outset_table->create_mutator();
        }
        #else
        AtomspaceHTabler(){
            std::cerr << "To use hypertable functionality, define" 
                << " HYPERTABLE_INSTALL_DIR and HYPERTABLE_CONFIG_FILE" 
                << " in opencog/hypertable/AtomspaceHTabler.h" << std::endl;
            exit(1);
        }   
        #endif
        virtual ~AtomspaceHTabler(){}
        
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

} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_HTABLER_H
