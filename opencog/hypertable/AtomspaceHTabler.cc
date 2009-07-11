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

#include "Common/Compat.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>
#include <sstream>
#include "Common/Error.h"

#include "Hypertable/Lib/ApacheLogParser.h"
#include "Hypertable/Lib/Client.h"
#include "Hypertable/Lib/KeySpec.h"

#include "opencog/hypertable/AtomspaceHTabler.h"
#include "opencog/atomspace/SimpleTruthValue.h"
#include "opencog/atomspace/TruthValue.h"
#include "opencog/atomspace/TLB.h"
#include "opencog/atomspace/Atom.h"
#include "opencog/atomspace/Node.h"
#include "opencog/atomspace/Link.h"
#include "opencog/atomspace/AttentionValue.h"


using namespace opencog;
using namespace Hypertable;

const int BUFF_SIZE = 1028; //TODO: Figure out how big this should actually be

/**
 * Recursively store the atom and anything in its outgoing set.
 * If the atom is already in storage, this will update it's
 * truth value, etc.
 */
void AtomspaceHTabler::storeAtom(Handle h)
{
    Atom *atom_ptr = TLB::getAtom(h);

    // If TLB didn't give us an atom, it was a bad handle and there's nothing
    // else to do.
    if (!atom_ptr) {
        std::cerr << "storeAtom(): Bad handle" << std::endl;
        return;
    }

    TablePtr table_ptr;
    TableMutatorPtr mutator_ptr;
    KeySpec key;
    memset(&key, 0, sizeof(key));

    try {
        table_ptr = client_ptr->open_table("Atomtable");
        mutator_ptr = table_ptr->create_mutator(3000);
    } catch (Exception &e) {
        std::cerr << e << std::endl;
        return;
    }
    
    // Create row index from handle
    char row[BUFF_SIZE]; //TODO: Figure out how big this should actually be
    key.row_len = snprintf(row, BUFF_SIZE, "%lu", h.value());
    key.row = row;
        
    // If it's a node...
    Node *n = dynamic_cast<Node *>(atom_ptr);
    if (n) {
        //Store the name
        key.column_family = "name";
        mutator_ptr->set(key, n->getName().c_str(), n->getName().length());
    }
    // If it's a link...
    else {
        // Store the outgoing set
        Link *l = dynamic_cast<Link *>(atom_ptr);
        int arity = l->getArity();
        std::vector<Handle> out = l->getOutgoingSet();
        std::stringstream ss;
        for (int i=0; i<arity; ++i)
		{
		    ss << ',';
            ss << out[i];
		
			// XXX: Should the outgoing atom itself also be stored?
			// It makes sense that it would, though that leads to problems
			// with cycles.
		}
		key.column_family = "outgoing";
		mutator_ptr->set(key, ss.str().c_str(), ss.str().length());
    }
    
    
    char val[BUFF_SIZE];
    int val_len;
    
    // Store type
    Type t = atom_ptr->getType();
    val_len = snprintf(val, BUFF_SIZE, "%d", t);
    key.column_family = "type";
    mutator_ptr->set(key, val, val_len);

    
    // Store the importance
    const AttentionValue& av = atom_ptr->getAttentionValue();
    short sti = av.getSTI();
    short lti = av.getLTI();
    unsigned short vlti = av.getVLTI();
    
    val_len = snprintf(val, BUFF_SIZE, "%hd", sti);
    key.column_family = "sti";
    mutator_ptr->set(key, val, val_len);
    
    val_len = snprintf(val, BUFF_SIZE, "%hd", lti);
    key.column_family = "lti";
    mutator_ptr->set(key, val, val_len);
    
    val_len = snprintf(val, BUFF_SIZE, "%hu", vlti);
    key.column_family = "vlti";
    mutator_ptr->set(key, val, val_len);
    
    
    // Store the truth value
    const TruthValue &tv = atom_ptr->getTruthValue();
    const SimpleTruthValue *stv = dynamic_cast<const SimpleTruthValue *>(&tv);
    if (NULL == stv) {
        std::cerr << "Non-simple truth values are not handled" << std::endl;
        return;
    }
    val_len = snprintf(val, BUFF_SIZE, "(%20.16g, %20.16g)",
                tv.getMean(), tv.getCount());
    key.column_family = "stv";
    mutator_ptr->set(key, val, val_len);
    
    //TODO: Find a way to get rid of this if possible; it may hurt performance.
    mutator_ptr->flush();     
    return;
}

/**
 * Return a pointer to an Atom associated with the given
 * handle, if it exists; else return NULL.
 */
Atom * AtomspaceHTabler::getAtom(Handle h) const
{
    TablePtr table_ptr;
    TableScannerPtr scanner_ptr;
    ScanSpecBuilder ssb;
    Cell cell;
    String str;

    str = "type";
    ssb.add_column(str.c_str());
    str = "name";
    ssb.add_column(str.c_str());
    str = "stv";
    ssb.add_column(str.c_str());
    str = "outgoing";
    ssb.add_column(str.c_str());
    str = "sti";
    ssb.add_column(str.c_str());
    str = "lti";
    ssb.add_column(str.c_str());
    str = "vlti";
    ssb.add_column(str.c_str());

    char rowbuff[BUFF_SIZE];
    snprintf(rowbuff, BUFF_SIZE, "%lu", h.value());
    ssb.add_row(rowbuff);
    ssb.set_max_versions(1);

    Atom *atom_ptr;

    try {
        table_ptr = client_ptr->open_table("Atomtable");
        scanner_ptr = table_ptr->create_scanner(ssb.get());
    } catch (Exception &e) {
        std::cerr << e << std::endl;
        return atom_ptr;
    }
    

    char* name;
    char* stv;
    int type;
    std::vector<Handle> handles;
    short sti;
    short lti;
    unsigned short vlti;   
    
    bool found = false;
    
    // retrieve & process all the information about the atom
    while (scanner_ptr->next(cell)) {
        found = true;
        if (!strcmp("type", cell.column_family)) {
            type = atoi((char *)cell.value);
        } else if (!strcmp("name", cell.column_family)) {
            name = (char *)cell.value;
        } else if (!strcmp("stv", cell.column_family)) {
            stv = (char *)cell.value;
        } else if (!strcmp("outgoing", cell.column_family)) {         
            char *end = (char *)cell.value + cell.value_len;
            char *comma = (char *)cell.value;
            while (comma != end) {
                Handle h = (Handle) strtoul(comma+1, &comma, 10);
                handles.push_back(h);
            }
        } else if (!strcmp("sti", cell.column_family)) {
            sti = (short) atoi((char *)cell.value);
        } else if (!strcmp("lti", cell.column_family)) {
            lti = (short) atoi((char *)cell.value);
        } else if (!strcmp("vlti", cell.column_family)) {
            vlti = (short) atoi((char *)cell.value);
        }
    }
    
    
    if (!found){
        return NULL;
    }
    
    if (classserver().isNode(type)) {
        atom_ptr = new Node(type, name);
    } else {
        atom_ptr = new Link(type, handles);
    }
    
    // Restore importance
    const AttentionValue av(sti,lti,vlti);
    atom_ptr->setAttentionValue(av);
    
    
    // Restore truth value
    double mean = atof(stv + 1);
    char *comma = strchr(stv + 2, ',');
    double count = atof(comma + 1);
    SimpleTruthValue nstv(mean, count);
    atom_ptr->setTruthValue(nstv);
    

    return atom_ptr;
}


