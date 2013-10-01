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

#include <Common/Compat.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>
#include <sstream>
#include <Common/Error.h>

#include <Hypertable/Lib/ApacheLogParser.h>
#include <Hypertable/Lib/Client.h>
#include <Hypertable/Lib/KeySpec.h>

#include <opencog/persist/hypertable/AtomspaceHTabler.h>

#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/TruthValue.h>

using namespace opencog;
using namespace Hypertable;

const int BUFF_SIZE = 1028; //TODO: Figure out how big this should actually be
const int TIMEOUT = 3000;

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


#ifdef HYPERTABLE_INSTALL_DIR
AtomspaceHTabler::AtomspaceHTabler(void)
{
    try {
        c = new Client(HYPERTABLE_INSTALL_DIR, HYPERTABLE_CONFIG_FILE, TIMEOUT);
    }
    catch (Hypertable::Exception& e) {
        if (e.code() == Error::REQUEST_TIMEOUT) {
            throw NetworkException(TRACE_INFO, "Timeout connecting to hyperspace."
                    " Confirm Hypertable servers are running and try again.");
        }
    }
    initTables();
}
#else
AtomspaceHTabler::AtomspaceHTabler(void)
{
    std::cerr << "To use hypertable functionality, define"
        << " HYPERTABLE_INSTALL_DIR and HYPERTABLE_CONFIG_FILE"
        << " in opencog/hypertable/AtomspaceHTabler.h" << std::endl;
    exit(1);
}
#endif

void AtomspaceHTabler::initTables(void)
{
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

void AtomspaceHTabler::clearData(void)
{
    c->drop_table("Atomtable", true);
    c->drop_table("Nametable", true);
    c->drop_table("Outsettable", true);
    initTables();
}

/**
 * This is a two-pass lookup: we get the handle from Outsettable
 * and use it to index into the main Atomtable.
 */
Link * AtomspaceHTabler::getLink(Type t, const std::vector<Handle>& handles) const
{
    TableScannerPtr scanner_ptr;
    ScanSpecBuilder ssb;
    Cell cell;
    std::stringstream row;

    ssb.add_column("handle");
    row << t;
    std::vector<Handle>::const_iterator iter;
    for (iter = handles.begin(); iter != handles.end(); ++iter)
    {
        row << ',';
        UUID uuid = (*iter).value();
        row << uuid;
    }
    ssb.add_row(row.str().c_str());
    ssb.set_max_versions(1);

    scanner_ptr = m_outset_table->create_scanner(ssb.get());
    if (scanner_ptr->next(cell))
    {
        std::string handle_str((char *)cell.value, cell.value_len);
        UUID uuid = strtoul(handle_str.c_str(), NULL, 10);
        Handle h(uuid);
        return dynamic_cast<Link *>(getAtom(h));
    }
    else {
        return NULL;
    }
}

/**
 * This is a two-pass lookup: we get the handle from Nametable
 * and use it to index into the main Atomtable.
 */
Node* AtomspaceHTabler::getNode(Type t, const char * name) const
{
    TableScannerPtr scanner_ptr;
    ScanSpecBuilder ssb;
    Cell cell;

    ssb.add_column("handle");
    std::stringstream row;
    row << name << ',' << (unsigned long) t;
    ssb.add_row(row.str().c_str());
    ssb.set_max_versions(1);

    scanner_ptr = m_name_table->create_scanner(ssb.get());
    if (scanner_ptr->next(cell))
    {
        std::string handle_str((char *)cell.value, cell.value_len);
        UUID uuid = strtoul(handle_str.c_str(), NULL, 10);
        Handle h(uuid);
        return dynamic_cast<Node *>(getAtom(h));
    }
    else
    {
        return NULL;
    }
}

/**
 * Recursively store the atom and anything in its outgoing set.
 * If the atom is already in storage, this will update it's
 * truth value, etc.
 */
void AtomspaceHTabler::storeAtom(Handle h)
{
    std::cout << "storeAtom() called\n";
    AtomSpace *as = cogserver().getAtomSpace();

    // If TLB didn't give us an atom, it was a bad handle and there's nothing
    // else to do.
    if (!as->isValidHandle(h)) {
        std::cerr << "storeAtom(): Bad handle\n";
        return;
    }

    KeySpec key;
    memset(&key, 0, sizeof(key));

    // Create row index from handle
    char row[BUFF_SIZE]; //TODO: Figure out how big this should actually be

    UUID uuid = h.value();
    key.row_len = snprintf(row, BUFF_SIZE, "%lu", uuid);
    key.row = row;

    // If it's a node...
    if (as->isNode(as->getType(h)))
    {
        // Store the name
        key.column_family = "name";
        m_handle_mutator->set(key, as->getName(h).c_str(), as->getName(h).length());

        // Store the handle in Nametable
        KeySpec name_key;
        memset(&name_key, 0, sizeof(name_key));

        char r[BUFF_SIZE];
        int len = snprintf(r, BUFF_SIZE, "%hu", as->getType(h));

        std::string name_index = as->getName(h) + ',' + std::string(r,len);
        name_key.row = name_index.c_str();
        name_key.row_len = name_index.length();
        name_key.column_family = "handle";

        m_name_mutator->set(name_key, key.row, key.row_len);
        m_name_mutator->flush(); //TODO: Flush to get rid of if possible
    }
    // If it's a link...
    else
    {
        // Store the outgoing set
        int arity = as->getArity(h);
        const std::vector<Handle> &out = as->getOutgoing(h);
        std::stringstream ss;
        for (int i = 0; i < arity; ++i)
        {
            ss << out[i];
            ss << ',';

            // XXX: Should the outgoing atom itself also be stored?
            // It makes sense that it would, though that leads to problems
            // with cycles.
        }
        key.column_family = "outgoing";
        m_handle_mutator->set(key, ss.str().c_str(), ss.str().length());

        // Store the handle in Outsettable
        KeySpec outset_key;
        memset(&outset_key, 0, sizeof(outset_key));

        char r[BUFF_SIZE];
        int len = snprintf(r, BUFF_SIZE, "%hu", as->getType(h));

        std::vector<Handle>::const_iterator iter;
        for (iter = as->getOutgoing(h).begin();
             iter != as->getOutgoing(h).end(); ++iter)
        {
            UUID uuid = (*iter).value();
            r[len++] = ',';
            len += snprintf(r+len, BUFF_SIZE-len, "%lu", uuid);
        }
        outset_key.row = r;
        outset_key.row_len = len;

        outset_key.column_family = "handle";
        m_outset_mutator->set(outset_key, key.row, key.row_len);
        m_outset_mutator->flush(); //TODO: Flush to get rid of if possible
    }

    char val[BUFF_SIZE];
    int val_len;

    // Store type
    Type t = as->getType(h);
    val_len = snprintf(val, BUFF_SIZE, "%d", t);
    key.column_family = "type";
    m_handle_mutator->set(key, val, val_len);

    // Store the importance
    const AttentionValue& av = as->getAV(h);
    short sti = av.getSTI();
    short lti = av.getLTI();
    unsigned short vlti = av.getVLTI();

    val_len = snprintf(val, BUFF_SIZE, "%hd", sti);
    key.column_family = "sti";
    m_handle_mutator->set(key, val, val_len);

    val_len = snprintf(val, BUFF_SIZE, "%hd", lti);
    key.column_family = "lti";
    m_handle_mutator->set(key, val, val_len);

    val_len = snprintf(val, BUFF_SIZE, "%hu", vlti);
    key.column_family = "vlti";
    m_handle_mutator->set(key, val, val_len);

    // Store the truth value
    const TruthValue &tv = as->getTV(h);
    const SimpleTruthValue *stv = dynamic_cast<const SimpleTruthValue *>(&tv);
    if (NULL == stv)
    {
        std::cerr << "Non-simple truth values are not handled\n";
        return;
    }
    val_len = snprintf(val, BUFF_SIZE, "(%f, %f)",
                tv.getMean(), tv.getCount());
    key.column_family = "stv";
    m_handle_mutator->set(key, val, val_len);


#ifdef THERE_IS_NO_NEED_TO_STORE_INCOMING_SET
    // Store incoming set
    HandleSeq hs = as->getIncoming(h);
    std::stringstream ss;
    foreach (Handle handle, hs) {
        ss << ',';
        ss << handle;
    }
    key.column_family = "incoming";
    m_handle_mutator->set(key, ss.str().c_str(), ss.str().length());
#endif /* THERE_IS_NO_NEED_TO_STORE_INCOMING_SET */

    // TODO: Find a way to get rid of this if possible; it may hurt performance.
    m_handle_mutator->flush();

    return;
}

/**
 * Return a vector containing the handles of the entire incoming
 * set of the indicated handle.
 */
std::vector<Handle> AtomspaceHTabler::getIncomingSet(Handle h) const
{
    TableScannerPtr scanner_ptr;
    ScanSpecBuilder ssb;
    Cell cell;
    std::vector<Handle> handles;

    ssb.add_column("incoming");
    char rowbuff[BUFF_SIZE];
    snprintf(rowbuff, BUFF_SIZE, "%lu", h.value());
    ssb.add_row(rowbuff);
    ssb.set_max_versions(1);

    scanner_ptr = m_handle_table->create_scanner(ssb.get());

    while (scanner_ptr->next(cell)) {
        char *end = (char *)cell.value + cell.value_len;
        char *comma = (char *)cell.value;
        while (comma != end) {
            Handle h = (Handle) strtoul(comma+1, &comma, 10);
            handles.push_back(h);
        }
    }

    return handles;
}

/**
 * Return a pointer to an Atom associated with the given
 * handle, if it exists; else return NULL.
 */
Atom * AtomspaceHTabler::getAtom(Handle h) const
{
    std::cout << "getAtom() called" << std::endl;
    TableScannerPtr scanner_ptr;
    ScanSpecBuilder ssb;
    Cell cell;

    ssb.add_column("type");
    ssb.add_column("name");
    ssb.add_column("stv");
    ssb.add_column("outgoing");
    ssb.add_column("sti");
    ssb.add_column("lti");
    ssb.add_column("vlti");

    char rowbuff[BUFF_SIZE];
    UUID uuid = h.value();
    snprintf(rowbuff, BUFF_SIZE, "%lu", uuid);
    ssb.add_row(rowbuff);
    ssb.set_max_versions(1);

    Atom *atom_ptr;

    try {
        scanner_ptr = m_handle_table->create_scanner(ssb.get());
    } catch (Exception &e) {
        std::cerr << e << std::endl;
        return NULL;
    }

    std::string name;
    const char* stv = 0;
    std::string stv_str;
    int type;
    std::vector<Handle> handles;
    short sti;
    short lti;
    unsigned short vlti;

    bool found = false;

    std::cout<< "getAtom(): scanning and processing data..." <<std::endl;

    //XXX: Can we guarantee the order these will be found? If so, we don't
    //      have to remember as much.
    // retrieve & process all the information about the atom
    while (scanner_ptr->next(cell))
    {
        found = true;
        if (!strcmp("type", cell.column_family))
        {
            std::cout << "getAtom(): processing type..." <<std::endl;
            type = atoi(std::string((char *)cell.value, cell.value_len).c_str());
        }
        else if (!strcmp("name", cell.column_family))
        {
            std::cout << "getAtom(): processing name..." <<std::endl;
            name = std::string((char *)cell.value, cell.value_len);
        }
        else if (!strcmp("stv", cell.column_family))
        {
            std::cout << "getAtom(): processing stv..." <<std::endl;
            std::cout << "getAtom(): stv length: " << cell.value_len << std::endl;
            stv = std::string((char *)cell.value, cell.value_len).c_str();
            stv_str = std::string((char *)cell.value, cell.value_len);
            std::cout << "getAtom(): cell data is at: " << &(cell.value) << ", stv is at: " << &stv << std::endl;
        }
        else if (!strcmp("outgoing", cell.column_family))
        {
            std::cout << "getAtom(): processing outgoing..." <<std::endl;
            char *end = (char *)cell.value + cell.value_len;
            char *comma = (char *)cell.value;
            while (comma != end) {
                Handle h = (Handle) strtoul(comma, &comma, 10);
                handles.push_back(h);
                comma++;
            }
        }
        else if (!strcmp("sti", cell.column_family))
        {
            std::cout << "getAtom(): processing sti..." <<std::endl;
            sti = (short) atoi(
                std::string((char *)cell.value, cell.value_len).c_str());
        }
        else if (!strcmp("lti", cell.column_family))
        {
            std::cout << "getAtom(): processing lti..." <<std::endl;
            lti = (short) atoi(
                std::string((char *)cell.value, cell.value_len).c_str());
        }
        else if (!strcmp("vlti", cell.column_family))
        {
            std::cout << "getAtom(): processing vlti..." <<std::endl;
            vlti = (short) atoi(
                std::string((char *)cell.value, cell.value_len).c_str());
        }
        std::cout << "getAtom(): item processed" <<std::endl;
        if (stv) std::cout << "getAtom(): stv char*: " << stv << std::endl;
            std::cout << "getAtom(): stv string: " << stv_str << std::endl;
    }
    std::cout<< "getAtom(): Processing complete. Creating atom." <<std::endl;
    std::stringstream mystrstr;
    mystrstr << stv;
    if (mystrstr.str().empty())
    {
        std::cout << "no further than this" << std::endl;
        // breakpoint here
        std::cout << "em effing stop" << std::endl;
    }
    std::cout << "getAtom(): stv char*: " << stv <<std::endl;
    std::cout << "getAtom(): stv string: " << stv_str << std::endl;

    if (!found) return NULL;

    if (classserver().isNode(type))
    {
        atom_ptr = new Node(type, name);
        std::cout << "getAtom(): Node created" <<std::endl;
        std::cout << "getAtom(): stv char*: " << stv <<std::endl;
        std::cout << "getAtom(): stv string: " << stv_str << std::endl;
    }
    else
    {
        atom_ptr = new Link(type, handles);
        std::cout << "getAtom(): Link created" <<std::endl;
        std::cout << "getAtom(): stv char*: " << stv <<std::endl;
        std::cout << "getAtom(): stv string: " << stv_str << std::endl;
    }

    // Restore importance
    const AttentionValue av(sti,lti,vlti);
    atom_ptr->setAttentionValue(av);
    std::cout<< "getAtom(): importance restored" <<std::endl;

    // Restore truth value
    std::cout << "getAtom(): stv char*: " << stv <<std::endl;
    std::cout << "getAtom(): stv string: " << stv_str << std::endl;
    double mean = atof(stv + 1);
    std::cout << "getAtom(): stv mean value: " << mean <<std::endl;
    char *comma = strchr(stv + 2, ',');
    std::cout << "getAtom(): stv comma:" << comma <<std::endl;
    double count = atof(comma + 1);
    std::cout << "getAtom(): stv count OK" <<std::endl;
    SimpleTruthValue nstv(mean, count);
    atom_ptr->setTruthValue(nstv);
    std::cout << "getAtom(): truth value restored" <<std::endl;

    // XXX we should explicitly add the atom to the TLB

    return atom_ptr;
}

