/*
 * src/CogServer/CommandRequestProcessor.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#include <pthread.h>
#include <sstream>

#include "CommandRequestProcessor.h"
#include "CommandRequest.h"
#include "FileXMLBufferReader.h"
#include "StringXMLBufferReader.h"
#include "NMXmlParser.h"


using namespace opencog;

CommandRequestProcessor::~CommandRequestProcessor() {
#ifdef HAVE_SQL_STORAGE
    if (store) delete store;
    store = NULL;
#endif /* HAVE_SQL_STORAGE */
#ifdef HAVE_GUILE
    if (ss) delete ss;
    ss = NULL;
#endif /* HAVE_GUILE */
}

CommandRequestProcessor::CommandRequestProcessor() {
    load_count = 0;
#ifdef HAVE_SQL_STORAGE
    store = NULL;
#endif /* HAVE_SQL_STORAGE */
#ifdef HAVE_GUILE
    ss = new SchemeShell();
#endif /* HAVE_GUILE */
}

/* ============================================================== */
/**
 * read XML data from a buffer
 */
std::string CommandRequestProcessor::loadXML(XMLBufferReader *buf) {
    load_count ++;
    char buff[20];
    snprintf(buff, 20, "%d", load_count);
    std::string msg = "xml load ";
    msg += buff;
    msg += " successful";
    AtomSpace *atomSpace = CogServer::getAtomSpace();
    std::vector<XMLBufferReader*> readers(1, buf);
    try {
        NMXmlParser::loadXML(readers, atomSpace);
    }
    // Can catch IOException, for file not found, or
    // RuntimeException, for bad file format.
    catch (StandardException &e)
    {
        msg = "xml load ";
        msg += buff;
        msg = " failed: ";
        msg += e.getMessage();
    }
    delete readers[0];
    return msg;
}

/* ============================================================== */
std::string CommandRequestProcessor::help(std::string topic)
{
    std::string reply = "";

    if (0 < topic.size())
    {
        reply += "No help available for command \"";
        reply += topic;
        reply += "\"\n\n";
    }


    reply += 
         "Available commands:\n"
         "data <xmldata>   -- load OpenCog XML data immediately following\n"
         "load <filename>  -- load OpenCog XML from indicated filename\n"
         "ls               -- list entire system contents\n"
         "ls <handle>      -- list handle and its incoming set\n"
         "ls <type> <name> -- list node and its incoming set\n";
#ifdef HAVE_GUILE
    reply += 
         "scm              -- enter the scheme interpreter\n";
#endif /* HAVE_GUILE */
#ifdef HAVE_SQL_STORAGE
    reply += 
         "sql-open <dbname> <username> <auth>\n"
         "                 -- open connection to SQL storage\n"
         "sql-close        -- close connection to SQL storage\n"
         "sql-store        -- store all server data to SQL storage\n"
         "sql-load         -- load server from SQL storage\n";
#endif
    reply += 
         "shutdown         -- stop the server, and exit\n";

    return reply;
}

/* ============================================================== */
/**
 * Read XML data from a file
 */
std::string CommandRequestProcessor::load(std::string fileName)
{
    return loadXML(new FileXMLBufferReader(fileName.c_str()));
}

/**
 * Read XML data from a string
 */
std::string CommandRequestProcessor::data(std::string buf)
{
    return loadXML(new StringXMLBufferReader(buf.c_str()));
}

std::string CommandRequestProcessor::ls(std::string type, std::string name)
{
    Type t = atoi(type.c_str());

    AtomSpace *atomSpace = CogServer::getAtomSpace();
    Handle h = atomSpace->getAtomTable().getHandle(name.c_str(), t);
    return ls(h);
}

/* ============================================================== */

std::string CommandRequestProcessor::ls(std::string arg)
{
    if (0 == arg.length())
    {
        return ls();
    }

    // If its all numeric, its a handle !?
    size_t alpha = arg.find_first_not_of("0123456789");
    if (std::string::npos == alpha)
    {
        Handle h = (Handle) strtoul(arg.c_str(), NULL, 10);
        return ls(h);
    }

    return "Invalid ls format: handle must be numeric";
}

std::string CommandRequestProcessor::ls(Handle h)
{
    if (false == TLB::isValidHandle(h)) return "Invalid handle/unknown node";

    char buff[20];
    std::string answer;

    snprintf(buff, 20, "%lu: ", (unsigned long) h);
    answer += buff;
    Atom *atom = TLB::getAtom(h);
    answer += atom->toString();
    answer += "\n";

    HandleEntry *he = atom->getIncomingSet(); 
    while (he)
    {
        answer += "\t";
        snprintf(buff, 20, "%lu: ", (unsigned long) he->handle);
        answer += buff;
        answer += TLB::getAtom(he->handle)->toString();
        answer += "\n";
        he = he->next;
    }
    return answer;
}

/**
 * A simple-minded list of the graph
 */
std::string CommandRequestProcessor::ls(void)
{
    AtomSpace *atomSpace = CogServer::getAtomSpace();;

    ostringstream stream;
    atomSpace->print(stream, NODE, true);
    atomSpace->print(stream, LINK, true);
    std::string answer = stream.str();

#ifdef ALTERNATE_LS_OUTPUT
    std::vector<Handle> nodes;
    atomSpace->getHandleSet(back_inserter(nodes), NODE, true);

    std::vector<Handle> links;
    atomSpace->getHandleSet(back_inserter(links), LINK, true);

    std::string answer;

    for (unsigned int i = 0; i < nodes.size(); i++) {
        answer.append(atomSpace->getName(atomSpace->getType(nodes[i])));
        answer.append(": ");
        answer.append(atomSpace->getName(nodes[i]));
        answer.append("\n");
    }

    for (unsigned int i = 0; i < links.size(); i++) {
        answer.append(atomSpace->getName(links[i]));
        answer.append("(");
        const HandleSeq &linkedAtoms = atomSpace->getOutgoing(links[i]);
        for (unsigned int i = 0; i < linkedAtoms.size(); i++) {
            if (atomSpace->isNode(linkedAtoms[i])) {
                answer.append(atomSpace->getName(linkedAtoms[i]));
            } else {
                answer.append("<link>");
            }
            if (i != (linkedAtoms.size() - 1)) {
                answer.append(" ");
            }
        }
        answer.append(")");
        answer.append("\n");
    }
#endif /* ALTERNATE_LS_OUTPUT */

    return answer;

}

/* ============================================================== */
#ifdef HAVE_SQL_STORAGE
/**
 * Open a connection to the sql server
 */
std::string CommandRequestProcessor::sql_open(std::string dbname,
                                              std::string username,
                                              std::string authentication)
{
    if (store) 
    {
        return "Error: SQL connection already open\n";
    }

    store = new AtomStorage(dbname, username, authentication);

    std::string answer = "Opened \"";
    answer += dbname;
    answer += "\" as user \"";
    answer += username;
    answer += "\"\n";
    return answer;
}

/**
 * Close connection to the sql server
 */
std::string CommandRequestProcessor::sql_close(void)
{
    if (store) 
    {
        delete store;
        store = NULL;
        return "SQL connection closed\n";
    }
    return "Warning: SQL connection not open\n";
}

/**
 * Load data from the sql server
 */
static void * sql_load_thread_start_routine(void *data)
{
    AtomStorage *store = (AtomStorage *) data;
    AtomSpace *atomSpace = CogServer::getAtomSpace();
    const AtomTable& atomTable = atomSpace->getAtomTable();

    AtomTable * ap = (AtomTable *) &atomTable; // It most certainly is NOT const!!
    store->load(*ap);

    return NULL;
}

std::string CommandRequestProcessor::sql_load(void)
{
    if (!store) return "Error: No SQL connection is open";

    pthread_t loader;
    int rc = pthread_create (&loader, NULL, sql_load_thread_start_routine, store);

    if (rc) return "Error:Failed to create SQL loader thread";
    return "SQL loader thread started\n";
}

/**
 * Store data to the sql server
 */
static void * sql_store_thread_start_routine(void *data)
{
    AtomStorage *store = (AtomStorage *) data;

    AtomSpace *atomSpace = CogServer::getAtomSpace();
    const AtomTable& atomTable = atomSpace->getAtomTable();
    store->store(atomTable);

    return NULL;
}

std::string CommandRequestProcessor::sql_store(void)
{
    if (!store) return "Error: No SQL connection is open";

    pthread_t saver;
    int rc = pthread_create (&saver, NULL, sql_store_thread_start_routine, store);

    if (rc) return "Error:Failed to create SQL store thread";
    return "SQL data store thread started\n";
}
#endif /* HAVE_SQL_STORAGE */

/* ============================================================== */

void CommandRequestProcessor::processRequest(CogServerRequest *req)
{
    CommandRequest * request = dynamic_cast<CommandRequest *>(req);

    std::string command = request->getCommand();
    std::queue<std::string> args = request->getArgs();

    std::string answer;
    if (command == "data") {
        if (args.size() != 1) {
            answer = "data: invalid command syntax";
        } else {
            answer = data(args.front());
        }
    }
    else if (command == "help")
    {
        if (args.size() == 0) answer = help("");
        else answer = help(args.front());
    }
    else if (command == "load") {
        if (args.size() != 1) {
            answer = "load: invalid command syntax";
        } else {
            answer = load(args.front());
        }
    } else if (command == "ls") {
        if (args.size() == 0) {
            answer = ls();
        } else if (args.size() == 1) {
            answer = ls(args.front());
        } else if (args.size() == 2) {
            std::string type = args.front();
            args.pop();
            std::string name = args.front();
            answer = ls(type, name);
        } else {
            answer = "ls: invalid command syntax";
        }
    } else if (command == "shutdown") {
        if (args.size() != 0) {
            answer = "shutdown: invalid command syntax";
        } else {
#ifdef HAVE_SQL_STORAGE
            sql_close();
#endif /* HAVE_SQL_STORAGE */
            server().stop();
        }
    } 
#ifdef HAVE_GUILE
    else if (command == "scm") {
        if (args.size() != 1) {
            answer = "scm: invalid command syntax";
        } else {
            answer = ss->eval(args.front());
        }
    }
#endif /* HAVE_GUILE */
#ifdef HAVE_SQL_STORAGE
    else if (command == "sql-open") {
        if (args.size() < 2) {
            answer = "sql-open: invalid command syntax";
        } else if (args.size() == 2) {
            std::string dbname = args.front();
            args.pop();
            std::string username = args.front();
            answer = sql_open(dbname, username, "");
        } else {
            std::string dbname = args.front();
            args.pop();
            std::string username = args.front();
            args.pop();
            std::string auth = args.front();
            answer = sql_open(dbname, username, auth);
        }
    } else if (command == "sql-close") {
        if (args.size() != 0) {
            answer = "sql-close: invalid command syntax";
        } else {
            answer = sql_close();
        }
    } else if (command == "sql-load") {
        if (args.size() != 0) {
            answer = "sql-load: invalid command syntax";
        } else {
            answer = sql_load();
        }
    } else if (command == "sql-store") {
        if (args.size() != 0) {
            answer = "sql-store: invalid command syntax";
        } else {
            answer = sql_store();
        }
    } 
#endif /* HAVE_SQL_STORAGE */
    else {
        answer = "unknown command >>" + command + "<<\n" +
                 "\tAvailable commands: data help load ls shutdown";
#ifdef HAVE_GUILE
        answer += " scm";
#endif /* HAVE_GUILE */
#ifdef HAVE_SQL_STORAGE
        answer += "\n\tsql-open sql-close sql-store sql-load";
#endif
        if(!args.empty())
            answer += "\tArgs: " + args.front();
    }

    request->setAnswer(answer);
    request->callBack();

#ifdef DEBUG 
     // Debug code
    answer = "processed command " + command + "(";
    while (! args.empty()) {
        answer.append(args.front());
        args.pop();
        if (! args.empty()) {
            answer.append(",");
        }
    }
    answer.append(")");
    ((CommandRequest *) request)->setAnswer(answer);
    request->callBack();
#endif /* DEBUG */

}
