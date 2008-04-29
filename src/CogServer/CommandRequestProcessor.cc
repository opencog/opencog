
#include "CommandRequestProcessor.h"
#include "CommandRequest.h"

#include <sstream>

#include <FileXMLBufferReader.h>
#include <StringXMLBufferReader.h>
#include <NMXmlParser.h>

using namespace opencog;

CommandRequestProcessor::~CommandRequestProcessor()
{
#ifdef HAVE_SQL_STORAGE
    if (store) delete store;
    store = NULL;
#endif /* HAVE_SQL_STORAGE */
}

CommandRequestProcessor::CommandRequestProcessor() 
{
#ifdef HAVE_SQL_STORAGE
    store = NULL;
#endif /* HAVE_SQL_STORAGE */
}

/**
 * read XML data from a buffer
 */
std::string CommandRequestProcessor::loadXML(XMLBufferReader *buf)
{
    std::string msg = "load successful";
    AtomSpace *atomSpace = CogServer::getAtomSpace();
    std::vector<XMLBufferReader*> readers(1, buf);
    try {
        NMXmlParser::loadXML(readers, atomSpace);
    }
    // Can catch IOException, for file not found, or
    // RuntimeException, for bad file format.
    catch (StandardException &e)
    {
        msg = "xload failed: ";
        msg += e.getMessage();
    }
    delete readers[0];
    return msg;
}

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
         "ls               -- list entire system contents\n";
#ifdef HAVE_SQL_STORAGE
    reply += 
         "sql-open <dbname> <username> <auth>\n"
         "                 -- open connection to SQL storage\n"
         "sql-close        -- close connection to SQL storage\n"
         "sql-store        -- store all server data to SQL storage\n"
         "sql-load         -- load server from SQL storage\n";
#endif
    return reply;
}

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

/**
 * A simple-minded list of the graph
 */
std::string CommandRequestProcessor::ls()
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

/**
 * Open a connection to the sql server
 */
std::string CommandRequestProcessor::sql_open(std::string dbname,
                                              std::string username,
                                              std::string authentication)
{
#ifdef HAVE_SQL_STORAGE
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
#endif /* HAVE_SQL_STORAGE */
}

/**
 * Close connection to the sql server
 */
std::string CommandRequestProcessor::sql_close(void)
{
#ifdef HAVE_SQL_STORAGE
    if (store) 
    {
        delete store;
        store = NULL;
        return "SQL connection closed\n";
    }
    return "Warning: SQL connection not open\n";
#endif /* HAVE_SQL_STORAGE */
}

/**
 * Load data from the sql server
 */
std::string CommandRequestProcessor::sql_load(void)
{
#ifdef HAVE_SQL_STORAGE
    if (!store) return "Error: No SQL connection is open";

    AtomSpace *atomSpace = CogServer::getAtomSpace();
    const AtomTable& atomTable = atomSpace->getAtomTable();

    AtomTable * ap = (AtomTable *) &atomTable; // It most certainly is NOT const!!
    store->load(*ap);

    return "SQL data loaded\n";
#endif /* HAVE_SQL_STORAGE */
}

/**
 * Store data to the sql server
 */
std::string CommandRequestProcessor::sql_store(void)
{
#ifdef HAVE_SQL_STORAGE
    if (!store) return "Error: No SQL connection is open";

    AtomSpace *atomSpace = CogServer::getAtomSpace();
    const AtomTable& atomTable = atomSpace->getAtomTable();
    store->store(atomTable);

    return "opencog data stored to SQL\n";
#endif /* HAVE_SQL_STORAGE */
}

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
        if (args.size() != 0) {
            answer = "ls: invalid command syntax";
        } else {
            answer = ls();
        }
    } else if (command == "shutdown") {
        if (args.size() != 0) {
            answer = "shutdown: invalid command syntax";
        } else {
            exit(0);
        }
    } else if (command == "sql-open") {
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
    } else {
        answer = "unknown command >>" + command + "<<\n" +
                 "\tAvailable commands: data help load ls shutdown";
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
