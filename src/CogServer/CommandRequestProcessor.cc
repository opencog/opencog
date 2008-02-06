#include "CommandRequestProcessor.h"
#include "CommandRequest.h"

#include <sstream>

#include <FileXMLBufferReader.h>
#include <NMXmlParser.h>

using namespace opencog;

CommandRequestProcessor::~CommandRequestProcessor() {
}

CommandRequestProcessor::CommandRequestProcessor() {
}

std::string CommandRequestProcessor::load(std::string fileName)
{
    std::string msg = "load successful";
    AtomSpace *atomSpace = CogServer::getAtomSpace();
    std::vector<XMLBufferReader*> readers(1, new FileXMLBufferReader(fileName.c_str()));
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

std::string CommandRequestProcessor::ls() {

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

void CommandRequestProcessor::processRequest(CogServerRequest *request)
{
    std::string command;
    std::queue<std::string> args;

    // Use RTTI for a safe cast
    if (typeid(request) == typeid(CommandRequest))
    { 
        command = ((CommandRequest *) request)->getCommand();
        args = ((CommandRequest *) request)->getArgs();
    }
    else
    {
        return;
    }

    std::string answer;
    if (command == "load") {
        if (args.size() != 1) {
            answer = "invalid command syntax";
        } else {
            answer = load(args.front());
        }
    } else if (command == "ls") {
        if (args.size() != 0) {
            answer = "invalid command syntax";
        } else {
            answer = ls();
        }
    } else if (command == "shutdown") {
        if (args.size() != 0) {
            answer = "invalid command syntax";
        } else {
            exit(0);
        }
    } else {
        answer = "unknown command\n\tAvailable commands: load ls shutdown";
    }

    ((CommandRequest *) request)->setAnswer(answer);
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
