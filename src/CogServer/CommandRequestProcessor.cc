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

void CommandRequestProcessor::load(std::string fileName) {
    AtomSpace *atomSpace = CogServer::getAtomSpace();;
    std::vector<XMLBufferReader*> readers(1, new FileXMLBufferReader(fileName.c_str()));
    NMXmlParser::loadXML(readers, atomSpace);
    delete readers[0];
}

std::string CommandRequestProcessor::ls() {

    AtomSpace *atomSpace = CogServer::getAtomSpace();;

    ostringstream stream;
    atomSpace->print(stream, NODE, true);
    atomSpace->print(stream, LINK, true);
    std::string answer = stream.str();

    /*
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
    */

    return answer;

}

void CommandRequestProcessor::processRequest(CogServerRequest *request) {

    std::string command = ((CommandRequest *) request)->getCommand();
    std::queue<std::string> args = ((CommandRequest *) request)->getArgs();
    std::string answer;
    
    if (command == "load") {
        if (args.size() != 1) {
            answer = "invalid command syntax";
        } else {
            load(args.front());
            answer = "load sucessful";
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
        answer = "unknown command";
    }

    ((CommandRequest *) request)->setAnswer(answer);
    request->callBack();

    /*
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
    */
}
