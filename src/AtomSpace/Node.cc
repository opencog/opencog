/**
 * Node.cc
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */
#include <stdio.h>
#include "Node.h"
#include "ClassServer.h"
//#include "exceptions.h"
#include "Link.h"
#include "TLB.h"
#include "utils.h"
#include "AtomSpaceDefinitions.h"


Node::Node(Type type, const std::string& name, const TruthValue& tv) throw (InvalidParamException, AssertionException) : Atom(type, std::vector<Handle>(), tv) {

    if (!ClassServer::isAssignableFrom(NODE, type)) {
        throw InvalidParamException(TRACE_INFO, "Node - Invalid node type '%d'.", type);
    }
    
    cassert(TRACE_INFO, &name, "Node - name parameter should not be NULL.");
    
    // If name is empty let it unitialized for saving memory...
    if (!name.empty()) {
        this->name = name;
    }
}

Node::~Node() throw () {
//    fprintf(stdout, "Deleting Node:\n%s\n", this->toString().c_str());
//    fflush(stdout);
}

const std::string& Node::getName() const{
    return name;
}

void Node::setName(const std::string& name) throw (RuntimeException) {
    
    if (atomTable != NULL){
        throw RuntimeException(TRACE_INFO, 
              "Node - Cannot change the name of a node already inserted into an AtomTable.");
    }

    if (!name.empty()) {
        this->name = name;
    } else if (!(this->name.empty())) {
        this->name = name;
    }
}

void Node::merge(Atom* atom) throw (InconsistenceException) {

    if (!equals(atom)){
        throw InconsistenceException(TRACE_INFO, "Node - Different nodes cannot be merged");
    }

    Atom::merge(atom);
}

std::string Node::toShortString() {
    char buf[1024];
    sprintf(buf, "node[%d:%s%s]", type, name.c_str(), (getFlag(HYPOTETHICAL_FLAG)?":h":""));
    return buf;
}

std::string Node::toString() {
    char buf[1024];
    sprintf(buf, "node[%d:%s] (%d,%f)", type, name.c_str(), (int)getAttentionValue().getSTI(), 0.0); //activation here at 0: can be replace with LTI
    return buf;
}

bool Node::equals(Atom *other){
    bool result = false;
    if (other != NULL && (type == other->getType())){
        Node *otherNode = (Node *) other;
        result = !strcmp(name.c_str(), otherNode->getName().c_str());
    }
    return(result);
}

int Node::hashCode(){
    int result = Atom::hashCode();
    result += Util::hash<const char*>()(name.c_str());
    return result;
}


