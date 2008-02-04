/**
 * NMXmlParser.cc - uses expat lib to parse xml and load wordnet
 *
 * Copyright (c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

//#include "platform.h"

#include "NMXmlParser.h"
#include "FileXMLBufferReader.h"
#include "NMXmlDefinitions.h"
#include "SimpleTruthValue.h"
#include "Link.h" 
#include "Logger.h"

#include <math.h>
#include <stack>

extern "C" {
#include "xmlparse.h"
}

// NOTE: Any time information (including Timestamps) are represented using Atoms as follows: 
// AtTimeLink(TimeNode:<temporalStringValue>, Atom1 [, Atom2 [... , AtomN]])

Util::hash_map<char *, Handle, Util::hash<char *>, Util::eqstr> NMXmlParser::hypHandles;
bool NMXmlParser::fresh = true;
bool NMXmlParser::freshLinks = false;

//const TruthValue& NMXmlParser::DEFAULT_TV() = SimpleTruthValue(0.000001f, 0.5f);
const TruthValue& NMXmlParser::DEFAULT_TV() {
    static TruthValue* instance = new SimpleTruthValue(0.000001f, 0.5f);
    return *instance;
}

// const int TYPE_LENGTH = (int) log10((double)NUMBER_OF_CLASSES) + 1;

typedef struct {
    unsigned enabled: 1;
    unsigned ignoring: 1;
    unsigned processNodes: 1;
    unsigned processRelationships: 1;
} Status;

typedef std::stack<void*> ParserStack;
// TODO: CREATE AN ABSTRACT CLASS "Stackable" for check for subclasses 
// when poping elements from stack.

typedef struct {
    AtomSpace* atomSpace;
    //AtomTable* atomTable;
    ParserStack stack;
    Handle lastInsertedHandle;
    Status status;
} UserData;

void push(ParserStack& ps, void* p) {
    ps.push(p);
}

void* top(ParserStack& ps) {
    if (!ps.size()) return NULL;
    return ps.top();
}

void* pop(ParserStack& ps) {
    if (!ps.size()) return NULL;
    void* ret = ps.top();
    ps.pop();  
    return ret;
}

Type getTypeForString(const char *name, bool onlyClassName) {
    Type result = ClassServer::getType(name);
    if (result != NOTYPE) {          
        return result;
    }

  if (strcmp(name, POLYGON_CORNER_TOKEN) != 0 && (onlyClassName || strcmp(name, ELEMENT_TOKEN) != 0)){
      MAIN_LOGGER.log(Util::Logger::ERROR, "Warning: null type for name returned! (%s)\n", name);
  }
  
  return 0;
}

char *nativeBuildLinkKey(Atom *link){
    char key[1<<16];
    char aux[1 << 16];

    key[0] = '\0';

    sprintf(aux, "%d ",link->getType());
    strcat(key, aux);

    for (int i = 0; i < link->getArity(); i++){
        sprintf(aux, "%p ", TLB::getHandle(link->getOutgoingAtom(i)));
        strcat(key, aux);
    }

    return strdup(key);
}

//#include <sys/time.h>
//unsigned long int cumulativeParseNodesStart = 0;
//unsigned long int cumulativeParseLinksStart = 0;
//unsigned long int cumulativeParseEnd = 0;
//unsigned long int cumulativeParseEnd1 = 0;
//unsigned long int cumulativeParseEnd2 = 0;

void nativeStartElement(void *userData, const char *name, const char **atts) throw (RuntimeException, InconsistenceException){

    Atom* r = NULL;
    Type typeFound;
    
    UserData* ud = (UserData*) userData;

    // precesses head tags (list, tagdescription, etc)
    if (strcmp(name, LIST_TOKEN) == 0) {
        ud->status.enabled = 1;
        return;
    } else if (strcmp(name, TAG_DESCRIPTION_TOKEN) == 0) {
        ud->status.ignoring = 1;
        return;
    }

    // abandons element if necessary
    if ((!ud->status.enabled) || ud->status.ignoring)
        return;

    typeFound = getTypeForString(name, false);

    // processes nodes
    if (ud->status.processNodes) {
//        timeval s;
//        gettimeofday(&s, NULL);


        if (ClassServer::isAssignableFrom(NODE, typeFound)) {
            //cprintf(5,"Processing Node: %d (%s)\n",  typeFound, name);
            r = (Atom*) new Node(typeFound, "", NMXmlParser::DEFAULT_TV());
            //printf("Pushing r = %p\n", r);
            push(ud->stack, r);
            const TruthValue& t = r->getTruthValue();
            if (typeid(t) != typeid(SimpleTruthValue)){
                throw InconsistenceException(TRACE_INFO, 
                        "DefaultTruthValue is not SimpleTruthValue.");
            }
            while (*atts != NULL) {
                float buffer;
                if (strcmp(*atts, NAME_TOKEN) == 0) {
                    atts++;
                    NMXmlParser::setNodeName((Node*) r, *atts);
                }else if (strcmp(*atts, CONFIDENCE_TOKEN) == 0) {
                    atts++;
                    sscanf(*atts, "%f", &buffer);
                    if (buffer == 1.0){
                        throw RuntimeException(TRACE_INFO, 
                                "fatal error: confidence can not be 1.0");
                    }
                    SimpleTruthValue newTv((const SimpleTruthValue&)r->getTruthValue());
                                        newTv.setConfidence(buffer);
                    r->setTruthValue(newTv);
                } else if (strcmp(*atts, STRENGTH_TOKEN) == 0) {
                    atts++;
                    sscanf(*atts, "%f", &buffer);
                    SimpleTruthValue newTv((const SimpleTruthValue&)r->getTruthValue());
                                        newTv.setMean(buffer);
                    r->setTruthValue(newTv);
                } else{
                    MAIN_LOGGER.log(Util::Logger::ERROR, "unrecognized Node token: %s\n", *atts);
                }
                atts++;
            }
        }
//        timeval e;
//        gettimeofday(&e, NULL);
//        unsigned long spentTime = (e.tv_sec -  s.tv_sec)*1000000+(e.tv_usec -  s.tv_usec);
//        cumulativeParseNodesStart += spentTime;
    }

    // processes relationships
    if (ud->status.processRelationships) {
        // inheritance links may be declared inside lexical category node
        // tags. this information is ignored once it is redundant with source
        // and target data contained in the inheritance link declaration
        if (ClassServer::isAssignableFrom(LINK, typeFound)) {
//            timeval s;
//            gettimeofday(&s, NULL);
            //cprintf(5,"Processing Link: %d (%s)\n", typeFound, name);
            r = (Atom*) new Link(typeFound, std::vector<Handle>(), NMXmlParser::DEFAULT_TV());

            const TruthValue& t = r->getTruthValue();
            if (typeid(t) != typeid(SimpleTruthValue)){
                throw InconsistenceException(TRACE_INFO, 
                        "DefaultTruthValue is not SimpleTruthValue");
            }
            while (*atts != NULL) {
                float buffer;
                if (strcmp(*atts, CONFIDENCE_TOKEN) == 0) {
                    atts++;
                    sscanf(*atts, "%f", &buffer);
                    if (buffer == 1.0){
                        throw RuntimeException(TRACE_INFO, 
                                "fatal error: confidence can not be 1.0");
                    }
                    SimpleTruthValue newTv((const SimpleTruthValue&)r->getTruthValue());
                                        newTv.setConfidence(buffer);
                    r->setTruthValue(newTv);
                } else if (strcmp(*atts, STRENGTH_TOKEN) == 0) {
                    atts++;
                    sscanf(*atts, "%f", &buffer);
                    SimpleTruthValue newTv((const SimpleTruthValue&)r->getTruthValue());
                                        newTv.setMean(buffer);
                    r->setTruthValue(newTv);
                } else {
                    //MAIN_LOGGER.log(Util::Logger::ERROR, "unrecognized Link token: %s\n", *atts);
                }
                atts++;
            }
            Atom* currentAtom = (Atom*) top(ud->stack);
            if (currentAtom != NULL) {
                //printf("Getting link element inside currentAtom = %p\n", currentAtom);
                if (ClassServer::isAssignableFrom(LINK,currentAtom->getType())){
                    if (r != NULL){
                        NMXmlParser::addOutgoingAtom(currentAtom, NULL);
                    }else{
                        throw RuntimeException(TRACE_INFO, "fatal error: NULL inner link");
                    }
                }
            }

            //printf("Pushing r = %p\n", r);
            push(ud->stack, r);
//            timeval e;
//            gettimeofday(&e, NULL);
//            unsigned long spentTime = (e.tv_sec -  s.tv_sec)*1000000+(e.tv_usec -  s.tv_usec);
//            cumulativeParseLinksStart += spentTime;
        } else if (strcmp(name, ELEMENT_TOKEN) == 0) {
            // processes elements of other relationships, and inserts them in
            // the link previously defined;
          
            Atom* currentAtom = (Atom*) top(ud->stack);
            if (currentAtom == NULL) {
                MAIN_LOGGER.log(Util::Logger::ERROR, "error: this token (%s) is expected to be nested\n", name);
                return;
            }
            //printf("Getting node element inside currentAtom = %p\n", currentAtom);
          
          
            const char *n = NULL, *t = NULL;
            while (*atts != NULL) {
                if (strcmp(*atts, NAME_TOKEN) == 0) {
                    atts++;
                    n = *atts;
                }else if (strcmp(*atts, CLASS_TOKEN) == 0) {
                    atts++;
                    t = *atts;
                }else{
                    MAIN_LOGGER.log(Util::Logger::ERROR, "unrecognized token: %s\n", *atts);
                }
                atts++;
            }

            Handle h;
            //r = getRelationshipByNameAndType(n, getTypeForString(t, true));

//            printf("Getting existing node (%s,%s)\n", n, t);
                //h = ud->atomTable->getHandle(n, getTypeForString(t, true));
                h = ud->atomSpace->getHandle(getTypeForString(t, true), n);
//            printf(" => h = %p\n", h);
            if (h != NULL) {
                NMXmlParser::addOutgoingAtom(currentAtom, h);
            } else {
                throw RuntimeException(TRACE_INFO, 
                        "fatal error: unable to find atom named %s, type %s", n, t);
            }
        }
    }
}

void nativeEndElement(void *userData, const char *name) throw (InconsistenceException) {

    UserData* ud = (UserData*) userData;

    if (strcmp(name, LIST_TOKEN) == 0) {
        ud->status.enabled = 0;
    } else if (strcmp(name, TAG_DESCRIPTION_TOKEN) == 0) {
        ud->status.ignoring = 0;
    } else {
        void* object = top(ud->stack);
        Atom* currentAtom = (Atom*) object;
        if (currentAtom != NULL) {
//            timeval s;
//            gettimeofday(&s, NULL);
            Type type = getTypeForString(name, false);
            if (((ClassServer::isAssignableFrom(NODE, type)) && ud->status.processNodes) ||
                (ClassServer::isAssignableFrom(LINK, type)) && ud->status.processRelationships) {
                if (currentAtom->getType() == type) {
                    if (ClassServer::isAssignableFrom(LINK, type)){
                        pop(ud->stack);
                        //printf("(1) Pushing currentAtom = %p\n", currentAtom);
                        push(ud->stack, currentAtom);
                    }
                    if (ClassServer::isAssignableFrom(UNORDERED_LINK, type)) {
                        // Forces the sorting of outgoing by calling setOutgoingSet
                        // TODO: implement a sortOutgoingSet for doing the same thing more efficiently... 
                        std::vector<Handle> outgoing = currentAtom->getOutgoingSet();
                        NMXmlParser::setOutgoingSet(currentAtom, outgoing); 
                    }
                    Handle oldHandle = TLB::getHandle(currentAtom);
//                    timeval s1;
//                    gettimeofday(&s1, NULL);
                            //printf("currentAtom => %s\n", currentAtom->toString().c_str());
                    //Handle newHandle = ud->atomTable->add(currentAtom);
                    Handle newHandle = ud->atomSpace->addRealAtom(*currentAtom);
//                    timeval e1;
//                    gettimeofday(&e1, NULL);
//                    unsigned long spentTime1 = (e1.tv_sec -  s1.tv_sec)*1000000+(e1.tv_usec -  s1.tv_usec);
//                    //unsigned long spentTime1 = (e1.tv_sec -  s.tv_sec)*1000000+(e1.tv_usec -  s.tv_usec);
//                    cumulativeParseEnd1 += spentTime1;
                    // Updates last inserted/merged atom handle
                    ud->lastInsertedHandle = newHandle;
                    if (CoreUtils::handleCompare(&oldHandle, &newHandle)){
                        // already existed
                        delete currentAtom;
                        //printf("Already existed\n");
                        currentAtom = TLB::getAtom(newHandle);
                        pop(ud->stack);
                        //printf("(2) Pushing currentAtom = %p\n", currentAtom);
                        push(ud->stack, currentAtom);
                    }
                    if (ClassServer::isAssignableFrom(LINK, currentAtom->getType())) {
                        //KMI -- find out if this is a nested link
                        pop(ud->stack);
                        Atom* nextUd = (Atom*) top(ud->stack);
                        //printf("Getting link element inside nextUd = %p\n", nextUd);
                        //printf("(3) Pushing currentAtom = %p\n", currentAtom);
                        push(ud->stack, currentAtom);
                        if ((nextUd != NULL)&&(ClassServer::isAssignableFrom(LINK, nextUd->getType()))){
                            int arity = nextUd->getArity();
                            std::vector<Handle> outgoingSet = nextUd->getOutgoingSet();
                            for (int i = 0; i < arity; i++){
                                if (outgoingSet[i] == NULL){
                                    outgoingSet[i] = newHandle;
                                    break;
                                }
                            }
                            NMXmlParser::setOutgoingSet(nextUd, outgoingSet);
                        }
                    }
                } else {
                    throw InconsistenceException(TRACE_INFO, 
                            "fatal error: relationship type mismatch.");
                }
                pop(ud->stack);
            }
//            timeval e;
//            gettimeofday(&e, NULL);
//            unsigned long spentTime = (e.tv_sec -  s.tv_sec)*1000000+(e.tv_usec -  s.tv_usec);
//            cumulativeParseEnd += spentTime;

        //} else {
        //    cprintf(NORMAL, "WARNING: Got NULL Atom* from the parser stack!\n");
        }
    }

}

NMXmlParser::NMXmlParser(AtomSpace* atomSpace,  bool fresh, bool freshLinks) {
    this->atomSpace = atomSpace;
    this->fresh = fresh;
    this->freshLinks = freshLinks;
}

NMXmlParser::~NMXmlParser() {
}

void NMXmlParser::setNodeName(Node* node, const char* name) {
    node->setName(name);
}

void NMXmlParser::addOutgoingAtom(Atom* atom, Handle h) {
    atom->addOutgoingAtom(h);
}

void NMXmlParser::setOutgoingSet(Atom* atom, const std::vector<Handle>& outgoing) {
    atom->setOutgoingSet(outgoing);
}

HandleEntry* 
NMXmlParser::loadXML(const std::vector<XMLBufferReader*>& xmlReaders,
                     AtomSpace * atomSpace,
                     bool fresh,
                     bool freshLinks)
{
    //printf("NMXmlParser::loadXML\n");
    cassert(TRACE_INFO, atomSpace != NULL,
        "loadXML - atomSpace should pointer should not be NULL.");
    HandleEntry* result = NULL;

    if (xmlReaders.size() <= 0) return result;

    NMXmlParser parser(atomSpace, fresh, freshLinks);

//    time_t start = time(NULL);
//        timeval s;
//        gettimeofday(&s, NULL);
    // Only nodes are processed in the first pass
    for (unsigned int i = 0; i < xmlReaders.size(); i++) {
        if (typeid(*xmlReaders[i]) == typeid(FileXMLBufferReader)) {
            MAIN_LOGGER.log(Util::Logger::DEBUG,
                            "First pass: processing file %s\n",
                            ((FileXMLBufferReader*) xmlReaders[i])->getFilename());
        }
        //MAIN_LOGGER.log(Util::Logger::WARNING, "Loading XML: %d%% done.\r", (int) (100 * ((float) i / (size * 2))));
        // fflush(stdout);
        parser.parse(xmlReaders[i], PARSE_NODES);
    }
        //timeval e;
        //gettimeofday(&e, NULL);
        //unsigned long spentTime = (e.tv_sec*1000 + e.tv_usec/1000) - (s.tv_sec*1000 + s.tv_usec/1000);
        //printf("PARSE NODES time = %lu\n", spentTime);
        //s = e;

    // only links are processed in the second pass
    for (unsigned int i = 0; i < xmlReaders.size(); i++) {
        if (typeid(*xmlReaders[i]) == typeid(FileXMLBufferReader)) {
            MAIN_LOGGER.log(Util::Logger::DEBUG, 
                            "Second pass: processing file %s\n", 
                            ((FileXMLBufferReader*) xmlReaders[i])->getFilename());
        }
        // MAIN_LOGGER.log(Util::Logger::WARNING, "Loading XML: %d%% done.\r", (int) (100 * ((float) (i + size) / (size * 2))));
        // fflush(stdout);
        Handle lastInsertedLinkHandle = parser.parse(xmlReaders[i], PARSE_LINKS);
        if (CoreUtils::handleCompare(&lastInsertedLinkHandle, &UNDEFINED_HANDLE)) {
            result = HandleEntry::concatenation(result, new HandleEntry(lastInsertedLinkHandle));
        }
    }
//    time_t duration = time(NULL) - start;
//        timeval e;
//        gettimeofday(&e, NULL);
//        unsigned long spentTime = (e.tv_sec-s.tv_sec)*1000000 + (e.tv_usec-s.tv_usec);
//        printf("PARSE XML time = %lu\n", spentTime);

//    MAIN_LOGGER.log(Util::Logger::WARNING, "Loading XML contents: 100%% done (in %d second%c).\n", (int) duration, duration == 1 ? '\0' : 's');
    //cprintf(NORMAL, "Number of timestamp entries: %d\n", stackTimeFlag);
    return result;
}

Handle NMXmlParser::parse_pass(XMLBufferReader* xmlReader, NMXmlParseType pass)
{
    char buf[BUFSIZ];
    int done;
    UserData userData;

    userData.lastInsertedHandle = UNDEFINED_HANDLE;
    //userData.atomTable = atomSpace->getAtomTable();
    userData.atomSpace = atomSpace;

    userData.status.enabled = 0;
    userData.status.ignoring = 0;

    if (pass == PARSE_NODES) {
        userData.status.processNodes = 1;
        userData.status.processRelationships = 0;
    } else {
        userData.status.processNodes = 0;
        userData.status.processRelationships = 1;
    }
    
    XML_Parser parser = XML_ParserCreate(NULL);
    XML_SetUserData(parser, &userData);
    XML_SetElementHandler(parser, nativeStartElement, nativeEndElement);

    xmlReader->open();
    do {
        size_t len = xmlReader->read(buf, 1, sizeof(buf));
        done = len < sizeof(buf);
        if (!XML_Parse(parser, buf, len, done)) {
            fprintf(stderr,
                    "%s at line %d\n",
                    XML_ErrorString(XML_GetErrorCode(parser)),
                    XML_GetCurrentLineNumber(parser));
            userData.lastInsertedHandle = UNDEFINED_HANDLE;
            break;
        }
    } while (!done);
    xmlReader->close();

    XML_ParserFree(parser);
    return userData.lastInsertedHandle;
}

Handle NMXmlParser::parse(XMLBufferReader* xmlReader, NMXmlParseType pass)
{
    Handle h = UNDEFINED_HANDLE;
    if (pass == PARSE_NODES) {

        MAIN_LOGGER.log(Util::Logger::DEBUG, "Parsing nodes...\n");
        
        // FIRST PASS - creates relationships with arity == 0 (nodes)
        h = parse_pass(xmlReader, pass);
        if (h == UNDEFINED_HANDLE) return UNDEFINED_HANDLE;
        
    } else if (pass == PARSE_LINKS) {

        MAIN_LOGGER.log(Util::Logger::DEBUG, "Parsing links...\n");
        
        // SECOND PASS - creates other relationships
        // second pass must be avoided once subgraph insertion and/or lazy
        // insertion is implemented
        h = parse_pass(xmlReader, pass);
    }
    return h;
}
