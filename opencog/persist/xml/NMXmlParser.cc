/*
 * opencog/xml/NMXmlParser.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#include <stack>
#include <string>

#include <expat.h>
#include <math.h>

#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/platform.h>
#include <opencog/persist/xml/FileXMLBufferReader.h>
#include <opencog/persist/xml/NMXmlDefinitions.h>
#include <opencog/persist/xml/NMXmlParser.h>

/*
 * XXX To be fixed: remove all of the uses of "throw" in this code,
 * to be replaced by a gentler error mechanism. The problem is two-fold:
 * the "throws" are made from within routines that are called by the
 * external libxmlparser library, which is written in C, not C++.
 * Thus, these errors can't ever actually be caught, since they never
 * get through the parser. As a result, any minor mistake in an XML
 * will cause the entire server to crash, which is just plain ugly.
 * Yuck!

 * NOTE: Any time information (including Timestamps) are represented using Atoms as follows:
 * AtTimeLink(TimeNode:<temporalStringValue>, Atom1 [, Atom2 [... , AtomN]])
 */

using namespace opencog;

boost::unordered_map<const std::string, Handle, boost::hash<std::string>, std::equal_to<std::string> > NMXmlParser::hypHandles;
bool NMXmlParser::fresh = true;
bool NMXmlParser::freshLinks = false;

// If unspecified, the default truth value is "true", with confidence of 0.5
const TruthValue& NMXmlParser::DEFAULT_TV()
{
    static SimpleTruthValue* instance = NULL;
    if (instance == NULL) {
        instance = new SimpleTruthValue(1.0f, 1.0f);
        instance->setConfidence(0.5);
    }
    return *instance;
}

// const int TYPE_LENGTH = (int) log10((double)NUMBER_OF_CLASSES) + 1;

typedef struct {
unsigned enabled: 1;
unsigned ignoring: 1;
unsigned processNodes: 1;
unsigned processRelationships: 1;
} Status;

typedef std::stack< AtomPtr > ParserStack;
// TODO: CREATE AN ABSTRACT CLASS "Stackable" for check for subclasses
// when poping elements from stack.

typedef struct {
    AtomSpace* atomSpace;
    //AtomTable* atomTable;
    ParserStack stack;
    Handle lastInsertedHandle;
    Status status;
    XML_Parser parser;
} UserData;

static void push(ParserStack& ps, AtomPtr p)
{
    ps.push(p);
}

static AtomPtr top(ParserStack& ps)
{
    if (!ps.size()) return NULL;
    return ps.top();
}

static AtomPtr pop(ParserStack& ps)
{
    if (!ps.size()) return NULL;
    AtomPtr ret = ps.top();
    ps.pop();
    return ret;
}

/**
 * Return the type corresponding to the type name
 */
static Type getTypeFromString(const char *name, bool onlyClassName)
{
    if (!name) return NOTYPE;

    Type result = classserver().getType(name);
    if (result != NOTYPE) {
        return result;
    }

    if (strcmp(name, POLYGON_CORNER_TOKEN) != 0 &&
            (onlyClassName || strcmp(name, ELEMENT_TOKEN) != 0)) {
        logger().error("Warning: null type for name returned! (%s)\n", name);
    }

    return 0;
}

/**
 * Handle attributes that are common to both Nodes and Links.
 * In other words, handle "confidence" and "strength" attributes.
 */
static const char ** scan_common_attrs (AtomPtr r, const char **atts)
{
    float buffer;
    if (strcmp(*atts, CONFIDENCE_TOKEN) == 0) {
        atts++;
        sscanf(*atts, "%f", &buffer);
        SimpleTruthValue newTv((const SimpleTruthValue&)r->getTruthValue());
        newTv.setConfidence(buffer);
        r->setTruthValue(newTv);
    } else if (strcmp(*atts, STRENGTH_TOKEN) == 0) {
        atts++;
        sscanf(*atts, "%f", &buffer);
        SimpleTruthValue newTv((const SimpleTruthValue&)r->getTruthValue());
        newTv.setMean(buffer);
        r->setTruthValue(newTv);
    }
    return atts;
}

static void nativeStartElement(void *userData, const char *name, const char **atts)
throw (RuntimeException, InconsistenceException)
{
    UserData* ud = (UserData*) userData;

    // processes head tags (list, tagdescription, etc)
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

    Type typeFound = getTypeFromString(name, false);
    if (NOTYPE == typeFound) return;

    // processes nodes
    if (ud->status.processNodes) {
//        timeval s;
//        gettimeofday(&s, NULL);

        if (classserver().isA(typeFound, NODE)) {
            //cprintf(5,"Processing Node: %d (%s)\n",  typeFound, name);

            std::string sname;
            AtomPtr junk = createNode(typeFound, "", NMXmlParser::DEFAULT_TV());
            while (*atts != NULL) {
                if (strcmp(*atts, NAME_TOKEN) == 0) {
                    atts++;
                    sname = *atts;
                } else {
                    const char **natts = scan_common_attrs(junk, atts);
                    if (atts == natts) {
                        logger().error("unrecognized Node token: %s\n", *atts);
                    }
                    atts = natts;
                }
                atts++;
            }
            AtomPtr r = createNode(typeFound, sname, junk->getTruthValue());
            logger().fine("Pushing r = %p", r.get());
            push(ud->stack, r);
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
        if (classserver().isA(typeFound, LINK)) {
//            timeval s;
//            gettimeofday(&s, NULL);
            logger().fine("Processing Link: %d (%s)\n", typeFound, name);

            AtomPtr junk = createLink(typeFound, HandleSeq(), NMXmlParser::DEFAULT_TV());
            while (*atts != NULL) {
                const char **natts = scan_common_attrs(junk, atts);
                if (atts == natts) {
                    logger().error("unrecognized Link token: %s\n", *atts);
                }
                atts = natts;
                atts++;
            }

            AtomPtr currentAtom = top(ud->stack);
            if (currentAtom) {
                logger().fine("Getting link element inside currentAtom = %p", currentAtom.get());
                LinkPtr link(LinkCast(currentAtom));
                if (link) {
/// I don't get it .. when is this ever fixed with the correct handle value ... ???
                    NMXmlParser::addOutgoingAtom(link, Handle::UNDEFINED);
                }
            }

            AtomPtr r = createLink(typeFound, std::vector<Handle>(), junk->getTruthValue());
            logger().fine("Pushing r = %p", r.get());
            push(ud->stack, r);
//            timeval e;
//            gettimeofday(&e, NULL);
//            unsigned long spentTime = (e.tv_sec -  s.tv_sec)*1000000+(e.tv_usec -  s.tv_usec);
//            cumulativeParseLinksStart += spentTime;
        } else if (strcmp(name, ELEMENT_TOKEN) == 0) {
            // processes elements of other relationships, and inserts them in
            // the link previously defined;

            const char *n = NULL, *t = NULL;
            while (*atts != NULL) {
                if (strcmp(*atts, NAME_TOKEN) == 0) {
                    atts++;
                    n = *atts;
                } else if (strcmp(*atts, CLASS_TOKEN) == 0) {
                    atts++;
                    t = *atts;
                } else {
                    logger().error("unrecognized token: %s\n", *atts);
                }
                atts++;
            }

            AtomPtr currentAtom = top(ud->stack);
            if (!currentAtom) {
                logger().error("error: this token (%s) is expected to be nested\n", name);
                return;
            }
            logger().fine("Getting node element inside currentAtom = %p", currentAtom.get());

            logger().fine("Getting existing node (%s,%s)", n, t);
            Handle h = ud->atomSpace->getHandle(getTypeFromString(t, true), n);
            logger().fine(" => h = %p", h.value());
            if (ud->atomSpace->isValidHandle(h)) {
                logger().fine(ud->atomSpace->atomAsString(h).c_str());
                LinkPtr link(LinkCast(currentAtom));
                if (link) {
                    logger().fine("adding atom %s to link %s", ud->atomSpace->atomAsString(h,true).c_str(), link->toShortString().c_str());
                    NMXmlParser::addOutgoingAtom(link, h);
                }
            } else {
#ifdef THROW_EXCEPTIONS
                throw RuntimeException(TRACE_INFO,
                                       "fatal error: unable to find atom named %s, type %s", n, t);
#else
                /* Don't just core dump on bad XML format.
                * (since exceptions can't be thrown past the C library
                * of the parser, a core dump is inevitable).
                */
                fprintf (stderr,
                         "Fatal error: unable to find atom named %s, type %s\n", n, t);
                logger().error(
                    "fatal error: unable to find atom named %s, type %s\n", n, t);
#endif
            }
        }
    }
}

static void nativeEndElement(void *userData, const char *name)
throw (InconsistenceException)
{
    UserData* ud = (UserData*) userData;

    if (strcmp(name, LIST_TOKEN) == 0) {
        ud->status.enabled = 0;
    } else if (strcmp(name, TAG_DESCRIPTION_TOKEN) == 0) {
        ud->status.ignoring = 0;
    } else {
        AtomPtr currentAtom = top(ud->stack);
        if (currentAtom) {
            //timeval s;
            //gettimeofday(&s, NULL);
            Type type = getTypeFromString(name, false);
            if ((classserver().isA(type, NODE) && ud->status.processNodes) ||
                (classserver().isA(type, LINK) && ud->status.processRelationships)) {
                if (currentAtom->getType() == type) {
                    if (classserver().isA(type, LINK)) {
                        pop(ud->stack);
                        logger().fine("(1) Pushing currentAtom = %p", currentAtom.get());
                        push(ud->stack, currentAtom);
                    }
                    if (classserver().isA(type, UNORDERED_LINK)) {
                        // Forces the sorting of outgoing by calling setOutgoingSet
                        // TODO: implement a sortOutgoingSet for doing the same thing more efficiently...
                        LinkPtr link(LinkCast(currentAtom));
                        if (link) {
                            const HandleSeq& outgoing = link->getOutgoingSet();
                            NMXmlParser::setOutgoingSet(link, outgoing);
                        }
                    }
                    Handle oldHandle =  currentAtom->getHandle();
                    //timeval s1;
                    //gettimeofday(&s1, NULL);
                    logger().fine("currentAtom => %s", currentAtom->toString().c_str());
                    Handle newHandle = ud->atomSpace->addRealAtom(*currentAtom);
                    //timeval e1;
                    //gettimeofday(&e1, NULL);
                    //signed long spentTime1 = (e1.tv_sec -  s1.tv_sec)*1000000+(e1.tv_usec -  s1.tv_usec);
                    //unsigned long spentTime1 = (e1.tv_sec -  s.tv_sec)*1000000+(e1.tv_usec -  s.tv_usec);
                    //cumulativeParseEnd1 += spentTime1;
                    // Updates last inserted/merged atom handle
                    ud->lastInsertedHandle = newHandle;
                    if (Handle::compare(oldHandle, newHandle)) {
                        // already existed -- replace old with new
                        logger().fine("Already existed");
                        currentAtom = ud->atomSpace->cloneAtom(newHandle);
                        pop(ud->stack);
                        logger().fine("(2) Pushing currentAtom = %p", currentAtom.get());
                        push(ud->stack, currentAtom);
                    }
                    // XXX FIXME:
                    // would be better if this was replaced by
                    // if (NULL != boost::shared_dynamic_cast<Link>(currentAtom))
                    // and also a few lines down.
                    if (classserver().isA(currentAtom->getType(), LINK)) {
                        //KMI -- find out if this is a nested link
                        pop(ud->stack);
                        AtomPtr nextUd = top(ud->stack);
                        logger().fine("Getting link element inside nextUd = %p", nextUd.get());
                        logger().fine("(3) Pushing currentAtom = %p", currentAtom.get());
                        push(ud->stack, currentAtom);

                        LinkPtr nextlink(LinkCast(nextUd));
                        if (nextlink) {
                            Arity arity = nextlink->getArity();
                            HandleSeq outgoingSet = nextlink->getOutgoingSet();
                            for (Arity i = 0; i < arity; i++) {
                                if (!ud->atomSpace->isValidHandle(outgoingSet[i])) {
                                    outgoingSet[i] = newHandle;
                                    break;
                                }
                            }
                            NMXmlParser::setOutgoingSet(nextlink, outgoingSet);
                        }
                    }
                } else {
                    char buff[300];
                    snprintf(buff, 300,
                             "XML parse error: relationship type mismatch at line %d.\n"
                             "\txml type=%s(%d) current atom type=%d %s",
                             (int) XML_GetCurrentLineNumber(ud->parser),
                             name, type, currentAtom->getType(), currentAtom->toString().c_str());
                    throw InconsistenceException(TRACE_INFO, buff);
                }
                pop(ud->stack);
            }
            //timeval e;
            //gettimeofday(&e, NULL);
            //unsigned long spentTime = (e.tv_sec -  s.tv_sec)*1000000+(e.tv_usec -  s.tv_usec);
            //cumulativeParseEnd += spentTime;
            else {
                logger().fine("WARNING: Got NULL AtomPtr from the parser stack!");
            }
        }
    }
}

NMXmlParser::NMXmlParser(AtomSpace* atomSpace,  bool fresh, bool freshLinks)
{
    this->atomSpace = atomSpace;
    this->fresh = fresh;
    this->freshLinks = freshLinks;
}

NMXmlParser::~NMXmlParser()
{
}

void NMXmlParser::addOutgoingAtom(LinkPtr link, Handle h)
{
    link->addOutgoingAtom(h);
}

void NMXmlParser::setOutgoingSet(LinkPtr link, const HandleSeq& outgoing)
{
    link->setOutgoingSet(outgoing);
}


HandleSeq
NMXmlParser::loadXML(const std::vector<XMLBufferReader*>& xmlReaders,
                     AtomSpace * atomSpace,
                     bool fresh,
                     bool freshLinks)
{
    logger().fine("NMXmlParser::loadXML");
    OC_ASSERT(atomSpace != NULL,
            "loadXML - atomSpace should pointer should not be NULL.");
    HandleSeq result;

    if (xmlReaders.size() <= 0) return result;

    NMXmlParser parser(atomSpace, fresh, freshLinks);

//    time_t start = time(NULL);
//        timeval s;
//        gettimeofday(&s, NULL);
    // Only nodes are processed in the first pass
    for (unsigned int i = 0; i < xmlReaders.size(); i++) {
        if (typeid(*xmlReaders[i]) == typeid(FileXMLBufferReader)) {
            logger().fine("First pass: processing file %s\n",
                           ((FileXMLBufferReader*) xmlReaders[i])->getFilename());
        }
        // please don't log -- this will spew millions of messages when
        // there are millions of files to be loaded, as would be the 
        // case when the xml is auto-generated and piped over from
        // another process.
        // logger().warn("Loading XML: %d%% done.\r", (int) (100 * ((float) i / (xmlReaders.size() * 2))));
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
            logger().fine("Second pass: processing file %s\n",
                           ((FileXMLBufferReader*) xmlReaders[i])->getFilename());
        }
        // logger().warn("Loading XML: %d%% done.\r", (int) (100 * ((float) (i + xmlReaders.size()) / (xmlReaders.size() * 2))));
        Handle lastInsertedLinkHandle = parser.parse(xmlReaders[i], PARSE_LINKS);
        if (lastInsertedLinkHandle != Handle::UNDEFINED) {
            result.push_back(lastInsertedLinkHandle);
        }
    }

    // finally, null names of unnamed nodes
    // Do we need this? I would expect not because we shouldn't be able to
    // change the node name.
    /*for (HandleEntry *e = result; e; e = e->next)
    {
        if (classserver().isNode(atomSpace->getType(e->handle)))
        {
            if (atomSpace->getName(e->handle)[0] == '#') {
                xxxxx n->setName(""); xxxx
            }
        }
    }*/

    //time_t duration = time(NULL) - start;
    //timeval e;
    //gettimeofday(&e, NULL);
    //unsigned long spentTime = (e.tv_sec-s.tv_sec)*1000000 + (e.tv_usec-s.tv_usec);
    //printf("PARSE XML time = %lu\n", spentTime);

    //logger().warn("Loading XML contents: 100%% done (in %d second%c).\n", (int) duration, duration == 1 ? '\0' : 's');
    //cprintf(NORMAL, "Number of timestamp entries: %d\n", stackTimeFlag);
    return result;
}

Handle NMXmlParser::parse_pass(XMLBufferReader* xmlReader, NMXmlParseType pass)
{
    char buf[BUFSIZ];
    int done;
    UserData userData;

    userData.lastInsertedHandle = Handle::UNDEFINED;
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
    userData.parser = parser;
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
                    (int) XML_GetCurrentLineNumber(parser));
            fprintf(stderr, "\tBuffer was:\n");
            fflush(stderr);
            fwrite(buf, 1, len, stderr);
            fflush(stderr);
            fprintf(stderr, "\n");
            fflush(stderr);
            userData.lastInsertedHandle = Handle::UNDEFINED;
            break;
        }
    } while (!done);
    xmlReader->close();

    XML_ParserFree(parser);
    return userData.lastInsertedHandle;
}

Handle NMXmlParser::parse(XMLBufferReader* xmlReader, NMXmlParseType pass)
{
    Handle h = Handle::UNDEFINED;
    if (pass == PARSE_NODES) {

        logger().fine("Parsing nodes...\n");

        // FIRST PASS - creates relationships with arity == 0 (nodes)
        h = parse_pass(xmlReader, pass);
        if (h == Handle::UNDEFINED) return Handle::UNDEFINED;

    } else if (pass == PARSE_LINKS) {

        logger().fine("Parsing links...\n");

        // SECOND PASS - creates other relationships
        // second pass must be avoided once subgraph insertion and/or lazy
        // insertion is implemented
        h = parse_pass(xmlReader, pass);
    }
    return h;
}
