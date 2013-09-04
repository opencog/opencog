/*
 * opencog/xml/StringXMLBufferReader.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 *
 * Written by Rodrigo Barra
 * All Rights Reserved
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

#include "NMXmlParserExperiment.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cxxtest/TestSuite.h>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/platform.h>

using namespace std;
using namespace opencog;

#ifdef WIN32
#include <fcntl.h>
#include <io.h>

#define open _open
#define write _write
#define close _close

#define _S_IREAD 256
#define _S_IWRITE 128
int mkstemp(char *tmpl)
{
    int ret = -1;
    _mktemp(tmpl);
    ret = open(tmpl, O_RDWR | O_BINARY | O_CREAT | O_EXCL | _O_SHORT_LIVED, _S_IREAD | _S_IWRITE);
    return ret;
}
#endif

bool NMXmlParserExperiment::noCheck             = false;
char *NMXmlParserExperiment::currentFileName    = NULL;
int NMXmlParserExperiment::currentExperiment    = -1;

Handle NMXmlParserExperiment::one               = Handle::UNDEFINED;
Handle NMXmlParserExperiment::two               = Handle::UNDEFINED;
Handle NMXmlParserExperiment::link_one_two      = Handle::UNDEFINED;
Handle NMXmlParserExperiment::hihger_order_link = Handle::UNDEFINED;
AtomSpace* NMXmlParserExperiment::atomSpace     = NULL;

void NMXmlParserExperiment::initStaticVars()
{
    NMXmlParserExperiment::noCheck = false;
    NMXmlParserExperiment::currentFileName = NULL;
    NMXmlParserExperiment::currentExperiment = -1;

    NMXmlParserExperiment::one = Handle::UNDEFINED;
    NMXmlParserExperiment::two = Handle::UNDEFINED;
    NMXmlParserExperiment::link_one_two = Handle::UNDEFINED;
    NMXmlParserExperiment::hihger_order_link = Handle::UNDEFINED;
    NMXmlParserExperiment::atomSpace = NULL;
}

int NMXmlParserExperiment::getNExperiments()
{
    return(NNMXMLXMLEXPERIMENTS);
}

void NMXmlParserExperiment::createExperiment(int exp, AtomSpace* as)
{
    if ((exp < 0) || (exp > NNMXMLXMLEXPERIMENTS)) {
        throw new RuntimeException(TRACE_INFO, "Invalid Experiment\n");
    }

    if (currentExperiment != -1) {
        throw new RuntimeException(TRACE_INFO, "Tried to start a new experiment without destroying the last one");
    }

    currentExperiment = exp;

    //currentFileName = strdup("/var/tmp/xmltest.XXXXXX");
    currentFileName = strdup("xmltest.XXXXXX");
    printf("just allocated currentFileName = %s\n", currentFileName);

    int fd = mkstemp(currentFileName);
    printf("after mkstemp currentFileName = %s\n", currentFileName);

    if (fd == -1) {
        throw new RuntimeException(TRACE_INFO, "Could not create temporary file\n");
    }

    ssize_t rc = write(fd, expContents[exp%2], strlen(expContents[exp%2]));
    if (rc != (ssize_t) strlen(expContents[exp%2]))
        throw new RuntimeException(TRACE_INFO, "Failed to write");

    close(fd);

    if (atomSpace) delete atomSpace;
    atomSpace = as;
}

bool NMXmlParserExperiment::checkExperiment()
{
    if (noCheck) {
        return (true);
    }
    if (currentExperiment == -1) {
        return(true);
    }
    switch (currentExperiment) {
    case 0:
        return(checkExp0());
        break;
    case 1:
        return(checkExp1());
        break;
    }
    return(false);
}

AtomSpace* NMXmlParserExperiment::destroyExperiment(bool cleanup)
{
    if (currentExperiment == -1) {
        return atomSpace;
    }
    printf("currentFileName = %s\n", currentFileName);
    remove(currentFileName);
#ifndef WIN32 // For some reason, this causes error on Windows
    free(currentFileName);
#endif
    currentFileName = NULL;
    currentExperiment = -1;
    if (cleanup) {
        cleanupAtomSpace();
    }
    return atomSpace;
}

AtomSpace* NMXmlParserExperiment::cleanupAtomSpace()
{
    delete atomSpace;
    atomSpace = new AtomSpace();
    return atomSpace;
}

AtomSpace* NMXmlParserExperiment::getAtomSpace()
{
    return atomSpace;
}

bool NMXmlParserExperiment::checkExp0()
{
    one = atomSpace->getHandle(NUMBER_NODE, "1");
    two = atomSpace->getHandle(NUMBER_NODE, "2");

    TS_ASSERT(atomSpace->isValidHandle(one));
    TS_ASSERT(atomSpace->isValidHandle(two));
    if (!atomSpace->isValidHandle(one) || !atomSpace->isValidHandle(two)) {
        return(false);
    }

    std::vector<Handle> handles;
    atomSpace->getHandleSet(back_inserter(handles), INHERITANCE_LINK, true);

    TS_ASSERT(handles.size() == 1);
    if (handles.size() != 1) {
        return(false);
    }
    link_one_two = handles[0];

    TS_ASSERT((atomSpace->getOutgoing(link_one_two)[0]) == two);
    TS_ASSERT((atomSpace->getOutgoing(link_one_two)[1]) == one);
    if ((atomSpace->getOutgoing(link_one_two)[0] != two) ||
            (atomSpace->getOutgoing(link_one_two)[1] != one)) {
        return(false);
    }

    return(true);
}

bool NMXmlParserExperiment::checkExp1()
{

    one = atomSpace->getHandle(NUMBER_NODE, "1");
    two = atomSpace->getHandle(NUMBER_NODE, "2");

    TS_ASSERT(atomSpace->isValidHandle(one));
    TS_ASSERT(atomSpace->isValidHandle(two));
    if (!atomSpace->isValidHandle(one) || !atomSpace->isValidHandle(two)) {
        return(false);
    }

    std::vector<Handle> handles;
    atomSpace->getHandleSet(back_inserter(handles), INHERITANCE_LINK, true);
    TS_ASSERT(handles.size() == 0);

    atomSpace->getHandleSet(back_inserter(handles), INHERITANCE_LINK, true);
    TS_ASSERT(handles.size() == 2);
    if (handles.size() != 2) {
        return(false);
    }

    link_one_two = Handle::UNDEFINED;
    std::vector<Handle>::iterator it;
    for (it = handles.begin(); it != handles.end(); it++) {
        if (atomSpace->getIncoming(*it).size() == 1) {
            TS_ASSERT(!atomSpace->isValidHandle(link_one_two));
            link_one_two = *it;
        }
    }
    handles.clear();

    TS_ASSERT(atomSpace->isValidHandle(link_one_two));
    TS_ASSERT((atomSpace->getOutgoing(link_one_two)[0]) == two);
    TS_ASSERT((atomSpace->getOutgoing(link_one_two)[1]) == one);
    if ((atomSpace->getOutgoing(link_one_two)[0] != two) ||
            (atomSpace->getOutgoing(link_one_two)[1] != one)) {
        return(false);
    }

    atomSpace->getHandleSet(back_inserter(handles), MEMBER_LINK, true);


    TS_ASSERT(handles.size() == 1);
    if (handles.size() != 1) {
        return(false);
    }
    //atom = TLB::getAtom(handles[0]);
    hihger_order_link = handles[0];

    //link = dynamic_cast<Link *>(atom);
    TS_ASSERT(atomSpace->getOutgoing(handles[0])[0] == link_one_two);
    TS_ASSERT(atomSpace->getOutgoing(handles[0])[1] == two);
    if ((atomSpace->getOutgoing(handles[0])[0] != link_one_two) ||
            (atomSpace->getOutgoing(handles[0])[1] != two)) {
        return(false);
    }

    /*
     entry = atomSpace->getAtomTable()->getHandleSet(INHERITANCE_LINK, true);

     TS_ASSERT(entry->getSize() == 2);
     if (entry->getSize() != 2){
      delete entry;
      return(false);
     }

     link_one_two = NULL;
     Atom *atom = NULL;

     HandleEntry *it = entry;
     while (it){
      atom = it->getAtom();
      if (atom->getIncomingSet()->getSize() == 1){
       TS_ASSERT(TLB::isInvalidHandle(link_one_two));
       link_one_two = it->handle;
      }
      it = it->next;
     }
     delete entry;

     TS_ASSERT(link_one_two != NULL);

     TS_ASSERT(atom->getOutgoingSet()[0]->getHandleSet() == two);
     TS_ASSERT(atom->getOutgoingSet()[1]->getHandleSet() == one);
     if ((atom->getOutgoingSet()[0] != two) ||
      (atom->getOutgoingSet()[1] != one)){
      return(false);
     }

     entry = atomSpace->getAtomTable()->getHandleSet(MEMBER_LINK, true);


     TS_ASSERT(entry->getSize() == 1);
     if (entry->getSize() != 1){
      delete entry;
      return(false);
     }
     atom = entry->getAtom();
     hihger_order_link = entry->handle;
     delete entry;

     TS_ASSERT(atom->getOutgoingSet()[0] == link_one_two);
     TS_ASSERT(atom->getOutgoingSet()[1] == two);
     if ((atom->getOutgoingSet()[0] != link_one_two) ||
      (atom->getOutgoingSet()[1] != two)){
      return(false);
     }
    */
    return(true);
}


const char *NMXmlParserExperiment::expContents[NNMXMLXMLEXPERIMENTS] = {
    "<?xml version=\"1.0\"?>                                                 \
    <list>                                                                   \
        <tagdescription>                                                     \
            <tag name=\"NumberNode\" value=\"NumberNode\"/>                  \
            <tag name=\"InheritanceLink\" value=\"InheritanceLink\"/>        \
        </tagdescription>                                                    \
                                                                             \
        <NumberNode name=\"1\" timestamp=\"3422826\"/>                       \
        <NumberNode name=\"2\" timestamp=\"3422826\"/>                       \
                                                                             \
        <InheritanceLink hyp=\"hyp_1\" strength=\"1.0\" confidence=\"0.95\"> \
            <Element name=\"2\" class=\"NumberNode\"/>                       \
            <Element name=\"1\" class=\"NumberNode\"/>                       \
        </InheritanceLink>                                                   \
                                                                             \
        <InheritanceLink hyp=\"hyp_1\" strength=\"1.0\" confidence=\"0.95\"> \
            <Element name=\"2\" class=\"NumberNode\"/>                       \
            <Element name=\"1\" class=\"NumberNode\"/>                       \
        </InheritanceLink>                                                   \
    </list>"
    /* UNCOMMENT TO USE HYPOTHETICAL ATOMTABLE
    ,
    "<?xml version=\"1.0\"?> \
    <list> \
    <tagdescription> \
    <tag name=\"NumberNode\" value=\"NumberNode\"/> \
    <tag name=\"InheritanceLink\" value=\"InheritanceLink\"/> \
    <tag name=\"MemberLink\" value=\"MemberLink\"/> \
    </tagdescription> \
    <NumberNode name=\"1\" timestamp=\"3422827\"/> \
    <NumberNode name=\"2\" timestamp=\"3422827\"/> \
    <MemberLink strength=\"0.50\" confidence=\"0.80\"> \
      <InheritanceLink hyp=\"hyp_1\" strength=\"1.0\" confidence=\"0.95\"> \
        <Element name=\"2\" class=\"NumberNode\"/> \
        <Element name=\"1\" class=\"NumberNode\"/> \
      </InheritanceLink> \
      <Element name=\"2\" class=\"NumberNode\"/> \
    </MemberLink> \
    <InheritanceLink hyp=\"hyp_2\" strength=\"1.0\" confidence=\"0.95\"> \
      <Element name=\"two\" class=\"NumberNode\"/> \
      <Element name=\"one\" class=\"NumberNode\"/> \
    </InheritanceLink> \
    </list>"
    */
};
