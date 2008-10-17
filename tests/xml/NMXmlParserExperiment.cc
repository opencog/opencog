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
#include <opencog/atomspace/TLB.h>
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

bool NMXmlParserExperiment::noCheck = false;
char *NMXmlParserExperiment::currentFileName = NULL;
int NMXmlParserExperiment::currentExperiment = -1;

Handle NMXmlParserExperiment::sport = Handle::UNDEFINED;
Handle NMXmlParserExperiment::soccer = Handle::UNDEFINED;
Handle NMXmlParserExperiment::link_sport_socker = Handle::UNDEFINED;
Handle NMXmlParserExperiment::hihger_order_link = Handle::UNDEFINED;
AtomSpace* NMXmlParserExperiment::atomSpace = NULL;

void NMXmlParserExperiment::initStaticVars()
{
    NMXmlParserExperiment::noCheck = false;
    NMXmlParserExperiment::currentFileName = NULL;
    NMXmlParserExperiment::currentExperiment = -1;

    NMXmlParserExperiment::sport = Handle::UNDEFINED;
    NMXmlParserExperiment::soccer = Handle::UNDEFINED;
    NMXmlParserExperiment::link_sport_socker = Handle::UNDEFINED;
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

    write(fd, expContents[exp%2], strlen(expContents[exp%2]));

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
    soccer = atomSpace->getHandle(WORD_NODE, "soccer");
    sport = atomSpace->getHandle(WORD_NODE, "sport");

    TS_ASSERT(TLB::isValidHandle(soccer));
    TS_ASSERT(TLB::isValidHandle(sport));
    if (TLB::isInvalidHandle(soccer) || TLB::isInvalidHandle(sport)) {
        return(false);
    }

    std::vector<Handle> handles;
    atomSpace->getHandleSet(back_inserter(handles), INHERITANCE_LINK, true);

    TS_ASSERT(handles.size() == 1);
    if (handles.size() != 1) {
        return(false);
    }
    Atom *atom = TLB::getAtom(handles[0]);
    link_sport_socker = handles[0];

    TS_ASSERT((atom->getOutgoingSet()[0]) == soccer);
    TS_ASSERT((atom->getOutgoingSet()[1]) == sport);
    if ((atom->getOutgoingSet()[0] != soccer) ||
            (atom->getOutgoingSet()[1] != sport)) {
        return(false);
    }

    return(true);
}

bool NMXmlParserExperiment::checkExp1()
{

    soccer = atomSpace->getHandle(WORD_NODE, "soccer");
    sport = atomSpace->getHandle(WORD_NODE, "sport");

    TS_ASSERT(TLB::isValidHandle(soccer));
    TS_ASSERT(TLB::isValidHandle(sport));
    if (TLB::isInvalidHandle(soccer) || TLB::isInvalidHandle(sport)) {
        return(false);
    }

    HandleEntry* entry = atomSpace->getAtomTable().getHandleSet(INHERITANCE_LINK, true);

    std::vector<Handle> handles;
    atomSpace->getHandleSet(back_inserter(handles), INHERITANCE_LINK, true);
    TS_ASSERT(handles.size() == 0);

    atomSpace->getHandleSet(back_inserter(handles), INHERITANCE_LINK, true);
    TS_ASSERT(handles.size() == 2);
    if (handles.size() != 2) {
        return(false);
    }

    TS_ASSERT(entry == NULL);

    link_sport_socker = Handle::UNDEFINED;
    Atom *atom = NULL;
    std::vector<Handle>::iterator it;
    for (it = handles.begin(); it != handles.end(); it++) {
        atom = TLB::getAtom((Handle) * it);
        if (atom->getIncomingSet()->getSize() == 1) {
            TS_ASSERT(TLB::isInvalidHandle(link_sport_socker));
            link_sport_socker = *it;
        }
    }
    handles.clear();

    TS_ASSERT(TLB::isValidHandle(link_sport_socker));
    TS_ASSERT((atom->getOutgoingSet()[0]) == soccer);
    TS_ASSERT((atom->getOutgoingSet()[1]) == sport);
    if ((atom->getOutgoingSet()[0] != soccer) ||
            (atom->getOutgoingSet()[1] != sport)) {
        return(false);
    }

    atomSpace->getHandleSet(back_inserter(handles), MEMBER_LINK, true);


    TS_ASSERT(handles.size() == 1);
    if (handles.size() != 1) {
        return(false);
    }
    atom = TLB::getAtom(handles[0]);
    hihger_order_link = handles[0];

    TS_ASSERT(atom->getOutgoingSet()[0] == link_sport_socker);
    TS_ASSERT(atom->getOutgoingSet()[1] == soccer);
    if ((atom->getOutgoingSet()[0] != link_sport_socker) ||
            (atom->getOutgoingSet()[1] != soccer)) {
        return(false);
    }


    /*
     entry = atomSpace->getAtomTable()->getHandleSet(INHERITANCE_LINK, true);

     TS_ASSERT(entry->getSize() == 2);
     if (entry->getSize() != 2){
      delete entry;
      return(false);
     }

     link_sport_socker = NULL;
     Atom *atom = NULL;

     HandleEntry *it = entry;
     while (it){
      atom = it->getAtom();
      if (atom->getIncomingSet()->getSize() == 1){
       TS_ASSERT(TLB::isInvalidHandle(link_sport_socker));
       link_sport_socker = it->handle;
      }
      it = it->next;
     }
     delete entry;

     TS_ASSERT(link_sport_socker != NULL);

     TS_ASSERT(TLB::getHandle(atom->getOutgoingSet()[0]) == soccer);
     TS_ASSERT(TLB::getHandle(atom->getOutgoingSet()[1]) == sport);
     if ((atom->getOutgoingSet()[0] != soccer) ||
      (atom->getOutgoingSet()[1] != sport)){
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

     TS_ASSERT(atom->getOutgoingSet()[0] == link_sport_socker);
     TS_ASSERT(atom->getOutgoingSet()[1] == soccer);
     if ((atom->getOutgoingSet()[0] != link_sport_socker) ||
      (atom->getOutgoingSet()[1] != soccer)){
      return(false);
     }
    */
    return(true);
}


const char *NMXmlParserExperiment::expContents[NNMXMLXMLEXPERIMENTS] = {
    "<?xml version=\"1.0\"?> \
    <list> \
    <tagdescription> \
    <tag name=\"WordNode\" value=\"WordNode\"/> \
    <tag name=\"InheritanceLink\" value=\"InheritanceLink\"/> \
    </tagdescription> \
    <WordNode name=\"soccer\" timestamp=\"3422826\"/> \
    <WordNode name=\"sport\" timestamp=\"3422826\"/> \
    <InheritanceLink hyp=\"hyp_1\" strength=\"1.0\" confidence=\"0.95\"> \
    <Element name=\"soccer\" class=\"WordNode\"/> \
    <Element name=\"sport\" class=\"WordNode\"/> \
    </InheritanceLink> \
    <InheritanceLink hyp=\"hyp_1\" strength=\"1.0\" confidence=\"0.95\"> \
    <Element name=\"soccer\" class=\"WordNode\"/> \
    <Element name=\"sport\" class=\"WordNode\"/> \
    </InheritanceLink> \
    </list>"
    /* UNCOMMENT TO USE HYPOTHETICAL ATOMTABLE
    ,
    "<?xml version=\"1.0\"?> \
    <list> \
    <tagdescription> \
    <tag name=\"WordNode\" value=\"WordNode\"/> \
    <tag name=\"InheritanceLink\" value=\"InheritanceLink\"/> \
    <tag name=\"MemberLink\" value=\"MemberLink\"/> \
    </tagdescription> \
    <WordNode name=\"soccer\" timestamp=\"3422827\"/> \
    <WordNode name=\"sport\" timestamp=\"3422827\"/> \
    <MemberLink strength=\"0.50\" confidence=\"0.80\"> \
      <InheritanceLink hyp=\"hyp_1\" strength=\"1.0\" confidence=\"0.95\"> \
        <Element name=\"soccer\" class=\"WordNode\"/> \
        <Element name=\"sport\" class=\"WordNode\"/> \
      </InheritanceLink> \
      <Element name=\"soccer\" class=\"WordNode\"/> \
    </MemberLink> \
    <InheritanceLink hyp=\"hyp_2\" strength=\"1.0\" confidence=\"0.95\"> \
      <Element name=\"soccer\" class=\"WordNode\"/> \
      <Element name=\"sport\" class=\"WordNode\"/> \
    </InheritanceLink> \
    </list>"
    */
};
