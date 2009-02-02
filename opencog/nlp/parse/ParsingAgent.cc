/*
 * opencog/nlp/parse/ParsingAgent.cc
 *
 * Copyright (C) 2009 by Singularity Institute for Artificial Intelligence
 * Written by Trent Waddington <trent.waddington@gmail.com>
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

#ifdef HAVE_LINK_GRAMMAR

#define NOISY 1

#include "ParsingAgent.h"

#include <algorithm>
#include <sstream>

#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/util/Config.h>
#include <opencog/atomspace/Link.h>

//#define LINK_GRAMMAR_DATA_DIR (char*)"/usr/share/link-grammar/en/"

using namespace opencog;

ParsingAgent::ParsingAgent()
{
#if NOISY
    printf("ParsingAgent::ParsingAgent()\n");
#endif

    // Provide a logger, but disable it initially
    log = NULL;
    setLogger(new opencog::Logger("ParsingAgent.log", Logger::WARN, true));
    log->disable();

    opts  = parse_options_create();
    dict  = dictionary_create((char*)LINK_GRAMMAR_DATA_DIR "4.0.dict", 
                              (char*)LINK_GRAMMAR_DATA_DIR "4.0.knowledge", 
                              NULL, 
                              (char*)LINK_GRAMMAR_DATA_DIR "4.0.affix");
}

ParsingAgent::~ParsingAgent()
{
#if NOISY
    printf("ParsingAgent::~ParsingAgent()\n");
#endif

    if (log) delete log;
    dictionary_delete(dict);
    parse_options_delete(opts);
}

Logger* ParsingAgent::getLogger()
{
    return log;
}

void ParsingAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

void ParsingAgent::run(CogServer *c)
{
    log->fine("=========== ParsingAgent::run =======");
    a = c->getAtomSpace();

    HandleSeq sNodes;
    a->getHandleSet(back_inserter(sNodes), SENTENCE_NODE, false);
    for (HandleSeq::iterator it = sNodes.begin(); it != sNodes.end(); it++)
        parse(*it);
}

struct compare_link {
    Type t;
    bool subclass;
    Arity position;
    Type tt;
    bool targetSubclass;

    compare_link(Type t, bool s = false, 
                 Arity p = (Arity)-1, Type tt = ATOM, bool ts = true) 
        : t(t), subclass(s), position(p), tt(tt), targetSubclass(ts) { }

    bool operator()(const Handle& h) {
        Atom *a = TLB::getAtom(h);
        Link *l = (Link*)a;
        Type at = a->getType();

        if (!(at == t || subclass && ClassServer::isAssignableFrom(t, at)))
            return false;

        if (position == (Arity)-1)
            return true;

        if (l->getArity() <= position)
            return false;

        Atom *ta = TLB::getAtom(l->getOutgoingSet()[position]);
        Type tat = ta->getType();
        
        if (tat == tt)
            return true;

        if (!targetSubclass)
            return false;

        return ClassServer::isAssignableFrom(tt, tat);
    }
};

void ParsingAgent::parse(Handle hSentenceNode)
{
    HandleSeq out = a->getIncoming(hSentenceNode);
    HandleSeq parses;
    a->filter(out.begin(), out.end(), back_inserter(parses), compare_link(PARSE_LINK));
    if (parses.size() != 0) 
        return;   // already parsed

    Node *n = (Node*)TLB::getAtom(hSentenceNode);
    char *str = strdup(n->getName().c_str());
#if NOISY
    printf("parsing: %s\n", str);
#endif

    Sentence sent = sentence_create(str, dict);
    free(str);

    if (sent == NULL) {
#if NOISY
        printf("sentence_create failed\n");
#endif
        return;
    }

    // this might be useful some day.. suppose parsing fails and you want
    // the engine to figure out why.
    //HandleSeq wnis;
    //addTokenization(hSentenceNode, sent, wnis);

    int num_linkages = sentence_parse(sent, opts);
    if (num_linkages == 0) {
        sentence_delete(sent);
#if NOISY
        printf("sentence_parse failed\n");
#endif
        return;
    }

    for (int i = 0; i < num_linkages; i++) {
        Linkage linkage = linkage_create(i, sent, opts);
        addParsing(hSentenceNode, sent, linkage);
        linkage_delete(linkage);
    }

    sentence_delete(sent);
}

void ParsingAgent::addTokenization(Handle hNode, Sentence sent, HandleSeq &wnis)
{
    for (int w = 0; w < sentence_length(sent); w++) {
        std::string str = sentence_get_word(sent, w);

        Handle wn;
        Handle wni;
        if (str == "LEFT-WALL" || str == "RIGHT-WALL")
        {
            wn = a->addNode(CONCEPT_NODE, str);
            wni = a->addNode(NODE);
        }
        else
        {
            wn = a->addNode(WORD_NODE, str);
            wni = a->addNode(WORD_INSTANCE_NODE);
        }

        a->addLink(EXTENSIONAL_INHERITANCE_LINK, wni, wn);
        wnis.push_back(wni);
    }

    Handle l = a->addLink(LIST_LINK, wnis);
    a->addLink(HOLONYM_LINK, l, hNode);
}

void ParsingAgent::addParsing(Handle hSentenceNode, Sentence sent, Linkage linkage)
{
#if NOISY
    char *diagram = linkage_print_diagram(linkage);
    printf("%s\n", diagram);
    free(diagram);
#endif

    Handle hParseNode = a->addNode(PARSE_NODE);
    a->addLink(PARSE_LINK, hSentenceNode, hParseNode);

    HandleSeq wnis;
    addTokenization(hParseNode, sent, wnis);

    for (int i = 0; i < linkage_get_num_links(linkage); i++)
    {
        int left = linkage_get_link_lword(linkage, i);
        int right = linkage_get_link_rword(linkage, i);
        Handle l = a->addLink(LIST_LINK, wnis[left], wnis[right]);
        Handle hRel = a->addNode(LINK_GRAMMAR_RELATIONSHIP_NODE,
                                   linkage_get_link_label(linkage, i));
        Handle hRelInst = a->addNode(NODE);
        a->addLink(EXTENSIONAL_INHERITANCE_LINK, hRelInst, hRel);
        a->addLink(HOLONYM_LINK, l, hRelInst);
        a->addLink(HOLONYM_LINK, hRelInst, hParseNode);
    }
}

#endif // HAVE_LINK_GRAMMAR

