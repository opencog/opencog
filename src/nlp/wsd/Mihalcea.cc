/*
 * Mihalcea.cc
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "FollowLink.h"
#include "ForeachChaseLink.h"
#include "Mihalcea.h"
#include "Node.h"

using namespace opencog;

Mihalcea::Mihalcea(void)
{
}

Mihalcea::~Mihalcea()
{
}

/**
 * Anotate every word in every parse of the sentence with every possible
 * word sense for that word. The argument handle is presumed to identify
 * a SentenceNode, which is linked to parses via a ParseLink:
 * 
 *    <ParseLink>
 *      <ConceptNode name="parse_2" strength=0.8 confidence=0.5/>
 *      <SentenceNode name="sentence_22" />
 *    </ParseLink>
 */
void Mihalcea::annotate_sentence(Handle h)
{
	ForeachChaseLink<Mihalcea> chase;
	chase.backtrack_binary_link(h, PARSE_LINK, 
	                            &Mihalcea::annotate_parse, this);
}

/**
 * Anotate every word in the given parse with every possible word sense
 * for that word. The argument handle is presumed to identify a specific
 * parse. Each word-instance in the parse is linked to it via a 
 * ParseInstanceLink:
 *
 *    <ParseInstanceLink>
 *       <ConceptNode name="bark_169" />
 *       <ConceptNode name="parse_3" />
 *    </ParseInstanceLink>
 */
bool Mihalcea::annotate_parse(Handle h)
{
	printf("found parse %x\n", (unsigned long) h);
	ForeachChaseLink<Mihalcea> chase;
	chase.backtrack_binary_link(h, PARSE_INSTANCE_LINK,
	                            &Mihalcea::annotate_word, this);
	return false;
}

/**
 * Anotate the given word with every possible word sense, given its 
 * part-of-speech. The argument handle is assumed to point at a specific 
 * word-instance in some parse.
 *
 * Each word-instance is assumed to be link to a single WordNode via 
 * a ReferenceLink:
 *
 *    <ReferenceLink>
 *      <ConceptNode name="bark_169" />
 *      <WordNode name="#bark">
 *    </ReferenceLink>
 *
 */
bool Mihalcea::annotate_word(Handle h)
{
	Atom *word_instance = TLB::getAtom(h);
	Node *n = dynamic_cast<Node *>(word_instance);
	printf("found word-inst %s\n",  n->toString().c_str());

	FollowLink fl;
	Atom *dict_word = fl.follow_binary_link(word_instance, REFERENCE_LINK);
	n = dynamic_cast<Node *>(dict_word);
	printf("found word-dict %s\n",  n->toString().c_str());
 
	return false;
}

void Mihalcea::process_sentence(Handle h)
{
	annotate_sentence(h);
}

