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
#include "SimpleTruthValue.h"

using namespace opencog;

Mihalcea::Mihalcea(void)
{
	atom_space = NULL;
}

Mihalcea::~Mihalcea()
{
	atom_space = NULL;
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
 *      <WordNode name="bark">
 *    </ReferenceLink>
 *
 * Each word-instance is assumed to be linked to a part-of-speech via
 *
 *    <PartOfSpeechLink>
 *       <ConceptNode name="bark_169" />
 *       <DefinedLinguisticConceptNode name="#noun" />
 *    </PartOfSpeechLink>
 *
 * Each dictionary-word is assumed to be linked to word senses via
 *
 *    <WordSenseLink>
 *       <WordNode name="bark" />
 *       <ConceptNode name="bark_sense_23" />
 *    </WordSenseLink>
 *  
 * Each word-sense is assumed to be linked to a prt-of-speech via
 *
 *    <PartOfSpeechLink>
 *       <ConceptNode name="bark_sense_23" />
 *       <ConceptNode name="noun" />
 *    </PartOfSpeechLink>
 *
 */
bool Mihalcea::annotate_word(Handle h)
{
	word_instance = TLB::getAtom(h);

Node *n = dynamic_cast<Node *>(word_instance);
printf("found word-inst %s\n",  n->toString().c_str());

	// Find the part-of-speech for this word instance.
	FollowLink fl;
	Atom *inst_pos = fl.follow_binary_link(word_instance, PART_OF_SPEECH_LINK);
	n = dynamic_cast<Node *>(inst_pos);
	word_inst_pos = n->getName();

	word_inst_pos.erase(0,1);  // remove leading hash sign

	// Reject some unwanted parts-of-speech.
	if (0 == word_inst_pos.compare("WORD")) return false;
	if (0 == word_inst_pos.compare("det")) return false;
	if (0 == word_inst_pos.compare("particle")) return false;
	if (0 == word_inst_pos.compare("prep")) return false;
	if (0 == word_inst_pos.compare("punctuation")) return false;


printf("found inst-pos %s\n",  word_inst_pos.c_str());

	Atom *dict_word = fl.follow_binary_link(word_instance, REFERENCE_LINK);
	Handle dict_word_h = TLB::getHandle(dict_word);
n = dynamic_cast<Node *>(dict_word);
printf("found word-dict %s\n",  n->toString().c_str());
 
	ForeachChaseLink<Mihalcea> chase;
	chase.follow_binary_link(dict_word_h, WORD_SENSE_LINK,
	                            &Mihalcea::annotate_word_sense, this);
	return false;
}

/**
 * Create a link coupling a specific word-instance to a possible 
 * word-sense.  The link to be created will resemble the following:
 *
 *   <InheritanceLink strength=0.9 confidence=0.1>
 *      <ConceptNode name="bark_144" />
 *      <WordSenseNode name="bark_sense_23" />
 *   </InheritanceLink>
 */
bool Mihalcea::annotate_word_sense(Handle h)
{
	Atom *word_sense = TLB::getAtom(h);

	// Find the part-of-speech for this word-sense.
	FollowLink fl;
	Atom *a = fl.follow_binary_link(word_sense, PART_OF_SPEECH_LINK);
	Node *n = dynamic_cast<Node *>(a);
	std::string sense_pos = n->getName();

	// If there's no POS match, skip this sense.
	if (word_inst_pos.compare(sense_pos)) return false;

n = dynamic_cast<Node *>(word_sense);
printf("found word-sense %s\n",  n->toString().c_str());
printf("keeping word-sense pos %s\n",  sense_pos.c_str());

	// Create a link connecting this word-instance to this word-sense.
	std::vector<Handle> out;
	out.push_back(TLB::getHandle(word_instance));
	out.push_back(TLB::getHandle(word_sense));

	// Give it a mediocre truth value, very low confidence.
	SimpleTruthValue stv(0.5, 1.0);
	stv.setConfidence(0.01);
	atom_space->addLink(INHERITANCE_LINK, out, stv);

	return false;
}

void Mihalcea::process_sentence(Handle h)
{
	annotate_sentence(h);
}

