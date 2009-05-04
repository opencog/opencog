/*
 * MihalceaLabel.cc
 *
 * Implements the word-instance labelling portion of the Rada Mihalcea
 * word-sense disambiguation algorithm. For each word-instance in a
 * sentence, one of several possible word-senses are attached. It is
 * assumed that the running opencog server already has a number of 
 * possible word senses loaded for a given word; this code merely
 * looks those up, and attaches them to a word instance. 
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "MihalceaLabel.h"

#include <stdio.h>

#include <opencog/util/platform.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/nlp/wsd/ForeachWord.h>

// #define DEBUG

using namespace opencog;

MihalceaLabel::MihalceaLabel(void)
{
	atom_space = NULL;
}

MihalceaLabel::~MihalceaLabel()
{
	atom_space = NULL;
}

/**
 * Anotate every word in every parse of the sentence with every possible
 * word sense for that word. The argument handle is presumed to identify
 * a SentenceNode.
 */
void MihalceaLabel::annotate_sentence(Handle h)
{
	foreach_parse(h, &MihalceaLabel::annotate_parse_f, this);
}

/**
 * Anotate every word in the given parse with every possible word sense
 * for that word. The argument handle is presumed to identify a specific
 * parse.
 */
void MihalceaLabel::annotate_parse(Handle h)
{
	total_words = 0;
	total_labels = 0;
	foreach_word_instance(h, &MihalceaLabel::annotate_word, this);
#ifdef DEBUG
	printf("; MihalceaLabel: Applied %d word-sense labels to %d words\n", total_labels, total_words);
	printf("; ---------------------------------\n");
#endif
}

bool MihalceaLabel::annotate_parse_f(Handle h)
{
	annotate_parse(h);
	return false;
}

/**
 * Anotate the given word with every possible word sense, given its 
 * part-of-speech. The argument handle is assumed to point at a specific 
 * word-instance in some parse.
 */
bool MihalceaLabel::annotate_word(Handle h)
{
	word_instance = TLB::getAtom(h);

	// Find the part-of-speech for this word instance.
	std::string word_inst_pos = get_part_of_speech(h);

	// Reject some unwanted parts-of-speech.
	if (0 == word_inst_pos.compare("")) return false;
	if (0 == word_inst_pos.compare("WORD")) return false;
	if (0 == word_inst_pos.compare("det")) return false;
	if (0 == word_inst_pos.compare("particle")) return false;
	if (0 == word_inst_pos.compare("prep")) return false;
	if (0 == word_inst_pos.compare("punctuation")) return false;

	Handle lemma_h = get_lemma_of_word_instance(h);

#ifdef DEBUG
	total_words ++;
	Node *n = dynamic_cast<Node *>(word_instance);
	printf("; MihalceaLabel::annotate_word(%lx)\n", (unsigned long) h);
	printf("; found word-inst %s\n",  n->toString().c_str());
	printf(";\thas inst-POS %s\n",  word_inst_pos.c_str());
	n = dynamic_cast<Node *>(TLB::getAtom(lemma_h));
	printf(";\thas word-dict %s\n",  n->toString().c_str());
#endif

	// loop over all word senses with this part-of-speech.
	foreach_dict_word_sense_pos(lemma_h, word_inst_pos,
	                        &MihalceaLabel::annotate_word_sense, this);
	return false;
}

/**
 * Create a link coupling a specific word-instance to a possible 
 * word-sense.  The link to be created will resemble the following:
 *
 *   InheritanceLink strength=0.9 confidence=0.1
 *      WordInstanceNode "bark_144"
 *      WordSenseNode "bark_sense_23"
 */
bool MihalceaLabel::annotate_word_sense(Handle h)
{
	Atom *word_sense = TLB::getAtom(h);

#ifdef DEBUG
	Node *n = dynamic_cast<Node *>(word_sense);
	printf(";\thas word-sense %s\n",  n->toString().c_str());
	total_labels++;
#endif

	// Create a link connecting this word-instance to this word-sense.
	std::vector<Handle> out;
	out.push_back(TLB::getHandle(word_instance));
	out.push_back(TLB::getHandle(word_sense));

	// Give it a true truth value; but no confidence.
	CountTruthValue ctv(1.0f, 0.0f, 1.0f);
	atom_space->addLink(INHERITANCE_LINK, out, ctv);

	return false;
}

/* ============================== END OF FILE ====================== */
