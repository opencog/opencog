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
#include <opencog/atomspace/Foreach.h>
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

void MihalceaLabel::set_atom_space(AtomSpace *as)
{
	atom_space = as;
	no_sense = atom_space->addNode(WORD_SENSE_NODE, "#UNKNOWN SENSE");
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
 * Annotate the given word with every possible word sense, given its 
 * part-of-speech. The argument handle is assumed to point at a specific 
 * word-instance in some parse.
 */
bool MihalceaLabel::annotate_word(Handle h)
{
    
	word_instance = h;

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
	printf("; MihalceaLabel::annotate_word(%lx)\n", (unsigned long) h);
	printf("; found word-inst %s\n", atom_space.atomAsString(word_instance).c_str());
	printf(";\thas inst-POS %s\n", word_inst_pos.c_str());
	printf(";\thas word-dict %s\n", atom_space.atomAsString(lemma_h).c_str());
#endif

	// Pull in word senses from the persistent store, if needed.
#ifdef USE_DYNAMIC_SENSE_LOADING
	fetch_senses(lemma_h);
#endif

	// Loop over all word senses with this part-of-speech.
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
bool MihalceaLabel::annotate_word_sense(Handle word_sense)
{
#ifdef DEBUG
	printf(";\thas word-sense %s\n", atom_space->atomAsString(word_sense).c_str());
	total_labels++;
#endif

	// Create a link connecting this word-instance to this word-sense.
	std::vector<Handle> out;
	out.push_back(word_instance);
	out.push_back(word_sense);

	// Give it a true truth value; but no confidence.
	TruthValuePtr ctv(CountTruthValue::createTV(1.0f, 0.0f, 1.0f));
	atom_space->addLink(INHERITANCE_LINK, out, ctv);

	return false;
}

// ==================================================================

bool MihalceaLabel::have_sense(Handle h)
{
	if (atom_space->getType(h) == WORD_SENSE_LINK) return true;
	return false;
}

bool MihalceaLabel::pull_pos(Handle sense_h)
{
	atom_space->fetchIncomingSet(sense_h, false);
	return false;
}

/**
 * fetch_senses -- fetch all word-senses for lemma from persistent storage.
 *
 * Given a lemma, check to see if there are any WordSenseLinks on it.
 * If there some, then assume that all senses have been loaded, and 
 * do nothing.  But it theere aren't any senses, then pull them from
 * the persistent store. Be sure to pull the POS tags as well. These
 * are linked via a PartOfSpeechLink to the word-sense.
 *
 * Some things, like proper names, will not have any senses. Avoid 
 * repeated lookups of these by tagging them with a single, unknown 
 * sense.
 */
void MihalceaLabel::fetch_senses(Handle lemma_h)
{
	bool rc = foreach_incoming_handle(lemma_h, &MihalceaLabel::have_sense, this);
   if (rc) return;

	// If we are here, we need to pull senses from the database.
	atom_space->fetchIncomingSet(lemma_h, false);

	// Also pull the POS tags.
	foreach_binary_link(lemma_h, WORD_SENSE_LINK, &MihalceaLabel::pull_pos, this);

	// If we now have senses, we are done.
	rc = foreach_incoming_handle(lemma_h, &MihalceaLabel::have_sense, this);
	if (rc) return;

	// Add a bogus sense.
	atom_space->addLink(WORD_SENSE_LINK, lemma_h, no_sense);
}

/* ============================== END OF FILE ====================== */
