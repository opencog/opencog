/*
 * MihalceaEdge.cc
 *
 * Implements the edge creation portion of the Rada Mihalcea
 * word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "ForeachWord.h"
#include "MihalceaEdge.h"
#include "SenseCache.h"
#include "SenseSimilarity.h"
#include "SimpleTruthValue.h"

#define DEBUG

using namespace opencog;

MihalceaEdge::MihalceaEdge(void)
{
	atom_space = NULL;
}

MihalceaEdge::~MihalceaEdge()
{
	atom_space = NULL;
}

void MihalceaEdge::set_atom_space(AtomSpace *as)
{
	atom_space = as;
}

/** Loop over all parses for this sentence. */
void MihalceaEdge::annotate_sentence(Handle h)
{
	foreach_parse(h, &MihalceaEdge::annotate_parse_f, this);
}

/**
 * For each parse, loop over all word-instance syntactic relationships.
 * (i.e. _subj, _obj, _nn, _amod, and so on). For each relationship,
 * create an edge between all corresponding (word-instance, word-sense)
 * pairs.
 */
void MihalceaEdge::annotate_parse(Handle h)
{
	foreach_word_instance(h, &MihalceaEdge::look_at_word, this);

	// At this point, "words" contains all of the relex-participating
	// words in the parse. Loop over word-pairs, and annotate them.
	std::set<Handle>::const_iterator f;
	for (f = words.begin(); f != words.end(); f++)
	{
		std::set<Handle>::const_iterator s = f;
		for (s++; s != words.end(); s++)
		{
			annotate_word_pair(*f, *s);
		}
	}
}

bool MihalceaEdge::annotate_parse_f(Handle h)
{
	annotate_parse(h);
	return false;
}

/**
 * For each pair of parses, create word-sense edge-links between
 * the two parses.
 */
void MihalceaEdge::annotate_parse_pair(Handle earlier, Handle later)
{
}

/**
 * For each word-instance loop over all syntactic relationships.
 * (i.e. _subj, _obj, _nn, _amod, and so on). Generate a list of all
 * words that participate in relationships. Then create links between
 * word-senses of every pair of words.
 */
bool MihalceaEdge::look_at_word(Handle h)
{
	words.clear();
	foreach_relex_relation(h, &MihalceaEdge::look_at_relation, this);

	return false;
}

/**
 * This routine is called for every relation between word-instances in
 * a parse. It simply creates a list of all of the words in a sentence
 * that participate in RelEx relations.
 */
bool MihalceaEdge::look_at_relation(const std::string &relname, Handle first, Handle second)
{
	words.insert(first);
	words.insert(second);
	return false;
}

/**
 * Create edges between all senses of a pair of words.
 *
 * This routine implements a doubley-nested foreach loop, iterating
 * over all senses of each word, and creating an edge between them.
 *
 * All of the current word-sense similarity algorithms report zero
 * similarity when the two words are different parts of speech. 
 * Therefore, in order to improve performance, this routine does not 
 * create any edges between words of differing parts-of-speech.
 */
bool MihalceaEdge::annotate_word_pair(Handle first, Handle second)
{
#ifdef DEBUG
	Node *f = dynamic_cast<Node *>(TLB::getAtom(first));
	Node *s = dynamic_cast<Node *>(TLB::getAtom(second));
	const std::string &fn = f->getName();
	const std::string &sn = s->getName();
	printf("(%s, %s)\n", fn.c_str(), sn.c_str());
#endif

	// Don't bother linking words with different parts-of-speech;
	// the similarity measures don't support these.
	std::string first_pos = get_part_of_speech(first);
	std::string second_pos = get_part_of_speech(second);
	if (0 != first_pos.compare(second_pos))
	{
		return false;
	}

	second_word_inst = second;
	foreach_word_sense_of_inst(first, &MihalceaEdge::sense_of_first_inst, this);
	
	return false;
}

/**
 * Called for every pair (word-instance,word-sense) of the first
 * word-instance of a relex relationship. This, in turn iterates
 * over the second word-instance of the relex relationship.
 */
bool MihalceaEdge::sense_of_first_inst(Handle first_word_sense_h,
                                       Handle first_sense_link_h)
{
	first_word_sense = first_word_sense_h;

	// printf("first sense %s\n", sense->getName().c_str());
	// Get the handle of the link itself .. 
	first_sense_link = first_sense_link_h;

	foreach_word_sense_of_inst(second_word_inst,
	                           &MihalceaEdge::sense_of_second_inst, this);
	return false;
}

/**
 * Called for every pair (word-instance,word-sense) of the second
 * word-instance of a relex relationship. This routine is the last,
 * most deeply nested loop of all of this set of nested loops.  This
 * routine now has possession of both pairs, and can now create a 
 * Mihalcea-graph edge between these pairs.
 *
 * As discussed in the README file, the resulting structure is:
 *
 *    <!-- the word "tree" occured in the sentence -->
 *    <CosenseLink strength=0.49 confidence=0.3>
 *       <InheritanceLink strength=0.9 confidence=0.6>
 *          <ConceptNode name="tree_99" />
 *          <WordSenseNode name="tree_sense_12" />
 *       </InheritanceLink>
 *       
 *       <InheritanceLink strength=0.9 confidence=0.1>
 *          <ConceptNode name="bark_144" />
 *          <WordSenseNode name="bark_sense_23" />
 *       </InheritanceLink>
 *    </CosenseLink>
 */
bool MihalceaEdge::sense_of_second_inst(Handle second_word_sense_h,
                                        Handle second_sense_link)
{
	// printf("second sense %s!\n", sense->getName().c_str());
	
	// Get the similarity between the two word senses out of the 
	// cache (if it exists).
	SenseCache sc;
	SimpleTruthValue stv(0.5,0.5);
	stv = sc.similarity(first_word_sense, second_word_sense_h);
	if (stv == TruthValue::DEFAULT_TV())
	{
		// Use a word-sense similarity/relationship measure to assign an 
		// initial truth value to the edge.
		SenseSimilarity ss;
		stv = ss.lch_similarity(first_word_sense, second_word_sense_h);
		Link * l = sc.set_similarity(first_word_sense, second_word_sense_h, stv);
		atom_space->addRealAtom(*l);
		delete l;
	}

	// Skip making edges between utterly unrelated nodes. 
	if (stv.getMean() < 0.01) return false;

	// Create a link connecting the first pair to the second pair.
	std::vector<Handle> out;
	out.push_back(first_sense_link);
	out.push_back(second_sense_link);

	atom_space->addLink(COSENSE_LINK, out, stv);

	return false;
}
