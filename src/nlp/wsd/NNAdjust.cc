/*
 * NNAdjust.cc
 *
 * Bumps up the strength of the edges connecting two nouns in a  
 * noun-modifier (nn) relationship. Basically, if there is a 
 * noun-modifier phrase, then make a stronger tie to make sure 
 * that the two word senses corellate with each other.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "ForeachWord.h"
#include "NNAdjust.h"
#include "SimpleTruthValue.h"

#define DEBUG

using namespace opencog;

NNAdjust::NNAdjust(void)
{
}

NNAdjust::~NNAdjust()
{
}

/** Loop over all parses for this sentence. */
void NNAdjust::adjust_sentence(Handle h)
{
	foreach_parse(h, &NNAdjust::adjust_parse, this);
}

/**
 * For each parse, loop over all word-instances
 */
bool NNAdjust::adjust_parse(Handle h)
{
	foreach_word_instance(h, &NNAdjust::adjust_word, this);
	return false;
}

/**
 * For each word-instance loop over all syntactic relationships.
 * (i.e. _subj, _obj, _nn, _amod, and so on). Discard all but the _nn.
 */
bool NNAdjust::adjust_word(Handle h)
{
	foreach_relex_relation(h, &NNAdjust::adjust_relation, this);
	return false;
}

/**
 * This routine is called for every relation between word-instances in
 * a parse. 
 */
bool NNAdjust::adjust_relation(const std::string &relname, Handle first, Handle second)
{
	if (relname.compare("_nn")) return false;
#ifdef DEBUG
	Node *f = dynamic_cast<Node *>(TLB::getAtom(first));
	Node *s = dynamic_cast<Node *>(TLB::getAtom(second));
	const std::string &fn = f->getName();
	const std::string &sn = s->getName();
	printf("_nn(%s, %s)\n", fn.c_str(), sn.c_str());
#endif

	second_word_inst = second;
	foreach_word_sense_of_inst(first, &NNAdjust::sense_of_first_inst, this);
	
	return false;
}

/**
 * Called for every pair (word-instance,word-sense) of the first
 * word-instance of a relex relationship. This, in turn iterates
 * over the second word-instance of the relex relationship.
 */
bool NNAdjust::sense_of_first_inst(Handle first_word_sense_h,
                                       Handle first_sense_link_h)
{
	first_word_sense = first_word_sense_h;

	// printf("first sense %s\n", sense->getName().c_str());
	// Get the handle of the link itself .. 
	first_sense_link = first_sense_link_h;

	foreach_word_sense_of_inst(second_word_inst,
	                           &NNAdjust::sense_of_second_inst, this);
	return false;
}

/**
 * Called for every pair (word-instance,word-sense) of the second
 * word-instance of a relex relationship. This routine is the last,
 * most deeply nested loop of all of this set of nested loops.  This
 * routine now has possession of both pairs, and can now adjust the
 * strength of the link between them.
 *
 * As discussed in the README file, the expected structure is:
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
bool NNAdjust::sense_of_second_inst(Handle second_word_sense_h,
                                        Handle second_sense_link)
{
	// printf("second sense %s!\n", sense->getName().c_str());

	return false;
}
