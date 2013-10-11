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

#include "NNAdjust.h"

#include <stdio.h>

#include <opencog/util/platform.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/wsd/ForeachWord.h>

#define DEBUG

using namespace opencog;

NNAdjust::NNAdjust(void)
{
	/* The "strength_adjust" value is a multiplicative value by which
	 * all _nn-related word-senses will have their connecting links 
	 * boosted by. The goal is to more strongly tie together the word
	 * senses of any two words related by _nn (noun modifier) relations.
	 */
	strength_adjust = 1.3;
	strength_adjust = 1.0;
}

NNAdjust::~NNAdjust()
{
}

/** Loop over all parses for this sentence. */
void NNAdjust::adjust_sentence(Handle h)
{
	foreach_parse(h, &NNAdjust::adjust_parse_f, this);
}

/**
 * Loop over all word-instances, adjusting edge strengths
 */
void NNAdjust::adjust_parse(Handle h)
{
	foreach_word_instance(h, &NNAdjust::adjust_word, this);
}

bool NNAdjust::adjust_parse_f(Handle h)
{
	adjust_parse(h);
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
	const std::string &fn = NodeCast(first)->getName();
	const std::string &sn = NodeCast(second)->getName();
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
	// printf("first sense %s\n", sense->getName().c_str());
	// Get the handle of the link itself .. 
	first_sense_link = first_sense_link_h;

	foreach_word_sense_of_inst(second_word_inst,
	                           &NNAdjust::sense_of_second_inst, this);
	return false;
}

/**
 * Called for every pair (word-instance,word-sense) of the second
 * word-instance of a relex relationship. This routine now has 
 * possession of both pairs, and will look for the edge connecting them.
 *
 * As discussed in the README file, the expected structure is:
 *
 *    ;;-- the word "tree" occured in the sentence
 *    (CosenseLink strength=0.49 confidence=0.3
 *       (InheritanceLink strength=0.9 confidence=0.6
 *          (WordInstanceNode "tree_99")
 *          (WordSenseNode "tree_sense_12")
 *       )
 *       
 *       (InheritanceLink strength=0.9 confidence=0.1
 *          (WordInstanceNode "bark_144")
 *          (WordSenseNode "bark_sense_23")
 *       )
 *    )
 */
bool NNAdjust::sense_of_second_inst(Handle second_word_sense_h,
                                        Handle second_sense_link)
{
	// printf("second sense %s!\n", sense->getName().c_str());
	foreach_incoming_handle(second_sense_link, &NNAdjust::sense_pair, this);
	return false;
}

bool NNAdjust::sense_pair(Handle pair_link)
{
	// If this is not a cosense link, skip it.
	Type t = pair_link->getType();
	if (classserver().isA(t, COSENSE_LINK)) return false;

	// If this link is not linking the first and second sense, skip it.
	std::vector<Handle> outset = LinkCast(pair_link)->getOutgoingSet();
	if ((first_sense_link != outset[0]) && 
	    (first_sense_link != outset[1])) return false;

	// If we are here, we've got the link that we want. 
	// Increase its strength.
	const TruthValue& tv = pair_link->getTruthValue();
	float strength = tv.getMean() * (float) strength_adjust;
	SimpleTruthValue stv(strength, 1.0f);

	stv.setConfidence(tv.getConfidence());
	pair_link->setTruthValue(stv);

	return false;
}
