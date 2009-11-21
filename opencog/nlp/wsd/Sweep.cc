
/*
 * Sweep.cc
 *
 * Implement a mark-n-sweep algo. The goal is to find the largest 
 * connected graph, and remove all other graphs.
 *
 * Copyright(c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

namespace opencog {

class Sweep
{
	private:
		int total_labels;
		bool mark_word(Handle h);
	public:
		void sweep_parse(Handle);
};

};

#include "ForeachWord.h"

using namespace opencog;

/**
 * Walk over all graphs associated with this parse, deleting
 * all graphs but the single largest one.
 */
void sweep_parse(Handle h)
{
	total_labels = 0;
	foreach_word_instance(h, &Sweep::mark_word, this);
}

/**
 * walk over the entire connected graph for this word.
 */
bool Sweep::mark_word(Handle h)
{
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
	foreach_dict_word_sense_pos(lemma_h, word_inst_pos,
	                        &MihalceaLabel::annotate_word_sense, this);
	return false;
}
