
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
		std::set<Handle> maxgraph;
		std::set<Handle> curgraph;
		bool mark_word(Handle h);
		bool start_mark_sense(Handle h);
		bool mark_sense(Handle h);
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
	maxgraph.clear();
	foreach_word_instance(h, &Sweep::mark_word, this);
}

/**
 * walk over the entire connected graph for this word.
 */
bool Sweep::mark_word(Handle wordinst)
{
	curgraph.clear();
	foreach_word_sense_of_inst(wordinst, 
	                        &Sweep::start_mark_sense, this);
	return false;
}

/**
 * Starting at this sense, walk and mark all connected senses.
 */
bool Sweep::start_mark_sense(Handle sense, Handle edge)
{
	// Have we already visited this node? if so, try the next sense.
	if(maxgraph.end() != maxgraph.find(sense)) return false;

	// Hmm. We've never seen this sense before. Find the graph that
	// its connected to.
	curgraph.clear();

	// Walk the entire connected component
	foreach_sense_edge(sense, &Sweep::mark_sense, this);

	if (curgraph.size() <= maxgraph.size())
	{
		// delete all senses and edges in currgraph
	}
	else
	{
		// delete all senses and edges in maxgraph
		// Save the new maxgraph
		maxgraph = curgraph;
	}
	
	return false;
}

/**
 * Recursive walk to all attached senses
 */
bool Sweep::mark_sense(Handle sense, Handle edge)
{
	if (curgraph.end() != curgraph.find(sense)) return false;
	curgraph.insert(sense);
	foreach_sense_edge(sense, &Sweep::mark_sense, this);
	return false;
}
