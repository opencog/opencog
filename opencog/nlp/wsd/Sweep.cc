
/*
 * Sweep.cc
 *
 * Implement a mark-n-sweep algo. The goal is to find the largest 
 * connected graph, and remove all other graphs.
 *
 * Copyright(c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#include <list>
#include <set>

#include <opencog/atomspace/types.h>
#include "ForeachWord.h"

namespace opencog {

class Sweep
{
	private:
		std::set<Handle> maxgraph;
		std::set<Handle> curgraph;
		std::list<Handle> maxedges;
		std::list<Handle> curedges;
		bool mark_word(Handle);
		bool start_mark_sense(Handle, Handle);
		bool mark_sense(Handle, Handle);
		void delete_edges(std::list<Handle> edges);
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
void Sweep::sweep_parse(Handle h)
{
	maxgraph.clear();
	maxedges.clear();
	foreach_word_instance(h, &Sweep::mark_word, this);
}

/**
 * walk over the entire connected graph for this word.
 */
bool Sweep::mark_word(Handle wordinst)
{
	curgraph.clear();
	curedges.clear();
	foreach_word_sense_of_inst(wordinst, 
	                        &Sweep::start_mark_sense, this);
	return false;
}

/**
 * Starting at this sense, walk all edges and find all connected
 * senses. Compare it to the largest known graph, and keep the
 * larger of the two, deleting the smaller one.
 *
 * This algorithm is not efficient, because we can't set a "mark"
 * bit on an sense atom. Instead, we have to add it to a bag of atoms
 * we've previously visited, and then search that bag for each new
 * sense that we encounter. Bummer.
 */
bool Sweep::start_mark_sense(Handle sense, Handle edge)
{
	// Have we already visited this node? If so, try the next sense.
	if (maxgraph.end() != maxgraph.find(sense))
	{
		maxedges.push_back(edge);
		return false;
	}

	// Hmm. We've never seen this sense before. Find the graph that
	// its connected to.
	curgraph.clear();
	curedges.clear();

	// Walk the entire connected component
	foreach_sense_edge(sense, &Sweep::mark_sense, this);

	// If we found a small graph, delete it.
	if (curgraph.size() <= maxgraph.size())
	{
		// delete all edges in currgraph
		
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
	curedges.push_back(edge);
	if (curgraph.end() != curgraph.find(sense)) return false;
	curgraph.insert(sense);
	foreach_sense_edge(sense, &Sweep::mark_sense, this);
	return false;
}
 
void Sweep::delete_edges(std::list<Handle> edges)
{
}
