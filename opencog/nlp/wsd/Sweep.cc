
/*
 * Sweep.cc
 *
 * Implement a mark-n-sweep algo. The goal is to find the largest 
 * connected graph, and remove all other graphs.
 *
 * Copyright(c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#include "ForeachWord.h"
#include "Sweep.h"

using namespace opencog;

void Sweep::set_atom_space(AtomSpace *as)
{
   atom_space = as;
}

/**
 * Walk over all word-sense graphs associated with this parse, 
 * deleting all graphs but the single largest one. Here, a "graph"
 * is the mihalcea graph, with word-senses as vertexes, and word-sense
 * pairs forming edges.
 */
void Sweep::sweep_parse(Handle h)
{
	maxgraph.clear();
	maxedges.clear();
	foreach_word_instance(h, &Sweep::mark_word, this);
}

/**
 * Walk over the entire connected graph for this word.
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
bool Sweep::start_mark_sense(Handle sense, Handle slink)
{
	// Have we already visited this node? If so, try the next sense.
	if (maxgraph.end() != maxgraph.find(sense)) return false;

	// Hmm. We've never seen this sense before. Find the graph that
	// its connected to.
	curgraph.clear();
	curedges.clear();

	// Walk the entire connected component
	curgraph.insert(sense);
	foreach_sense_edge(sense, &Sweep::mark_sense, this);

	// At this point, "curgraph" holds an entire connected component.
	// If we found a small graph, delete it.
	if (curgraph.size() <= maxgraph.size())
	{
		// delete all edges in curgraph
		delete_edges(curedges);
	}
	else
	{
		// Delete all edges in maxgraph (we'll delete sensdes later)
		delete_edges(maxedges);

		// Save the new maxgraph
		maxgraph = curgraph;
		maxedges = curedges;
	}
	curgraph.clear();
	curedges.clear();
	
	return false;
}

/**
 * Recursive walk to all attached senses
 */
bool Sweep::mark_sense(Handle sense, Handle edge)
{
	curedges.insert(edge);
	if (curgraph.end() != curgraph.find(sense)) return false;
	curgraph.insert(sense);
	foreach_sense_edge(sense, &Sweep::mark_sense, this);
	return false;
}
 
void Sweep::delete_edges(std::set<Handle> &edges)
{
	// Remove all of the senses
	std::set<Handle>::iterator it;
	for (it=edges.begin(); it != edges.end(); ++it)
	{
		Handle edge_h = *it;
		atom_space->removeAtom(edge_h, false);
	}
}
