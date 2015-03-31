
/*
 * Sweep.h
 *
 * Implement a mark-n-sweep algo. The goal is to find the largest 
 * connected graph, and remove all other graphs.
 *
 * Copyright(c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifndef _OPENCOG_WSD_SWEEP_H
#define _OPENCOG_WSD_SWEEP_H

#include <set>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/types.h>

namespace opencog {

class Sweep
{
	private:
		AtomSpace *atom_space;
		std::set<Handle> maxgraph;
		std::set<Handle> curgraph;
		std::set<Handle> maxedges;
		std::set<Handle> curedges;
		bool mark_word(const Handle&);
		bool start_mark_sense(const Handle&, const Handle&);
		bool mark_sense(const Handle&, const Handle&);
		void delete_edges(std::set<Handle> &);
	public:
		void set_atom_space(AtomSpace *);
		void sweep_parse(const Handle&);
};

};

#endif /* _OPENCOG_WSD_SWEEP_H */

