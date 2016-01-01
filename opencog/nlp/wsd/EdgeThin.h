/*
 * EdgeThin.h
 *
 * Thin out, remove edges between words, sentences.
 *
 * Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifndef _OPENCOG_WSD_THIN_EDGE_H
#define _OPENCOG_WSD_THIN_EDGE_H

#include <list>

#include <opencog/atoms/base/Atom.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog {

class EdgeThin
{
	private:
		AtomSpace *atom_space;
		int prune_count;
		bool prune_sense(const Handle&, const Handle&);
		bool prune_word(const Handle&);

		int sense_count;
		bool count_sense(const Handle&, const Handle&);

		std::list<Handle> sense_list;
		bool make_sense_list(const Handle&, const Handle&);
		bool delete_sim(const Handle&);

		int keep;
		int edge_count;
		bool thin_word(const Handle&);
	public:
		void set_atom_space(AtomSpace *);
		void thin_parse(const Handle&, int);
		void prune_parse(const Handle&);
};

}

#endif /* _OPENCOG_WSD_THIN_EDGE_H */
