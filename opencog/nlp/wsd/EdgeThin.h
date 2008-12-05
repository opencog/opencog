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

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog {

class EdgeThin
{
	private:
		AtomSpace *atom_space;
		int prune_count;
		bool prune_sense(Handle, Handle);
		bool prune_word(Handle);

		int sense_count;
		bool count_sense(Handle, Handle);

		std::list<Handle> sense_list;
		bool make_sense_list(Handle, Handle);
		bool delete_sim(Handle);

		int keep;
		int edge_count;
		bool thin_word(Handle);
	public:
		void set_atom_space(AtomSpace *);
		void thin_parse(Handle, int);
		void prune_parse(Handle);
};

}

#endif /* _OPENCOG_WSD_THIN_EDGE_H */
