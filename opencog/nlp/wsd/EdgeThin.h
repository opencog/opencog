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

#include "EdgeUtils.h"

namespace opencog {

class EdgeThin : private EdgeUtils
{
	private:
		int word_pair_count;
		int edge_count;
		int keep;

		std::list<Handle> sense_list;
		bool make_sense_list(Handle, Handle);

		bool thin_word_pair(Handle, Handle, int);
	public:
		void thin_parse(Handle, int);
		void thin_parse_pair(Handle, Handle, int);
};

}

#endif /* _OPENCOG_WSD_THIN_EDGE_H */
