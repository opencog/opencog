/*
 * EdgeUtils.h
 *
 * Utilities for counting, manipulating edges between words in a sentence.
 *
 * Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifndef _OPENCOG_WSD_EDGE_UTILS_H
#define _OPENCOG_WSD_EDGE_UTILS_H

#include <set>

#include <opencog/atoms/base/Atom.h>

namespace opencog {

class EdgeUtils
{
	public:
		OrderedHandleSet words;
		bool look_at_relation(const std::string &, const Handle&, const Handle&);
		bool look_at_word(const Handle&);
};

}

#endif /* _OPENCOG_WSD_EDGE_UTILS_H */
