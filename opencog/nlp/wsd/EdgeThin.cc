/*
 * EdgeThin.h
 *
 * Thin out, remove edges between words, sentences.
 *
 * Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifndef _OPENCOG_WSD_THIN_EDGE_H
#define _OPENCOG_WSD_THIN_EDGE_H

namespace opencog {

class EdgeThin
{
	private:
		bool thin_word_pair(Handle, Handle, int);

	public:
		void thin_parse(Handle, int);
		void thin_parse_pair(Handle, Handle, int);
};

}

#endif /* _OPENCOG_WSD_THIN_EDGE_H */

// #include "EdgeThin.h"

#define DEBUG

using namespace opencog;
/**
 * Remove edges between senses of sets of words
 *
 * The routines in this file operate in a manner similr to those in
 * MihalceaEdge, but "in reverse" -- removing edges instead of adding
 * them.  Most of the routines will have a "count" paramter, indicating
 * the number of edges to keep. The "count" parameter works as follows: 
 *
 * If zero, it will remove all edges between all word pairs.
 *
 * If one, it will remove all edges, except the one connecting the
 * top-ranked sense of each word.
 *
 * If two, it will remove all edges that do not connect the top-two
 * ranked senses for each word.
 *
 * If N, it will remove all edges that do not connect the top-N ranked
 * senses for each word.
 */

/**
 * Remove edges between senses in the indicated parse.
 *
 * Similar to, but opposite of MihalceaEdge::annotate_parse()
 * rather than adding edges, it removes them. 
 */
void EdgeThin::thin_parse(Handle parse, int keep)
{
}

/**
 * Remove edges between senses of a pair of words.
 *
 * Similar to, but opposite to MihalceaEdge::annotate_word_pair()
 * rather than adding edges, it removes them. 
 */
bool EdgeThin::thin_word_pair(Handle first, Handle second, int keep)
{
#ifdef DEBUG
	Node *f = dynamic_cast<Node *>(TLB::getAtom(first));
	Node *s = dynamic_cast<Node *>(TLB::getAtom(second));
	const std::string &fn = f->getName();
	const std::string &sn = s->getName();
	printf ("; Thin out wordPair (%s, %s)\n", fn.c_str(), sn.c_str());
#endif

	return false;
}
