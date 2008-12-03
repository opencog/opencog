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

	public:
		bool thin_word_pair(Handle, Handle, int);


};

}

#endif /* _OPENCOG_WSD_THIN_EDGE_H */


/**
 * Remove edges between senses of a pair of words.
 *
 * The routine is similar to, but runs "in reverse" to 
 * MihalceaEdge::annotate_word_pair() -- rather than adding edges, it 
 * removes them. The "keep" parameter indicates how many edges to keep.
 *
 * If zero, it will remove all edges between the word pairs.
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
bool EdgeThin::thin_word_pair(Handle first, Handle second, int keep);
{
}
