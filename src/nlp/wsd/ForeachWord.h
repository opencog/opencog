/*
 * ForeachWord.h
 *
 * Implements a collection of iterators for running over the multiple
 * parses of a sentence, the multiple word-instances of a parse, and so
 * on. The goal here is that these iterators hide the structural detail
 * of the opencog representation of sentences, pares, and so on. Thus,
 * if (when?) the opencog representation changes, then only this file 
 * needs to be adjusted, instead of the broad sweep of algorithms.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_FOREACH_WORD_H
#define OPENCOG_FOREACH_WORD_H

#include <ForeachChaseLink.h>

namespace opencog {

/**
 * Call the callback for each parse in a sentence.  The argument handle
 * is presumed to identify  a SentenceNode, which is linked to parses 
 * via a ParseLink:
 * 
 *    <ParseLink>
 *      <ConceptNode name="parse_2" strength=0.8 confidence=0.5/>
 *      <SentenceNode name="sentence_22" />
 *    </ParseLink>
 */
template<class T>
inline void foreach_parse(Handle h, bool (T::*cb)(Handle), T *data)
{
	ForeachChaseLink<T> chase;
	chase.backtrack_binary_link(h, PARSE_LINK, cb, data);
}

/**
 * Call the callback for every word-instance in a parse. The argument
 * handle is presumed to identify a specific parse. Each word-instance
 * in the parse is linked to it via a ParseInstanceLink:
 *
 *    <ParseInstanceLink>
 *       <ConceptNode name="bark_169" />
 *       <ConceptNode name="parse_3" />
 *    </ParseInstanceLink>
 */
template <class T>
inline void foreach_word_instance(Handle h, bool (T::*cb)(Handle), T *data)
{
	ForeachChaseLink<T> chase;
	chase.backtrack_binary_link(h, PARSE_INSTANCE_LINK, cb, data);
}

/**
 * Given a dictionary word, call the callback for each word sense
 * associated with that dictionary word (for all parts-of-speech).
 * The argument is presumed to point at a specific dictionary word.
 *
 * Each dictionary-word is assumed to be linked to word senses via
 *
 *    <WordSenseLink>
 *       <WordNode name="bark" />
 *       <ConceptNode name="bark_sense_23" />
 *    </WordSenseLink>
 *  
 */
template <class T>
inline void foreach_dict_word_sense(Handle h, bool (T::*cb)(Handle), T *data)
{
	ForeachChaseLink<T> chase;
	chase.follow_binary_link(h, WORD_SENSE_LINK, cb, data);
}

}

#endif /* OPENCOG_FOREACH_WORD_H */

