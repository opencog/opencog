/**
 * FrameQuery.h
 *
 * Impelement query processing at the semantic frame level.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_FRAME_QUERY_H
#define _OPENCOG_FRAME_QUERY_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/nlp/question/WordRelQuery.h>

namespace opencog {

class FrameQuery : public WordRelQuery
{
	private:
		bool is_frame_elt(Atom *);

		bool do_discard;
		bool discard_question_markup(Atom *atom);
		bool discard_eval_markup(Atom *atom);
		bool discard_heir_markup(Atom *atom);
	protected:
		virtual bool assemble_predicate(Atom *atom);

	public:
		FrameQuery(void);
		virtual ~FrameQuery();

		/* Callbacks called from PatternMatch */
		virtual bool node_match(Node *, Node *);
};

} // namespace opencog

#endif // _OPENCOG_FRAME_QUERY_H
