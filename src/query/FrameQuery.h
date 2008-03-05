/**
 * FrameQuery.h
 *
 * Impelement query processing at the semantic frame level.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_FRAME_QUERY_H_
#define OPENCOG_FRAME_QUERY_H_

#include <map>

#include "AtomSpace.h"
#include "PatternMatch.h"
#include "RelexQuery.h"

namespace opencog {

class FrameQuery : public RelexQuery
{
	protected:
		bool apply_rule(Atom *atom);

	public:
		FrameQuery(void);
		virtual ~FrameQuery();

		/* Callbacks called from PatternMatch */
		virtual bool node_match(Atom *, Atom *);
};

}
#endif /* OPENCOG_FRAME_QUERY_H_ */
