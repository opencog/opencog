/**
 * RelexQuery.h
 *
 * Impelement query processing for relex based queries.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "FollowLink.h"
#include "PatternMatch.h"

namespace opencog {

class RelexQuery : public PatternMatchCallback
{
	private:
		bool concept_match(Atom *, Atom *);

		FollowLink fl;

	public:
		virtual bool node_match(Atom *, Atom *);
		virtual bool solution(void);
};

}
