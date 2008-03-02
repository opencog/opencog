/**
 * RelexQuery.h
 *
 * Impelement query processing for relex based queries.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "PatternMatch.h"

namespace opencog {

class RelexQuery : public PatternMatchCallback
{
	public:
		virtual bool solution(void);
};

}
