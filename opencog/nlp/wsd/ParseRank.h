/*
 * ParseRank.h
 *
 * Place-holder -- does nothing except return the top-ranked parse,
 * as previously ranked by relex.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_PARSE_RANK_H
#define _OPENCOG_PARSE_RANK_H

#include <opencog/atoms/base/Handle.h>

namespace opencog {

class ParseRank
{
	private:
		Handle top;
		double top_rank;
		bool lookat_parse(const Handle&);

	public:
		ParseRank(void);
		~ParseRank();
		Handle get_top_ranked_parse(const Handle&);
};

} // namespace opencog

#endif // _OPENCOG_PARSE_RANK_H
