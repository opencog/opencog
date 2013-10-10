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

#include <opencog/atomspace/Handle.h>

namespace opencog {

class AtomSpace;

class ParseRank
{
	private:
		Handle top;
		double top_rank;
		bool lookat_parse(Handle);
        AtomSpace* as;

	public:
		ParseRank(void);
		~ParseRank();
        void set_atom_space(AtomSpace* _as) { as = _as; }
		Handle get_top_ranked_parse(Handle);

};

} // namespace opencog

#endif // _OPENCOG_PARSE_RANK_H
