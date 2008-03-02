/**
 * RelexQuery.h
 *
 * Impelement query processing for relex based queries.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_RELEX_QUERY_H_
#define OPENCOG_RELEX_QUERY_H_

#include <map>

#include "FollowLink.h"
#include "PatternMatch.h"

namespace opencog {

class RelexQuery : public PatternMatchCallback
{
	private:
		// Help determine if assertion is a query.
		bool match_node_name(Atom *);
		bool check_for_query(Handle);

		// Convert query into a normal form.
		bool is_ling_rel(Atom *);
		bool apply_rule(Atom *);
		bool find_vars(Handle);

		// Aid in equivalent node identification.
		bool concept_match(Atom *, Atom *);

		FollowLink fl;

	public:
		void setup(Handle);
		std::vector<Handle> normed_predicate;
		std::vector<Handle> bound_vars;

		bool is_query(Handle);

		/* Callbacks called from PatternMatch */
		virtual bool node_match(Atom *, Atom *);
		virtual bool solution(std::map<Handle, Handle> &pred_soln,
		                      std::map<Handle, Handle> &var_soln);
};

}
#endif /* OPENCOG_RELEX_QUERY_H_ */
