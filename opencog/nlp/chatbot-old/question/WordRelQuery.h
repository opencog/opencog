/**
 * WordRelQuery.h
 *
 * Impelement query processing for relex based queries.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_WORD_REL_QUERY_H
#define _OPENCOG_WORD_REL_QUERY_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/FollowLink.h>
#include <opencog/query/DefaultPatternMatchCB.h>
#include <opencog/query/PatternMatchEngine.h>

namespace opencog {

class WordRelQuery : 
	public DefaultPatternMatchCB
{
	private:
		// Help determine if assertion is a query.
		bool is_qVar(Handle);

		// Convert query into a normal form.
		bool is_ling_cncpt(Atom *);
		bool is_cncpt(Atom *);

		// Aid in equivalent node identification.
		// const char * get_word_instance(Atom *);

	protected:
		AtomSpace *atom_space;
		FollowLink fl;

		// Aid in equivalent node identification.
		bool word_instance_match(Atom *, Atom *);

		// create the predicate
		// virtual bool assemble_predicate(Atom *);
		virtual bool find_vars(Handle);

		void add_to_predicate(Handle);
		void add_to_vars(Handle);

		// normalized predicates
		HandleSeq normed_predicate;
		HandleSeq bound_vars;

		// solver
		PatternMatchEngine *pme;

		bool is_word_a_query(Handle);
	public:
		WordRelQuery(void);
		virtual ~WordRelQuery();

		/* Callbacks called from PatternMatch */
		virtual bool node_match(Node *, Node *);
		virtual bool solution(HandleMap &pred_soln,
		                      HandleMap &var_soln);
};

} // namespace opencog

#endif // _OPENCOG_WORD_REL_QUERY_H
