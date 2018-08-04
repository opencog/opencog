/**
 * SentenceQuery.h
 *
 * Impelement query processing for relex sentences queries.
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

#ifndef _OPENCOG_SENTENCE_QUERY_H
#define _OPENCOG_SENTENCE_QUERY_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/nlp/question/WordRelQuery.h>

namespace opencog {

class SentenceQuery : 
	public WordRelQuery
{
	private:
		bool parse_solve(Handle);
		bool wordlist_solve(Handle);
		bool word_solve(Handle);
		bool word_up(Handle);
		bool rel_up(Handle);

		bool is_ling_rel(Atom *);

		bool do_discard;
		bool discard_extra_markup(Atom *);

		// Help determine if assertion is a query.
		bool its_tq;
		bool is_tq(Handle);
		bool is_parse_a_query(Handle);
		bool is_parse_a_truth_query(Handle);
		bool is_wordlist_a_query(Handle);

	public:
		void solve(AtomSpace *, Handle);

		bool is_query(Handle);
};

} // namespace opencog

#endif // _OPENCOG_SENTENCE_QUERY_H
