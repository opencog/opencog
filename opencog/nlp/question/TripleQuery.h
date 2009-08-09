/**
 * TripleQuery.h
 *
 * * XXXXXXXXXXXXXXXXX
 *  * This code is more or less obsolete ... it represents a bad
 *   * design. This code should be removed ASAP. The chatbot no longer
 *    * uses it .. and this code will be killed once the chatbot is
 *    finally
 *     * refactored.
 *      * XXXXXXXXXXXXXXXXX
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

#ifndef _OPENCOG_TRIPLE_QUERY_H
#define _OPENCOG_TRIPLE_QUERY_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/nlp/question/WordRelQuery.h>

namespace opencog {

class TripleQuery : 
	public WordRelQuery
{
	public:
		TripleQuery(void);
		virtual ~TripleQuery();

		void clear(void);
		void add_triple(Handle);
		void solve(AtomSpace *);

};

} // namespace opencog

#endif // _OPENCOG_TRIPLE_QUERY_H
