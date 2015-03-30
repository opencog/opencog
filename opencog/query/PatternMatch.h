/*
 * PatternMatch.h
 *
 * Author: Linas Vepstas February 2008
 *
 * Copyright (C) 2008,2009 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_PATTERN_MATCH_H
#define _OPENCOG_PATTERN_MATCH_H

#include <set>

#include <opencog/atomspace/Handle.h>
#include <opencog/query/Implicator.h>
#include <opencog/query/PatternMatchCallback.h>
#include <opencog/query/Satisfier.h>

namespace opencog {

class PatternMatch
{
	private:
		// See PatternMatch.cc for comments
		void do_match(PatternMatchCallback *,
		              const std::set<Handle>& vars,
		              const std::vector<Handle>& virtuals,
		              const std::set<std::vector<Handle>>& components);

		bool recursive_virtual(PatternMatchCallback *cb,
		            const std::vector<Handle>& virtuals,
		            const std::vector<Handle>& negations,
		            const std::map<Handle, Handle>& var_gnds,
		            const std::map<Handle, Handle>& pred_gnds,
		            std::vector<std::vector<std::map<Handle, Handle>>> comp_var_gnds,
		            std::vector<std::vector<std::map<Handle, Handle>>> comp_pred_gnds);

		bool _used;

	public:
		PatternMatch(void);

		void match(PatternMatchCallback *,
		           const Handle& vars,
		           const Handle& clauses);

		// See PatternMatch.cc for comments
		void do_bindlink(const Handle&, Implicator&);
		void do_satlink(const Handle&, Satisfier&);

		// Deprecated: DO NOT USE IN NEW ODE!
		// This is used only in the unit-test cases.
		void do_imply(const Handle&, Implicator&);
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_H
