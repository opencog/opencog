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

namespace opencog {

class PatternMatch
{
	private:
		static int get_vartype(Handle,
		                std::set<Handle>&,
		                VariableTypeMap&);

		void do_match(PatternMatchCallback *,
		                std::set<Handle>& vars,
		                std::vector<Handle>& clauses,
		                std::vector<Handle>& negations)
			throw (InvalidParamException);

		void do_imply(Handle, Implicator&, std::set<Handle>&)
			throw (InvalidParamException);

		bool recursive_virtual(PatternMatchCallback *cb,
		            const std::vector<Handle>& virtuals,
		            const std::vector<Handle>& negations,
		            const std::map<Handle, Handle>& var_gnds,
		            const std::map<Handle, Handle>& pred_gnds,
		            std::vector<std::vector<std::map<Handle, Handle>>> comp_var_gnds,
		            std::vector<std::vector<std::map<Handle, Handle>>> comp_pred_gnds);

	public:
		PatternMatch(void);

		void match(PatternMatchCallback *,
		           Handle vars,
		           Handle clauses,
		           Handle negations = Handle::UNDEFINED)
			throw (InvalidParamException);

		void do_bindlink(Handle, Implicator&)
					throw (InvalidParamException);

		// Deprecated: used only in the unit-test cases.
		void do_imply(Handle, Implicator&)
			throw (InvalidParamException);
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_H
