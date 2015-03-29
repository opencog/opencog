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
#include <opencog/atoms/bind/BindLink.h>
#include <opencog/query/Implicator.h>
#include <opencog/query/PatternMatchCallback.h>
#include <opencog/query/Satisfier.h>

namespace opencog {

class PatternMatch : public BindLink
{
	private:
		// See PatternMatch.cc for comments
		static int get_vartype(const Handle&,
		                       std::set<Handle>&,
		                       VariableTypeMap&);

		// See PatternMatch.cc for comments
		void do_match(PatternMatchCallback *,
		              std::set<Handle>& vars,
		              std::vector<Handle>& clauses);

		// See PatternMatch.cc for comments
		void do_imply(const Handle&, Implicator&, std::set<Handle>&);

		bool recursive_virtual(PatternMatchCallback *cb,
		            const std::vector<Handle>& virtuals,
		            const std::vector<Handle>& negations,
		            const std::map<Handle, Handle>& var_gnds,
		            const std::map<Handle, Handle>& pred_gnds,
		            std::vector<std::vector<std::map<Handle, Handle>>> comp_var_gnds,
		            std::vector<std::vector<std::map<Handle, Handle>>> comp_pred_gnds);

		/// Unbundled variables and types for them.
		/// _typemap is the (possibly empty) list of restrictions on
		/// the variable types.
		/// Set by validate_bindvars()
		std::set<Handle> _varset;
		VariableTypeMap _typemap;

		/// Handle of the body of the expression.
		Handle _body;

		/// The actual clauses. Set by validate_implication()
		Handle _hclauses;
		Handle _implicand;
		std::vector<Handle> _clauses;

		/// The graph components. Set by validate_clauses()
		std::vector<Handle> _virtuals;
		std::vector<Handle> _nonvirts;

		bool _used;

		// Extract variable decls and the body.
		void unbundle_body(const Handle&);

		// Validate the variable decls
		void validate_vardecl(const Handle&);

		// Validate the strcuture of the body
		void validate_body(const Handle&);

		// Validate the clauses inside the body
		void unbundle_clauses(const Handle&);
		void validate_clauses(std::set<Handle>& vars,
		                      std::vector<Handle>& clauses);

	public:
		PatternMatch(void);

		// See PatternMatch.cc for comments
		void validate(const Handle&);

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
