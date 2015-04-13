/*
 * Pattern.h
 *
 * Author: Linas Vepstas April 2015
 *
 * Copyright (C) 2015 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_PATTERN_H
#define _OPENCOG_PATTERN_H

#include <map>
#include <set>
#include <stack>
#include <unordered_map>
#include <vector>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/query/PatternMatchCallback.h>

namespace opencog {

struct Pattern
{
	// Private, locally scoped typedefs, not used outside of this class.
	typedef std::vector<Handle> RootList;
	typedef std::map<Handle, RootList> ConnectMap;
	typedef std::pair<Handle, RootList> ConnectPair;

	// Used for managing ChoiceLink state
	typedef std::pair<const Handle&, const Handle&> Choice;
	typedef std::map<Choice, size_t> ChoiceState;

	// -------------------------------------------
	// The current set of clauses (redex context) being grounded.

	std::string _redex_name;  // for debugging only!

	// variables that need to be grounded.
	std::set<Handle> _bound_vars;

	// List of clauses that need to be grounded.
	// See ConcreteLink.h for additional documentation.
	HandleSeq        _cnf_clauses;
	HandleSeq        _mandatory;
	std::set<Handle> _optionals;
	std::set<Handle> _evaluatable;
	std::set<Type>   _connectives;

	// Map from variables to e*uatble terms they appear in.
	std::unordered_multimap<Handle,Handle> _in_evaluatable;
	std::unordered_multimap<Handle,Handle> _in_executable;

	ConnectMap       _connectivity_map;
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_H
