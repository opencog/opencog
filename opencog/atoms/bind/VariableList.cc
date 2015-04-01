/*
 * VariableList.cc
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atoms/TypeNode.h>

#include "VariableList.h"

using namespace opencog;

void VariableList::validate_vardecl(const HandleSeq& oset)
{
	for (Handle h: oset)
	{
		Type t = h->getType();
		if (VARIABLE_NODE == t)
		{
			_varset.insert(h);
		}
		else if (TYPED_VARIABLE_LINK == t)
		{
			if (get_vartype(h, _varset, _typemap))
				throw InvalidParamException(TRACE_INFO,
					"Don't understand the TypedVariableLink");
		}
		else
			throw InvalidParamException(TRACE_INFO,
				"Expected a VariableNode or a TypedVariableLink, got: %s",
					classserver().getTypeName(t).c_str());
	}
}

VariableList::VariableList(const HandleSeq& oset,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(VARIABLE_LIST, oset, tv, av)
{
	validate_vardecl(oset);
}

VariableList::VariableList(Type t, const HandleSeq& oset,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(t, oset, tv, av)
{
	// derived classes have a different initialization order
	if (VARIABLE_LIST != t) return;
	validate_vardecl(oset);
}

VariableList::VariableList(Link &l)
	: Link(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, VARIABLE_LIST))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a VariableList, got %s", tname.c_str());
	}

	// Dervided types have a different initialization sequence
	if (VARIABLE_LIST != tscope) return;
	validate_vardecl(l.getOutgoingSet());
}

/* ================================================================= */
typedef std::pair<Handle, const std::set<Type> > ATPair;

/**
 * Extract the variable type(s) from a TypedVariableLink
 *
 * The call is expecting htypelink to point to one of the two
 * following structures:
 *
 *    TypedVariableLink
 *       VariableNode "$some_var_name"
 *       TypeNode  "ConceptNode"
 *
 * or
 *
 *    TypedVariableLink
 *       VariableNode "$some_var_name"
 *       TypeChoice
 *          TypeNode  "ConceptNode"
 *          TypeNode  "NumberNode"
 *          TypeNode  "WordNode"
 *
 * In either case, the variable itself is appended to "vset",
 * and the list of allowed types are associated with the variable
 * via the map "typemap".
 */
int VariableList::get_vartype(const Handle& htypelink,
                            std::set<Handle> &vset,
                            VariableTypeMap &typemap)
{
	const std::vector<Handle>& oset = LinkCast(htypelink)->getOutgoingSet();
	if (2 != oset.size())
	{
		logger().warn("%s: TypedVariableLink has wrong size",
		              __FUNCTION__);
		return 1;
	}

	Handle varname = oset[0];
	Handle vartype = oset[1];

	// The vartype is either a single type name, or a list of typenames.
	Type t = vartype->getType();
	if (TYPE_NODE == t)
	{
		Type vt = TypeNodeCast(vartype)->getValue();
		std::set<Type> ts = {vt};
		typemap.insert(ATPair(varname, ts));
		vset.insert(varname);
	}
	else if (TYPE_CHOICE == t)
	{
		std::set<Type> ts;

		const std::vector<Handle>& tset = LinkCast(vartype)->getOutgoingSet();
		size_t tss = tset.size();
		for (size_t i=0; i<tss; i++)
		{
			Handle h(tset[i]);
			Type var_type = h->getType();
			if (TYPE_NODE != var_type)
			{
				logger().warn("%s: VariableChoiceLink has unexpected content:\n"
				              "Expected TypeNode, got %s",
				              __FUNCTION__,
				              classserver().getTypeName(h->getType()).c_str());
				return 3;
			}
			Type vt = TypeNodeCast(h)->getValue();
			ts.insert(vt);
		}

		typemap.insert(ATPair(varname,ts));
		vset.insert(varname);
	}
	else
	{
		logger().warn("%s: Unexpected contents in TypedVariableLink\n"
				        "Expected TypeNode or TypeChoice, got %s",
		              __FUNCTION__,
		              classserver().getTypeName(t).c_str());
		return 2;
	}

	return 0;
}

/* ================================================================= */
/**
 * Validate variable declarations for syntax correctness.
 *
 * This will check to make sure that a set of variable declarations are
 * of a reasonable form. Thus, for example, a structure similar to the
 * below is expected.
 *
 *       VariableList
 *          VariableNode "$var0"
 *          VariableNode "$var1"
 *          TypedVariableLink
 *             VariableNode "$var2"
 *             TypeNode  "ConceptNode"
 *          TypedVariableLink
 *             VariableNode "$var3"
 *             TypeChoice
 *                 TypeNode  "PredicateNode"
 *                 TypeNode  "GroundedPredicateNode"
 *
 * As a side-effect, the variables and type restrictions are unpacked.
 */

void VariableList::validate_vardecl(const Handle& hdecls)
{
	// Expecting the declaration list to be either a single
	// variable, or a list of variable declarations
	Type tdecls = hdecls->getType();
	if ((VARIABLE_NODE == tdecls) or
	    NodeCast(hdecls)) // allow *any* node as a variable
	{
		_varset.insert(hdecls);
	}
	else if (TYPED_VARIABLE_LINK == tdecls)
	{
		if (get_vartype(hdecls, _varset, _typemap))
			throw InvalidParamException(TRACE_INFO,
				"Cannot understand the typed variable definition");
	}
	else if (VARIABLE_LIST == tdecls or LIST_LINK == tdecls)
	{
		// The list of variable declarations should be .. a list of
		// variables! Make sure its as expected.
		const std::vector<Handle>& dset = LinkCast(hdecls)->getOutgoingSet();
		validate_vardecl(dset);
	}
	else
	{
		throw InvalidParamException(TRACE_INFO,
			"Expected a VariableList holding variable declarations");
	}
}

/* ================================================================= */
/**
 * Simple type checker.
 *
 * Returns true/false if the indicated handle is of the type that
 * we have memoized.
 */
bool VariableList::is_type(const Handle& h)
{
	// The arity must be one for there to be a match.
	if (1 != _varset.size()) return false;

	// No type restrictions.
	if (0 == _typemap.size()) return true;

	// Check the type restrictions.
	VariableTypeMap::const_iterator it =_typemap.find(_outgoing[0]);
	const std::set<Type> &tchoice = it->second;

	Type htype = h->getType();
	std::set<Type>::const_iterator allow = tchoice.find(htype);
	return allow != tchoice.end();
}

/* ================================================================= */
/**
 * Very simple type checker.
 *
 * Returns true/false if the indicated handles are of the type that
 * we have memoized.
 *
 * XXX TODO this does not currently handle type equations, as outlined
 * on the wiki; We would need the general pattern matcher to do type
 * checking, in that situation.
 */
bool VariableList::is_type(const HandleSeq& hseq)
{
	// The arity must be one for there to be a match.
	size_t len = hseq.size();
	if (_varset.size() != len) return false;
	// No type restrictions.
	if (0 == _typemap.size()) return true;

	// Check the type restrictions.
	for (size_t i=0; i<len; i++)
	{
		VariableTypeMap::const_iterator it =_typemap.find(_outgoing[i]);
		if (it == _typemap.end()) continue;  // no restriction

		const std::set<Type> &tchoice = it->second;
		Type htype = hseq[i]->getType();
		std::set<Type>::const_iterator allow = tchoice.find(htype);
		if (allow == tchoice.end()) return false;
	}
	return true;
}

/* ===================== END OF FILE ===================== */
