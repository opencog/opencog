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
			_varset.insert(h);    // tree (unordered)
			_varseq.push_back(h); // vector (ordered)
		}
		else if (TYPED_VARIABLE_LINK == t)
		{
			get_vartype(h);
		}
		else
			throw InvalidParamException(TRACE_INFO,
				"Expected a VariableNode or a TypedVariableLink, got: %s",
					classserver().getTypeName(t).c_str());
	}
	build_index();
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
	validate_vardecl(_outgoing);
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
void VariableList::get_vartype(const Handle& htypelink)
{
	const std::vector<Handle>& oset = LinkCast(htypelink)->getOutgoingSet();
	if (2 != oset.size())
	{
		throw InvalidParamException(TRACE_INFO,
			"TypedVariableLink has wrong size, got %lu", oset.size());
	}

	Handle varname = oset[0];
	Handle vartype = oset[1];

	// The vartype is either a single type name, or a list of typenames.
	Type t = vartype->getType();
	if (TYPE_NODE == t)
	{
		Type vt = TypeNodeCast(vartype)->getValue();
		std::set<Type> ts = {vt};
		_typemap.insert(ATPair(varname, ts));
		_varset.insert(varname);
		_varseq.push_back(varname);
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
				throw InvalidParamException(TRACE_INFO,
					"VariableChoice has unexpected content:\n"
				              "Expected TypeNode, got %s",
				              classserver().getTypeName(h->getType()).c_str());
			}
			Type vt = TypeNodeCast(h)->getValue();
			ts.insert(vt);
		}

		_typemap.insert(ATPair(varname,ts));
		_varset.insert(varname);
		_varseq.push_back(varname);
	}
	else
	{
		throw InvalidParamException(TRACE_INFO,
			"Unexpected contents in TypedVariableLink\n"
			"Expected TypeNode or TypeChoice, got %s",
			classserver().getTypeName(t).c_str());
	}
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
		get_vartype(hdecls);
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
 * we have memoized.  If this typelist contians more than one type in
 * it, then clearly, there is a mismatch.  If there are no type
 * restrictions, then it is trivially a match.  Otherwise, there must
 * be a TypeChoice, and so the handle must be one of the types in the
 * TypeChoice.
 */
bool VariableList::is_type(const Handle& h) const
{
	// The arity must be one for there to be a match.
	if (1 != _varset.size()) return false;

	// No type restrictions.
	if (0 == _typemap.size()) return true;

	// Check the type restrictions.
	VariableTypeMap::const_iterator it =_typemap.find(_varseq[0]);
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
bool VariableList::is_type(const HandleSeq& hseq) const
{
	// The arity must be one for there to be a match.
	size_t len = hseq.size();
	if (_varset.size() != len) return false;
	// No type restrictions.
	if (0 == _typemap.size()) return true;

	// Check the type restrictions.
	for (size_t i=0; i<len; i++)
	{
		VariableTypeMap::const_iterator it =_typemap.find(_varseq[i]);
		if (it == _typemap.end()) continue;  // no restriction

		const std::set<Type> &tchoice = it->second;
		Type htype = hseq[i]->getType();
		std::set<Type>::const_iterator allow = tchoice.find(htype);
		if (allow == tchoice.end()) return false;
	}
	return true;
}

/* ================================================================= */
/**
 * Build the reverse index from variable name, to its ordinal.
 */
void VariableList::build_index(void)
{
	if (0 < _index.size()) return;
	size_t sz = _varseq.size();
	for (size_t i=0; i<sz; i++)
	{
		_index.insert(std::pair<Handle,Arity>(_varseq[i], i));
	}
}

/* ================================================================= */
/**
 * Substitute variables in tree with the indicated values.
 * This is a lot like applying the function fun to the argument list
 * args, except that no actual evaluation is performed; only
 * substitution.  The resulting tree is NOT placed into any atomspace,
 * either. If you want that, you must do it youself.  If you want
 * evaluation or execution to happen during sustitution, use either
 * the EvaluationLink, the ExecutionOutputLink, or the Instantiator.
 *
 * So, for example, if this VariableList contains:
 *
 *   VariableList
 *       VariableNode $a
 *       VariableNode $b
 *
 * and func is the template:
 *
 *   EvaluationLink
 *      PredicateNode "something"
 *      ListLink
 *         VariableNode $b      ; note the reversed order
 *         VariableNode $a
 *
 * and the args is a list
 *
 *      ConceptNode "one"
 *      NumberNode 2.0000
 *
 * then the returned value will be
 *
 *   EvaluationLink
 *      PredicateNode "something"
 *      ListLink
 *          NumberNode 2.0000
 *          ConceptNode "one"
 *
 * That is, the values 1 and 2 were substituted for %a and $b.
 *
 * Again, only a substitution is performed, there is not evaluation.
 * Note also that the resulting tree is NOT placed into any atomspace!
 */
Handle VariableList::substitute(const Handle& fun,
                                const HandleSeq& args) const
{
	if (args.size() != _varseq.size())
		throw InvalidParamException(TRACE_INFO,
			"Incorrect numer of arguments specified, expecting %lu got %lu",
			_varseq.size(), args.size());

	if (not is_type(args))
		throw InvalidParamException(TRACE_INFO,
			"Arguments fail to match variable declarations");

	return substitute_nocheck(fun, args);
}

Handle VariableList::substitute_nocheck(const Handle& fun,
                                        const HandleSeq& args) const
{
	// If it is a singleton, just return that singleton.
	std::map<Handle, Arity>::const_iterator idx;
	idx = _index.find(fun);
	if (idx != _index.end())
		return args.at(idx->second);

	// If its a node, and its not a variable, then it is a constant,
	// and just return that.
	LinkPtr lfun(LinkCast(fun));
	if (NULL == lfun) return fun;

	// QuoteLinks halt the reursion
	if (QUOTE_LINK == fun->getType()) return fun;

	// Recursively fill out the subtrees.
	HandleSeq oset;
	for (const Handle& h : lfun->getOutgoingSet())
	{
		oset.push_back(substitute_nocheck(h, args));
	}
	return Handle(createLink(fun->getType(), oset));
}

/* ===================== END OF FILE ===================== */
