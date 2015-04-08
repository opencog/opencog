/*
 * ConcreteLink.cc
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

#include <opencog/util/Logger.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atomspace/Node.h>

#include "ConcreteLink.h"
#include "PatternUtils.h"

using namespace opencog;

void ConcreteLink::init(void)
{
	LambdaLink::init(_outgoing);
	unbundle_clauses(_body);
	validate_clauses(_varset, _clauses);
	extract_optionals(_varset, _clauses);

	// check to make sure there are no virtual clauses.
	HandleSeq concs, virts;
	unbundle_virtual(_varset, _cnf_clauses,
	                 concs, virts);
	if (0 < virts.size())
	{
		throw InvalidParamException(TRACE_INFO,
			"ConcreteLink must not have virtuals");
	}

	// Check to make sure the graph is connected. As a side-effect,
	// the (only) component is sorted into connection-order.
   std::vector<HandleSeq> comps;
   std::vector<std::set<Handle>> comp_vars;
	get_connected_components(_varset, _cnf_clauses, comps, comp_vars);

	// throw error if more than one component
	check_connectivity(comps);

	// This puts them into connection-order.
	_cnf_clauses = comps[0];

	make_connectivity_map(_cnf_clauses);
}

// Special ctor for use by SatisfactionLink; we are given
// the pre-computed components.
ConcreteLink::ConcreteLink(const std::set<Handle>& vars,
                           const VariableTypeMap& typemap,
                           const HandleSeq& compo,
                           const std::set<Handle>& opts)
	: LambdaLink(CONCRETE_LINK, HandleSeq())
{
	// First, lets deal with the vars. We have discarded the original
	// order of the variables, and I think that's OK, because we will
	// not be using the substitute method, I don't think. If we need it,
	// then the API will need to be changed...
	// So all we need is the varset, and the subset of the typemap.
	_varset = vars;
	for (const Handle& v : vars)
	{
		auto it = typemap.find(v);
		if (it != typemap.end())
			_typemap.insert(*it);
	}

	// Next, the body... there no _body for lambda. The compo is the
	// _cnf_clauses; we have to reconstruct the optionals.  We cannot
	// use extract_optionals becuase opts have een stripped already.

	_cnf_clauses = compo;
	for (const Handle& h : compo)
	{
		bool h_is_opt = false;
		for (const Handle& opt : opts)
		{
			if (is_atom_in_tree(opt, h))
			{
				_optionals.insert(opt);
				_clauses.push_back(opt);
				h_is_opt = true;
				break;
			}
		}
		if (not h_is_opt)
			_mandatory.push_back(h);
	}

	// The rest is easy: the evaluatables and the connection map
	HandleSeq concs, virts;
	unbundle_virtual(_varset, _cnf_clauses,
	                 concs, virts);
	make_connectivity_map(_cnf_clauses);
}

ConcreteLink::ConcreteLink(const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(CONCRETE_LINK, hseq, tv, av)
{
	init();
}

ConcreteLink::ConcreteLink(const Handle& vars, const Handle& body,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(CONCRETE_LINK, HandleSeq({vars, body}), tv, av)
{
	init();
}

ConcreteLink::ConcreteLink(Type t, const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(t, hseq, tv, av)
{
	// Derived link-types have other init sequences
	if (CONCRETE_LINK != t) return;
	init();
}

ConcreteLink::ConcreteLink(Link &l)
	: LambdaLink(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, CONCRETE_LINK))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a ConcreteLink, got %s", tname.c_str());
	}

	// Derived link-types have other init sequences
	if (CONCRETE_LINK != tscope) return;

	init();
}


/* ================================================================= */
///
/// Unpack the clauses.
///
/// The predicate is either an AndLink of clauses to be satisfied, or a
/// single clause. Other link types, such as OrLink and SequentialAnd,
/// are treated here as single clauses; unpacking them here would lead
/// to confusion in the pattern matcher. This is patly because, after
/// unpacking, clauses can be grounded in  an arbitrary order; thus,
/// SequentialAnd's must not be unpacked. In the case of OrLinks, there
/// is no flag to say that "these are disjoined", so again, that has to
/// happen later.
void ConcreteLink::unbundle_clauses(const Handle& hbody)
{
	Type t = hbody->getType();
	if (AND_LINK == t)
	{
		_clauses = LinkCast(hbody)->getOutgoingSet();
	}
	else
	{
		// There's just one single clause!
		_clauses.push_back(hbody);
	}
}

/* ================================================================= */
/**
 * A simple validatation a collection of clauses for correctness.
 *
 * Every clause should contain at least one variable in it; clauses
 * that are constants and can be trivially discarded.
 */
void ConcreteLink::validate_clauses(std::set<Handle>& vars,
                                    HandleSeq& clauses)

{
	// The Fuzzy matcher does some strange things: it declares no
	// vars at all, only clauses, and then uses the pattern matcher
	// to automatically explore nearby atoms. As a result, all of
	// its clauses are "constant", and we allow this special case.
	// Need to review the rationality of this design...
	if (0 < vars.size())
	{
		// Make sure that the user did not pass in bogus clauses.
		// Make sure that every clause contains at least one variable.
		// The presence of constant clauses will mess up the current
		// pattern matcher.  Constant clauses are "trivial" to match,
		// and so its pointless to even send them through the system.
		bool bogus = remove_constants(vars, clauses);
		if (bogus)
		{
			logger().warn("%s: Constant clauses removed from pattern",
			              __FUNCTION__);
		}
	}

	// Make sure that each declared variable appears in some clause.
	// We won't (can't) ground variables that don't show up in a
	// clause.  They are presumably there due to programmer error.
	// Quoted variables are constants, and so don't count.
	//
	// XXX Well, we could throw, here, but sureal gives us spurious
	// variables, so instead of throwing, we just discard them and
	// print a warning.
	for (const Handle& v : vars)
	{
		if (not is_unquoted_in_any_tree(clauses, v))
		{
/*
			logger().warn(
				"%s: The variable %s does not appear (unquoted) in any clause!",
			           __FUNCTION__, v->toShortString().c_str());
*/
			vars.erase(v);
			throw InvalidParamException(TRACE_INFO,
			   "The variable %s does not appear (unquoted) in any clause!",
			   v->toShortString().c_str());
		}
	}

	// The above 1-2 combination of removing constant clauses, and
	// removing variables, can result in an empty body. That surely
	// warrants a throw, and BuggyQuoteUTest is expecting one.
	if (0 == vars.size() and 0 == clauses.size())
			throw InvalidParamException(TRACE_INFO,
			   "No variable appears (unquoted) anywhere in any clause!");
}

/* ================================================================= */
/**
 * Given the initial list of variables and clauses, separate these into
 * the mandatory and optional clauses.
 */
void ConcreteLink::extract_optionals(const std::set<Handle> &vars,
                                     const std::vector<Handle> &component)
{
	// Split in positive and negative clauses
	for (const Handle& h : component)
	{
		Type t = h->getType();
		if (NOT_LINK == t or ABSENT_LINK == t)
		{
			Handle inv(LinkCast(h)->getOutgoingAtom(0));
			_optionals.insert(inv);
			_cnf_clauses.push_back(inv);
		}
		else
		{
			_mandatory.push_back(h);
			_cnf_clauses.push_back(h);
		}
	}
}

/* ================================================================= */

/// Sort out the list of clauses into three classes:
/// virtual, evaluatable and concrete.
///
/// A term is "evalutable" if it contains a GroundedPredicate,
/// or if it inherits from VirtualLink (such as the GreaterThanLink).
/// Such terms need evaluation at grounding time, to determine
/// thier truth values.
///
/// A clause is "virtual" if it has an evaluatable term inside of it,
/// and that term takes an argument that is a variable. Such virtual
/// clauses not only require evaluation, but an evaluation must be
/// performed for each different variable grounding.  Virtual clauses
/// get a very different (and more complex) treatment from the pattern
/// matcher.
///
void ConcreteLink::unbundle_virtual(const std::set<Handle>& vars,
                                    const HandleSeq& clauses,
                                    HandleSeq& fixed_clauses,
                                    HandleSeq& virtual_clauses)
{
	for (const Handle& clause: clauses)
	{
		bool is_virtual = false;
		FindAtoms fgpn(GROUNDED_PREDICATE_NODE, true);
		fgpn.find_atoms(clause);
		for (const Handle& sh : fgpn.least_holders)
		{
			_evaluatable_terms.insert(sh);
			if (any_unquoted_in_tree(sh, vars))
				is_virtual = true;
		}
		for (const Handle& sh : fgpn.holders)
			_evaluatable_holders.insert(sh);

		// Subclasses of VirtualLink, e.g. GreaterThanLink, which
		// are essentially a kind of EvaluationLink holding a GPN
		FindAtoms fgtl(VIRTUAL_LINK, true);
		fgtl.find_atoms(clause);
		// Unlike the above, its varset, not least_holders...
		// because its a link...
		for (const Handle& sh : fgtl.varset)
		{
			_evaluatable_terms.insert(sh);
			_evaluatable_holders.insert(sh);
			if (any_unquoted_in_tree(sh, vars))
				is_virtual = true;
		}
		for (const Handle& sh : fgtl.holders)
			_evaluatable_holders.insert(sh);

		if (is_virtual)
			virtual_clauses.push_back(clause);
		else
			fixed_clauses.push_back(clause);
	}
}

/* ================================================================= */
/**
 * Create a map that holds all of the clauses that a given atom
 * participates in.  In other words, it indicates all the places
 * where an atom is shared by multiple trees, and thus establishes
 * how the trees are connected.
 *
 * This is used for only one purpose: to find the next unsolved
 * clause. Perhaps this could be simplied somehow ...
 */
void ConcreteLink::make_connectivity_map(const HandleSeq& component)
{
	for (const Handle& h : _cnf_clauses)
	{
		make_map_recursive(h, h);
	}

	// Save some minor amount of space by erasing those atoms that
	// participate in only one clause. These atoms cannot be used
	// to determine connectivity between clauses, and so are un-needed.
	auto it = _connectivity_map.begin();
	auto end = _connectivity_map.end();
	while (it != end)
	{
		if (it->second.size() == 1)
			it = _connectivity_map.erase(it);
		else
			it++;
	}
}

void ConcreteLink::make_map_recursive(const Handle& root, const Handle& h)
{
	_connectivity_map[h].push_back(root);

	LinkPtr l(LinkCast(h));
	if (l)
	{
		for (const Handle& ho: l->getOutgoingSet())
			make_map_recursive(root, ho);
	}
}

/* ================================================================= */
/**
 * Check that all clauses are connected
 */
void ConcreteLink::check_connectivity(
	const std::vector<HandleSeq>& components)
{
	if (1 == components.size()) return;

	// Users are going to be stumped by this one, so print
	// out a verbose, user-freindly debug message to help
	// them out.
	std::stringstream ss;
	ss << "Pattern is not connected! Found "
	   << components.size() << " components:\n";
	int cnt = 1;
	for (const auto& comp : components)
	{
		ss << "Connected component " << cnt
		   << " consists of ----------------: \n";
		for (Handle h : comp) ss << h->toString();
		cnt++;
	}
	throw InvalidParamException(TRACE_INFO, ss.str().c_str());
}

/* ================================================================= */

void ConcreteLink::debug_print(const char* tag) const
{
	// Print out the predicate ...
	printf("\nRedex '%s' has following clauses:\n", tag);
	int cl = 0;
	for (const Handle& h : _mandatory)
	{
		printf("Mandatory %d:\n", cl);
		if (_evaluatable_holders.find(h) != _evaluatable_holders.end())
			printf(" (evaluatable) ");
		prt(h);
		cl++;
	}

	if (0 < _optionals.size())
	{
		printf("Predicate includes the following optional clauses:\n");
		cl = 0;
		for (const Handle& h : _optionals)
		{
			printf("Optional clause %d: ", cl);
			prt(h);
			cl++;
		}
	}
	else
		printf("No optional clauses\n");

	// Print out the bound variables in the predicate.
	for (const Handle& h : _varset)
	{
		if (NodeCast(h))
			printf(" Bound var: "); prt(h);
	}

	if (_varset.empty())
		printf("There are no bound vars in this pattern\n");

	printf("\n");
	fflush(stdout);
}

/* ===================== END OF FILE ===================== */
