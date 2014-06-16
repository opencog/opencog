/*
 * PatternMatch.cc
 *
 * Copyright (C) 2009, 2014 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
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

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/execution/GreaterThanLink.h>
#include <opencog/util/foreach.h>
#include <opencog/util/Logger.h>

#include "Instantiator.h"
#include "PatternMatch.h"
#include "PatternUtils.h"
#include "DefaultPatternMatchCB.h"
#include "CrispLogicPMCB.h"

using namespace opencog;

// #define DEBUG 1
#if DEBUG
	#define dbgprt(f, varargs...) printf(f, ##varargs)
#else
	#define dbgprt(f, varargs...)
#endif

PatternMatch::PatternMatch(void)
{
	_atom_space = NULL;
}

/* ================================================================= */
/// A pass-through class, which wraps a regular callback, but captures
/// all of the different possible groundings that result.  This class is
/// used to piece together graphs out of multiple components.
class PMCGroundings : public PatternMatchCallback
{
	private:
		PatternMatchCallback* _cb;

	public:
		PMCGroundings(PatternMatchCallback* cb) : _cb(cb) {}

		// Pass all the calls straight through, except one.
		bool node_match(Handle& node1, Handle& node2) {
			return _cb->node_match(node1, node2);
		}
		bool variable_match(Handle& node1, Handle& node2) {
			return _cb->variable_match(node1, node2);
		}
		bool link_match(LinkPtr& link1, LinkPtr& link2) {
			return _cb->link_match(link1, link2);
		}
		bool post_link_match(LinkPtr& link1, LinkPtr& link2) {
			return _cb->post_link_match(link1, link2);
		}
		bool clause_match(Handle& pattrn_link_h, Handle& grnd_link_h) {
			return _cb->clause_match(pattrn_link_h, grnd_link_h);
		}
		bool optional_clause_match(Handle& pattrn, Handle& grnd) {
			return _cb->optional_clause_match(pattrn, grnd);
		}
		IncomingSet get_incoming_set(Handle h) {
			return _cb->get_incoming_set(h);
		}
		void push(void) { _cb->push(); }
		void pop(void) { _cb->pop(); }
		void set_type_restrictions(VariableTypeMap &tm) {
			_cb->set_type_restrictions(tm);
		}
		void perform_search(PatternMatchEngine* pme,
                          std::set<Handle> &vars,
                          std::vector<Handle> &clauses,
                          std::vector<Handle> &negations)
		{
			_cb->perform_search(pme, vars, clauses, negations);
		}

		// This one we don't pass through. Instead, we collect the
		// groundings.
		bool grounding(const std::map<Handle, Handle> &var_soln,
		               const std::map<Handle, Handle> &pred_soln)
		{
			_pred_groundings.push_back(pred_soln);
			_var_groundings.push_back(var_soln);
			return false;
		}

		std::vector<std::map<Handle, Handle>> _pred_groundings;
		std::vector<std::map<Handle, Handle>> _var_groundings;
};

/**
 * Recursive evaluator/grounder/unifier of virtual link types.
 * The virtual links are in virtuals, a partial set of groundings
 * are in var_gnds and pred_gnds, and a collection of possible
 * groundings for disconnected graph components are in comp_var_gnds
 * and comp_pred_gnds.
 *
 * Notes below explain the recursive step: how the various disconnected
 * components are brought together int a candidate grounding. That
 * candidate is then run through each of the virtual links.  If these
 * accept the grounding, then the callback is called to make the final
 * determination.
 *
 * The recursion step terminates when comp_var_gnds, comp_pred_gnds
 * are empty, at which point the actual unification is done.
 */
bool PatternMatch::recursive_virtual(PatternMatchCallback *cb,
            const std::vector<Handle>& virtuals,
            const std::vector<Handle>& negations, // currently ignored
            const std::map<Handle, Handle>& var_gnds,
            const std::map<Handle, Handle>& pred_gnds,
            // copies, NOT references!
            std::vector<std::vector<std::map<Handle, Handle>>> comp_var_gnds,
            std::vector<std::vector<std::map<Handle, Handle>>> comp_pred_gnds)
{
	// If we are done with the recursive step, then submit the grounding
	// to the virtual links, and see what happens.
	if (0 == comp_var_gnds.size())
	{
#ifdef DEBUG
		dbgprt("Explore combinatoric grounding %zd clauses:\n",
		       var_gnds.size(), pred_gnds.size());
		PatternMatchEngine::print_solution(var_gnds, pred_gnds);
#endif

		Instantiator instor(_atom_space);

		for (Handle virt : virtuals)
		{
			// At this time, we exepect all virutal links to be
			// GreaterThanLinks having the structure
			//   GreaterThanLink
			//       GroundedSchemaNode "scm:blah"
			//       ListLink
			//           Arg1Atom
			//           Arg2Atom
			//
			LinkPtr lvirt(LinkCast(virt));
			Handle schema(lvirt->getOutgoingAtom(0));
			Handle arglist(lvirt->getOutgoingAtom(1));

			// Ground the args that the virtual node needs.
			Handle gargs = instor.instantiate(arglist, var_gnds);

			// At last! Actually perform the test!
			bool relation_holds = GreaterThanLink::do_execute(_atom_space, schema, gargs);
			dbgprt("Virtual accept/reject: %d\n", relation_holds);

			// Make a weak effort to clean up after ourselves. This is not
			// obviously, entirely correct, since the instantiator might
			// have created other odd-ball structures with possibly
			// inapproprite truth values in them, etc. !?
			_atom_space->purgeAtom(gargs, false);

			// The virtual relation failed to hold. Try the
			// next grounding.
			if (not relation_holds) return false;
		}

		// Yay! We found one! See what the callback thinks!
		return cb->grounding(var_gnds, pred_gnds);
	}
	dbgprt("Component recursion: num comp=%zd\n", comp_var_gnds.size());

	// Recurse over all components. If component k has N_k groundings,
	// and there are m components, then we have to explore all
	// N_0 * N_1 * N_2 * ... N_m possible combinations of groundings.
	// We do this recursively, by poping N_m off the back, and calling
	// ourselves.
	//
	// vg and vp will be the collection of groundings for one of the
	// components (well, for compnent m, in the above notation.)
	std::vector<std::map<Handle, Handle>> vg = comp_var_gnds.back();
	comp_var_gnds.pop_back();
	std::vector<std::map<Handle, Handle>> pg = comp_pred_gnds.back();
	comp_pred_gnds.pop_back();

	size_t ngnds = vg.size();
	for (size_t i=0; i<ngnds; i++)
	{
		// Given a set of groundings, tack on those for this component,
		// and recurse, with one less component. We need to make a copy, of course.
		std::map<Handle, Handle> rvg(var_gnds);
		std::map<Handle, Handle> rpg(pred_gnds);

		const std::map<Handle, Handle>& cand_vg(vg[i]);
		const std::map<Handle, Handle>& cand_pg(pg[i]);
		rvg.insert(cand_vg.begin(), cand_vg.end());
		rpg.insert(cand_pg.begin(), cand_pg.end());

		bool accept = recursive_virtual(cb, virtuals, negations, rvg, rpg, comp_var_gnds, comp_pred_gnds);

		// Halt recursion immeditately if match is accepted.
		if (accept) return true;
	}
	return false;
}

/* ================================================================= */
/**
 * Ground (solve) a pattern; perform unification. That is, find one
 * or more groundings for the variables occuring in a collection of
 * clauses (a hypergraph). The hypergraph can be thought of as a
 * a 'predicate' which becomes 'true' when a grounding exists.
 *
 * The predicate is defined in terms of two hypergraphs: one is a
 * hypergraph defining a pattern to be grounded, and the other is a
 * list of bound variables in the first.
 *
 * The bound variables are, by convention, VariableNodes.  (The code in
 * the pattern match engine doesn't care whether the variable nodes are
 * actually of type VariableNode, and so can work with variables that
 * are any kind of node. However, the default callbacks do check for
 * this type. Thus, the restriction, by convention, that the variables
 * must be of type VariableNode.)  The list of bound variables is then
 * assumed to be listed using the ListLink type. So, for example:
 *
 *    ListLink
 *        VariableNode "variable 1"
 *        VariableNode "another variable"
 *
 * The predicate hypergraph is assumed to be a list of "clauses", where
 * each "clause" should be thought of as the tree defined by the outging
 * sets in it.  The below assumes that the list of clauses is specified
 * by means of an AndLink, so, for example:
 *
 *     AndLink
 *        SomeLink ....
 *        SomeOtherLink ...
 *
 * The clauses are assumed to be connected by variables, i.e. each
 * clause has a variable that also appears in some other clause.  Even
 * more strongly, it is assumed that there is just one connected
 * component; the code below throws an error if there is more than one
 * connected component.  The reason for this is to avoid unintended
 * combinatoric explosions: the grounding of any one (connected)
 * component is completely independent of the grounding of any other
 * component.  So, if there are two components, and one has N groundings
 * and the other has M groundings, then the two together trivially have
 * MxN groundings. Its worse if there are 4, 4... components. Rather
 * than stupidly reporting a result MxNx... times, we just throw an
 * error, and let the user decide what to do.
 *
 * The grounding proceeds by requiring each clause to match some part
 * of the atomspace (i.e. of the universe of hypergraphs stored in the
 * atomspace). When a solution is found, PatternMatchCallback::solution
 * method is called, and it is passed two maps: one mapping the bound
 * variables to their groundings, and the other mapping the pattern
 * clauses to their corresponding grounded clauses.
 *
 * Note: the pattern matcher itself doesn't use the atomspace, or care
 * if the groundings live in the atomspace; it can search anything.
 * However, the default callbacks do use the atomspace to find an
 * initial starting point for the search, and thus the search defacto
 * happens on the atomspace.  This restriction can be lifted by tweaking
 * the callback that initially launches the search.
 *
 * At this time, the list of clauses is understood to be a single
 * disjunct; that is, all of the clauses must be simultaneously
 * satisfied.  A future extension could allow the use of MatchOrLinks
 * to support multiple exclusive disjuncts. See the README for more info.
 */
void PatternMatch::do_match(PatternMatchCallback *cb,
                            std::set<Handle>& vars,
                            std::vector<Handle>& clauses,
                            std::vector<Handle>& negations)

	throw (InvalidParamException)
{
	// Make sure that the user did not pass in bogus clauses
	// Make sure that every clause contains at least one variable.
	// The presence of constant clauses will mess up the current
	// pattern matcher.  Constant clauses are "trivial" to match,
	// and so its pointless to even send them through the system.
	bool bogus = remove_constants(vars, clauses);
	if (bogus)
	{
		logger().warn("%s: Constant clauses removed from pattern matching",
			__FUNCTION__);
	}

	bogus = remove_constants(vars, negations);
	if (bogus)
	{
		logger().warn("%s: Constant clauses removed from pattern negation",
			__FUNCTION__);
	}

	// Make sure that each declared variable appears in some clause.
	// We can't ground variables that aren't attached to something.
	for (Handle v : vars)
	{
		if ((not is_node_in_any_tree(clauses, v))
		     and (not is_node_in_any_tree(negations, v)))
		{
			std::stringstream ss;
			ss << "The variable " << v << " does not appear in any clause!";
			throw InvalidParamException(TRACE_INFO, ss.str().c_str());
		}
	}

	// Make sure that the pattern is connected
	std::set<std::vector<Handle>> components;

	if (0 < negations.size())
	{
		// The negations should be connected to the clauses.
		std::vector<Handle> all;
		all.reserve(clauses.size() + negations.size());
		all.insert(all.end(), clauses.begin(), clauses.end());
		all.insert(all.end(), negations.begin(), negations.end());
		get_connected_components(vars, all, components);
	}
	else
	{
		get_connected_components(vars, clauses, components);
	}
	if (1 != components.size())
	{
		// Users are going to be stumped by this one, so print
		// out a verbose, user-freindly debug message to help
		// them out.
		std::stringstream ss;
		ss << "Pattern is not connected! Found "
			<< components.size() << " components:\n";
		int cnt = 0;
		foreach (auto comp, components)
		{
			ss << "Connected component " << cnt
				<< " consists of ----------------: \n";
			foreach (Handle h, comp) ss << h->toString();
			cnt++;
		}
		throw InvalidParamException(TRACE_INFO, ss.str().c_str());
	}

	// get_connected_components places the clauses in connection-
	// sorted order. Use that, it makes matching slightly faster.
	if (0 == negations.size())
		clauses = *components.begin();

	// Are there any virtual links in the clauses? If so, then we need
	// to do some special handling.
	std::vector<Handle> virtuals;
	std::vector<Handle> nonvirts;
	for (Handle clause: clauses)
	{
		if (contains_linktype(clause, VIRTUAL_LINK))
			virtuals.push_back(clause);
		else
			nonvirts.push_back(clause);
	}

	// The simple case -- unit propagation through all of the clauses.
	if (0 == virtuals.size())
	{
		PatternMatchEngine pme;
		pme.match(cb, vars, clauses, negations);
		return;
	}

	// For now, the virtual links must be at the top. That's because
	// I don't understand what the semantics would be if they were
	// anywhere else... need to ask ben on the mailing list.
	for (Handle v : virtuals)
	{
		Type vt = v->getType();
		if (not classserver().isA(vt, VIRTUAL_LINK))
			throw InvalidParamException(TRACE_INFO,
				"Expeting VirtualLink at the top level!");
	}

	// I'm too lazy to do the optional/negated clause bit, just right
	// now. It adds complexity. So just throw, at this time.
	if (0 < negations.size())
		throw InvalidParamException(TRACE_INFO,
			"Patterns with both virtual and optional clauses not yet supported!");

	// If we are here, then we've got a knot in the center of it all.
	// Removing the virtual clauses from the hypergraph typically causes
	// the hypergraph to fall apart into multiple components, (i.e. none
	// are connected to one another). Teh virtual clauses tie all of
	// these back together into a single connected graph.
	//
	// There are several solution strategies posible at this point.
	// The one that we will pursue, for now, is to first ground all of
   // the distinct components individually, and then run each possible
	// grounding combination through the virtual link, for the final
	// accept/reject determination.

	std::vector<Handle> empty;
	std::set<std::vector<Handle>> nvcomps;
	get_connected_components(vars, nonvirts, nvcomps);

	std::vector<std::vector<std::map<Handle, Handle>>> comp_pred_gnds;
	std::vector<std::vector<std::map<Handle, Handle>>> comp_var_gnds;
	for (std::vector<Handle> comp : nvcomps)
	{
		// Find the variables in each component.
		std::set<Handle> cvars;
		for (Handle v : vars)
		{
			if (is_node_in_any_tree(comp, v)) cvars.insert(v);
		}

		// Pass through the callbacks, collect up answers.
		PMCGroundings gcb(cb);
		PatternMatchEngine pme;
		pme.match(&gcb, cvars, comp, empty);

		comp_var_gnds.push_back(gcb._var_groundings);
		comp_pred_gnds.push_back(gcb._pred_groundings);
	}

	// And now, try grounding each of the virtual clauses.
	dbgprt("BEGIN component recursion: ====================== num comp=%zd num virts=%zd\n",
	       comp_var_gnds.size(), virtuals.size());
	std::map<Handle, Handle> empty_vg;
	std::map<Handle, Handle> empty_pg;
	recursive_virtual(cb, virtuals, negations,
	                  empty_vg, empty_pg,
	                  comp_var_gnds, comp_pred_gnds);
}

void PatternMatch::match(PatternMatchCallback *cb,
                         Handle hvarbles,
                         Handle hclauses,
                         Handle hnegates)
	throw (InvalidParamException)
{
	// Both must be non-empty.
	LinkPtr lclauses(LinkCast(hclauses));
	LinkPtr lvarbles(LinkCast(hvarbles));
	if (NULL == lclauses or NULL == lvarbles) return;

	// Types must be as expected
	Type tvarbles = hvarbles->getType();
	Type tclauses = hclauses->getType();
	if (LIST_LINK != tvarbles)
		throw InvalidParamException(TRACE_INFO,
			"Expected ListLink for bound variable list.");

	if (AND_LINK != tclauses)
		throw InvalidParamException(TRACE_INFO,
			"Expected AndLink for clause list.");

	// negation clauses are optionally present
	std::vector<Handle> negs;
	LinkPtr lnegates(LinkCast(hnegates));
	if (lnegates)
	{
		if (AND_LINK != lnegates->getType())
			throw InvalidParamException(TRACE_INFO,
				"Expected AndLink holding negated/otional clauses.");
		negs = lnegates->getOutgoingSet();
	}

	std::set<Handle> vars;
	foreach (Handle v, lvarbles->getOutgoingSet()) vars.insert(v);

	std::vector<Handle> clauses(lclauses->getOutgoingSet());

	do_match(cb, vars, clauses, negs);
}

/* ================================================================= */

namespace opencog {

/**
 * class Implicator -- pattern matching callback for grounding implicands.
 *
 * This class is meant to be used with the pattern matcher. When the
 * pattern matcher calls the callback, it will do so with a particular
 * grounding of the search pattern. If this class is holding an ungrounded
 * implicand, and will create a grounded version of the implicand. If
 * the implicand is already grounded, then it's a no-op -- this class
 * alone will *NOT* change its truth value.  Use a derived class for
 * this.
 *
 * The 'var_soln' argument in the callback contains the map from variables
 * to ground terms. 'class Instantiator' is used to perform the actual
 * grounding.  A list of grounded expressions is created in 'result_list'.
 */
class Implicator :
	public virtual PatternMatchCallback
{
	protected:
		AtomSpace *_as;
		Instantiator inst;
	public:
		Implicator(AtomSpace* as) : _as(as), inst(as) {}
		Handle implicand;
		std::vector<Handle> result_list;
		virtual bool grounding(const std::map<Handle, Handle> &var_soln,
		                       const std::map<Handle, Handle> &pred_soln);
};

bool Implicator::grounding(const std::map<Handle, Handle> &var_soln,
                           const std::map<Handle, Handle> &pred_soln)
{
	// PatternMatchEngine::print_solution(pred_soln,var_soln);
	Handle h = inst.instantiate(implicand, var_soln);
	if (Handle::UNDEFINED != h)
	{
		result_list.push_back(h);
	}
	return false;
}

} // namespace opencog

/* ================================================================= */
/**
 * do_imply -- Evaluate an ImplicationLink.
 *
 * Given an ImplicationLink, this method will "evaluate" it, matching
 * the predicate, and creating a grounded implicand, assuming the
 * predicate can be satisfied. Thus, for example, given the structure
 *
 *    ImplicationLink
 *       AndList
 *          EvaluationList
 *             PredicateNode "_obj"
 *             ListLink
 *                ConceptNode "make"
 *                VariableNode "$var0"
 *          EvaluationList
 *             PredicateNode "from"
 *             ListLink
 *                ConceptNode "make"
 *                VariableNode "$var1"
 *       EvaluationList
 *          PredicateNode "make_from"
 *          ListLink
 *             VariableNode "$var0"
 *             VariableNode "$var1"
 *
 * Then, if the atomspace also contains a parsed version of the English
 * sentence "Pottery is made from clay", that is, if it contains the
 * hypergraph
 *
 *    EvaluationList
 *       PredicateNode "_obj"
 *       ListLink
 *          ConceptNode "make"
 *          ConceptNode "pottery"
 *
 * and the hypergraph
 *
 *    EvaluationList
 *       PredicateNode "from"
 *       ListLink
 *          ConceptNode "make"
 *          ConceptNode "clay"
 *
 * Then, by pattern matching, the predicate part of the ImplicationLink
 * can be fulfilled, binding $var0 to "pottery" and $var1 to "clay".
 * These bindings are refered to as the 'groundings' or 'solutions'
 * to the variables. So, e.g. $var0 is 'grounded' by "pottery".
 *
 * Next, a grounded copy of the implicand is then created; that is,
 * the following hypergraph is created and added to the atomspace:
 *
 *    EvaluationList
 *       PredicateNode "make_from"
 *       ListLink
 *          ConceptNode "pottery"
 *          ConceptNode "clay"
 *
 * As the above example illustrates, this function expects that the
 * input handle is an implication link. It expects the implication link
 * to consist entirely of one disjunct (one AndList) and one (ungrounded)
 * implicand.  The variables are explicitly declared in the 'varlist'
 * argument to this function. These variables should be understood as
 * 'bound variables' in the usual sense of lambda-calculus. (It is
 * strongly suggested that variables always be declared as VariableNodes;
 * there are several spots in the code where this is explicitly assumed,
 * and declaring some other node type as a vaiable may lead to
 * unexpected results.)
 *
 * Pattern-matching proceeds by finding groundings for these variables.
 * When a pattern match is found, the variables can be understood as
 * being grounded by some explicit terms in the atomspace. This
 * grounding is then used to create a grounded version of the
 * (ungrounded) implicand. That is, the variables in the implicand are
 * substituted by their grounding values.  This method then returns a
 * list of all of the grounded implicands that were created.
 *
 * The act of pattern-matching to the predicate of the implication has
 * an implicit 'for-all' flavour to it: the pattern is matched to 'all'
 * matches in the atomspace.  However, with a suitably defined
 * PatternMatchCallback, the search can be terminated at any time, and
 * so this method can be used to implement a 'there-exists' predicate,
 * or any quantifier whatsoever.
 *
 * Note that this method can be used to create a simple forward-chainer:
 * One need only to take a set of implication links, and call this
 * method repeatedly on them, until one is exhausted.
 */

void PatternMatch::do_imply (Handle himplication,
                             PatternMatchCallback *pmc,
                             std::set<Handle>& varset)
	throw (InvalidParamException)
{
	// Must be non-empty.
	LinkPtr limplication(LinkCast(himplication));
	if (NULL == limplication)
		throw InvalidParamException(TRACE_INFO,
			"Expected ImplicationLink");

	// Type must be as expected
	Type timpl = himplication->getType();
	if (IMPLICATION_LINK != timpl)
		throw InvalidParamException(TRACE_INFO,
			"Expected ImplicationLink");

	const std::vector<Handle>& oset = limplication->getOutgoingSet();
	if (2 != oset.size())
		throw InvalidParamException(TRACE_INFO,
			"ImplicationLink has wrong size: %d", oset.size());

	Handle hclauses(oset[0]);
	Handle implicand(oset[1]);

	// Must be non-empty.
	LinkPtr lclauses(LinkCast(hclauses));
	if (NULL == lclauses)
		throw InvalidParamException(TRACE_INFO,
			"Expected non-empty set of clauses in the ImplicationLink");

	// The predicate is either an AndList, or a single clause
	// If its an AndList, then its a list of clauses.
	// XXX FIXME Perhaps, someday, some sort of embedded OrList should
	// be supported, allowing several different patterns to be matched
	// in one go. But not today, this is complex and low priority. See
	// the README for slighly more detail
	std::vector<Handle> affirm, negate;
	Type tclauses = hclauses->getType();
	if (AND_LINK == tclauses)
	{
		// Input is in conjunctive normal form, consisting of clauses,
		// or their negations. Split these into two distinct lists.
		// Any clause that is a NotLink is "negated"; strip off the
		// negation and put it into its own list.
		const std::vector<Handle>& cset = lclauses->getOutgoingSet();
		size_t clen = cset.size();
		for (size_t i=0; i<clen; i++)
		{
			Handle h(cset[i]);
			Type t = h->getType();
			if (NOT_LINK == t)
			{
				negate.push_back(LinkCast(h)->getOutgoingAtom(0));
			}
			else
			{
				affirm.push_back(h);
			}
		}
	}
	else
	{
		// There's just one single clause!
		affirm.push_back(hclauses);
	}

	// Extract the set of variables, if needed.
	// This is used only by the deprecated imply() function, as the
	// BindLink will include a list of variables up-front.
	if (0 == varset.size())
	{
		FindVariables fv;
		fv.find_vars(hclauses);
		varset = fv.varset;
	}

	// Now perform the search.
	Implicator *impl = dynamic_cast<Implicator *>(pmc);
	impl->implicand = implicand;
	do_match(pmc, varset, affirm, negate);
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
 *       VariableTypeNode  "ConceptNode"
 *
 * or
 *
 *    TypedVariableLink
 *       VariableNode "$some_var_name"
 *       ListLink
 *          VariableTypeNode  "ConceptNode"
 *          VariableTypeNode  "NumberNode"
 *          VariableTypeNode  "WordNode"
 *
 * In either case, the variable itself is appended to "vset",
 * and the list of allowed types are associated with the variable
 * via the map "typemap".
 */
int PatternMatch::get_vartype(Handle htypelink,
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
	if (VARIABLE_TYPE_NODE == t)
	{
		const std::string &tn = NodeCast(vartype)->getName();
		Type vt = classserver().getType(tn);

		if (NOTYPE == vt)
		{
			logger().warn("%s: VariableTypeNode specifies unknown type: %s\n",
			               __FUNCTION__, tn.c_str());
			return 4;
		}

		std::set<Type> ts;
		ts.insert(vt);
		typemap.insert(ATPair(varname,ts));
		vset.insert(varname);
	}
	else if (LIST_LINK == t)
	{
		std::set<Type> ts;

		const std::vector<Handle>& tset = LinkCast(vartype)->getOutgoingSet();
		size_t tss = tset.size();
		for (size_t i=0; i<tss; i++)
		{
			Handle h(tset[i]);
			if (VARIABLE_TYPE_NODE != h->getType())
			{
				logger().warn("%s: TypedVariableLink has unexpected content:\n"
				              "Expected VariableTypeNode, got %s",
				              __FUNCTION__,
				              classserver().getTypeName(h->getType()).c_str());
				return 3;
			}
			const std::string &tn = NodeCast(h)->getName();
			Type vt = classserver().getType(tn);
			if (NOTYPE == vt)
			{
				logger().warn("%s: VariableTypeNode specifies unknown type: %s\n",
				               __FUNCTION__, tn.c_str());
				return 5;
			}
			ts.insert(vt);
		}

		typemap.insert(ATPair(varname,ts));
		vset.insert(varname);
	}
	else
	{
		logger().warn("%s: Unexpected contents in TypedVariableLink\n"
				        "Expected VariableTypeNode or ListLink, got %s",
		              __FUNCTION__,
		              classserver().getTypeName(t).c_str());
		return 2;
	}

	return 0;
}

/* ================================================================= */
/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Given a BindLink containing variable declarations and an
 * ImplicationLink, this method will "evaluate" the implication, matching
 * the predicate, and creating a grounded implicand, assuming the
 * predicate can be satisfied. Thus, for example, given the structure
 *
 *    BindLink
 *       ListLink
 *          VariableNode "$var0"
 *          VariableNode "$var1"
 *       ImplicationLink
 *          AndList
 *             etc ...
 *
 * Evaluation proceeds as decribed in the "do_imply()" function above.
 * The whole point of the BindLink is to do nothing more than
 * to indicate the bindings of the variables, and (optionally) limit
 * the types of acceptable groundings for the varaibles.
 */

void PatternMatch::do_bindlink (Handle hbindlink,
                                PatternMatchCallback *pmc)
	throw (InvalidParamException)
{
	// Must be non-empty.
	LinkPtr lbl(LinkCast(hbindlink));
	if (NULL == lbl)
		throw InvalidParamException(TRACE_INFO,
			"Expecting a BindLink");

	// Type must be as expected
	Type tscope = hbindlink->getType();
	if (BIND_LINK != tscope)
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a BindLink, got %s", tname.c_str());
	}

	const std::vector<Handle>& oset = lbl->getOutgoingSet();
	if (2 != oset.size())
		throw InvalidParamException(TRACE_INFO,
			"BindLink has wrong size %d", oset.size());

	Handle hdecls(oset[0]);  // VariableNode declarations
	Handle himpl(oset[1]);   // ImplicationLink

	// vset is the vector of variables.
	// typemap is the (possibly empty) list of restrictions on atom types.
	std::set<Handle> vset;
	VariableTypeMap typemap;

	// Expecting the declaration list to be either a single
	// variable, or a list of variable declarations
	Type tdecls = hdecls->getType();
	if ((VARIABLE_NODE == tdecls) or
	    NodeCast(hdecls)) // allow *any* node as a variable
	{
		vset.insert(hdecls);
	}
	else if (TYPED_VARIABLE_LINK == tdecls)
	{
		if (get_vartype(hdecls, vset, typemap))
			throw InvalidParamException(TRACE_INFO,
				"Cannot understand the typed variable definition");
	}
	else if (LIST_LINK == tdecls)
	{
		// The list of variable declarations should be .. a list of
		// variables! Make sure its as expected.
		const std::vector<Handle>& dset = LinkCast(hdecls)->getOutgoingSet();
		size_t dlen = dset.size();
		for (size_t i=0; i<dlen; i++)
		{
			Handle h(dset[i]);
			Type t = h->getType();
			if (VARIABLE_NODE == t)
			{
				vset.insert(h);
			}
			else if (TYPED_VARIABLE_LINK == t)
			{
				if (get_vartype(h, vset, typemap))
					throw InvalidParamException(TRACE_INFO,
						"Don't understand the TypedVariableLink");
			}
			else
				throw InvalidParamException(TRACE_INFO,
					"Expected a VariableNode or a TypedVariableLink");
		}
	}
  	else
	{
		throw InvalidParamException(TRACE_INFO,
			"Expected a ListLink holding variable declarations");
	}

	pmc->set_type_restrictions(typemap);
	do_imply(himpl, pmc, vset);
}

/* ================================================================= */

namespace opencog {

class DefaultImplicator:
	public virtual Implicator,
	public virtual DefaultPatternMatchCB
{
	public:
		DefaultImplicator(AtomSpace* asp) : Implicator(asp), DefaultPatternMatchCB(asp) {}
};

class CrispImplicator:
	public virtual Implicator,
	public virtual CrispLogicPMCB
{
	public:
		CrispImplicator(AtomSpace* asp) :
			Implicator(asp), DefaultPatternMatchCB(asp), CrispLogicPMCB(asp)
		{}
		virtual bool grounding(const std::map<Handle, Handle> &var_soln,
		                       const std::map<Handle, Handle> &pred_soln);
};

} // namespace opencog

/**
 * The crisp implicator needs to tweak the truth value of the
 * resulting implicand. In most cases, this is not (strictly) needed,
 * for example, if the implicand has ungrounded variables, then
 * a truth value can be assigned to it, and the implicand will obtain
 * that truth value upon grounding.
 *
 * HOWEVER, if the implicand is fully grounded, then it will be given
 * a truth value of (false, uncertain) to start out with, and, if a
 * solution is found, then the goal here is to change its truth value
 * to (true, certain).  That is the whole point of this function:
 * to tweak (affirm) the truth value of existing clauses!
 */
bool CrispImplicator::grounding(const std::map<Handle, Handle> &var_soln,
                                const std::map<Handle, Handle> &pred_soln)
{
	// PatternMatchEngine::print_solution(pred_soln,var_soln);
	Handle h = inst.instantiate(implicand, var_soln);

	if (h != Handle::UNDEFINED)
	{
		result_list.push_back(h);

		// Set truth value to true+confident
		TruthValuePtr stv(SimpleTruthValue::createTV(1, SimpleTruthValue::confidenceToCount(1)));
		h->setTruthValue(stv);
	}
	return false;
}

class SingleImplicator:
	public virtual Implicator,
	public virtual DefaultPatternMatchCB
{
	public:
		SingleImplicator(AtomSpace* asp) : Implicator(asp), DefaultPatternMatchCB(asp) {}
		virtual bool grounding(const std::map<Handle, Handle> &var_soln,
		                       const std::map<Handle, Handle> &pred_soln);
};

/**
 * The single implicator behaves like the default implicator, except that
 * it terminates after the first solution is found.
 */
bool SingleImplicator::grounding(const std::map<Handle, Handle> &var_soln,
                                 const std::map<Handle, Handle> &pred_soln)
{
	Handle h = inst.instantiate(implicand, var_soln);

	if (h != Handle::UNDEFINED)
	{
		result_list.push_back(h);
	}
	return true;
}

/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Use the default implicator to find pattern-matches. Associated truth
 * values are completely ignored during pattern matching; if a set of
 * atoms that could be a ground are found in the atomspace, then they
 * will be reported.
 *
 * See the do_bindlink function documentation for details.
 */
Handle PatternMatch::bindlink (Handle himplication)
{
	// Now perform the search.
	DefaultImplicator impl(_atom_space);
	do_bindlink(himplication, &impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Returns the first match only. Otherwise, the behavior is identical to
 * PatternMatch::bindlink above.
 *
 * See the do_bindlink function documentation for details.
 */
Handle PatternMatch::single_bindlink (Handle himplication)
{
	// Now perform the search.
	SingleImplicator impl(_atom_space);
	do_bindlink(himplication, &impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Use the crisp-logic callback to evaluate boolean implication
 * statements; i.e. statements that have truth values assigned
 * their clauses, and statements that start with NotLink's.
 * These are evaluated using "crisp" logic: if a matched clause
 * is true, its accepted, if its false, its rejected. If the
 * clause begins with a NotLink, true and false are reversed.
 *
 * The NotLink is also interpreted as an "absence of a clause";
 * if the atomspace does NOT contain a NotLink clause, then the
 * match is considered postive, and the clause is accepted (and
 * it has a null or "invalid" grounding).
 *
 * See the do_bindlink function documentation for details.
 */
Handle PatternMatch::crisp_logic_bindlink (Handle himplication)
{
	// Now perform the search.
	CrispImplicator impl(_atom_space);
	do_bindlink(himplication, &impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/* ================================================================= */
/**
 * DEPRECATED: USE BIND_LINK INSTEAD!
 * Right now, this method is used only in the unit test cases;
 * and it should stay that way.
 *
 * Default evaluator of implication statements.  Does not consider
 * the truth value of any of the matched clauses; instead, looks
 * purely for a structural match.
 *
 * See the do_imply function for details.
 */
Handle PatternMatch::imply (Handle himplication)
{
	// Now perform the search.
	DefaultImplicator impl(_atom_space);
	std::set<Handle> varset;

	do_imply(himplication, &impl, varset);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * DEPRECATED: USE CRISP_LOGIC_BINDLINK INSTEAD!
 * At this time, this method is used only by the unit test cases.
 * It should stay that way, too; no one else should use this.
 *
 * Use the crisp-logic callback to evaluate boolean implication
 * statements; i.e. statements that have truth values assigned
 * their clauses, and statements that start with NotLink's.
 * These are evaluated using "crisp" logic: if a matched clause
 * is true, its accepted, if its false, its rejected. If the
 * clause begins with a NotLink, true and false are reversed.
 *
 * The NotLink is also interpreted as an "absence of a clause";
 * if the atomspace does NOT contain a NotLink clause, then the
 * match is considered postive, and the clause is accepted (and
 * it has a null or "invalid" grounding).
 *
 * See the do_imply function for details.
 */
Handle PatternMatch::crisp_logic_imply (Handle himplication)
{
	// Now perform the search.
	CrispImplicator impl(_atom_space);
	std::set<Handle> varset;

	do_imply(himplication, &impl, varset);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/* ===================== END OF FILE ===================== */
