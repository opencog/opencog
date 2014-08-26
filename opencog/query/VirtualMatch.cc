/*
 * VirtualMatch.cc
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
		bool virtual_link_match(LinkPtr& link1, Handle& args) {
			throw InvalidParamException(TRACE_INFO, "Not expecting a virtual link here!");
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

		// We put all of the temporary atoms into a temporary atomspace.
		// Everything in here will go poof and disappear when this class
		// is destructed. And that's OK, that's exactly what we want for
		// these temporaries.  The atomspace is blown away when we finish.
		AtomSpace aspace;
		Instantiator instor(&aspace);

		for (Handle virt : virtuals)
		{
			// At this time, we expect all virutal links to be
			// EvaluationLinks having the structure
			//
			//   EvaluationLink
			//       GroundedPredicateNode "scm:blah"
			//       ListLink
			//           Arg1Atom
			//           Arg2Atom
			//
			// So, we ground the ListLink, and pass that to the callback.
			LinkPtr lvirt(LinkCast(virt));
			Handle arglist(lvirt->getOutgoingAtom(1));

			// Ground the args that the virtual node needs.
			Handle gargs(instor.instantiate(arglist, var_gnds));

			// At last! Actually perform the test!
			bool match = cb->virtual_link_match(lvirt, gargs);

			// After checking, remove the temporary atoms. 
			// The most fool-proof way to do this is to blow
			// away the entire atomspace.
			// if (0 == gargs->getIncomingSetSize())
				// _atom_space->purgeAtom(gargs, false);

			if (match) return false;

			// FYI ... if the virtual_link_match() accepts the match, we
			// still don't instantiate the grounded form in the atomspace,
			// nor do we add the grounded form to pred_gnds.  The former
			// can be done by the callback, if desired.  We can't do the
			// latter without access to the former ... all of which might
			// need to be unwound, if the final match is rejected. So...
			// Hmmm. Unclear if this should change...
		}

		// Yay! We found one! We now have a fully and completely grounded
		// pattern! See what the callback thinks of it.
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

	// get_connected_components re-orders the clauses so that adjacent
	// clauses are connected.  using this will make matching slightly
	// faster. But we don't do this if there are negations, because the
	// above jammed the negations into the thing, which we must keep 
	// separate.
	if (0 == negations.size())
		clauses = *components.begin();

	// Are there any virtual links in the clauses? If so, then we need
	// to do some special handling.
	std::vector<Handle> virtuals;
	std::vector<Handle> nonvirts;
	for (Handle clause: clauses)
	{
		if (contains_atomtype(clause, GROUNDED_PREDICATE_NODE))
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
		if (not classserver().isA(vt, EVALUATION_LINK))
			throw InvalidParamException(TRACE_INFO,
				"Expecting EvaluationLink at the top level!");
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

/* ===================== END OF FILE ===================== */
