
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/query/DefaultImplicator.h>
#include <opencog/query/PatternMatch.h>

using namespace opencog;

/**
 * Default evaluator of implication statements.  Does not consider
 * the truth value of any of the matched clauses; instead, looks
 * purely for a structural match.
 *
 * See the do_imply function for details.
 */
static inline Handle imply(AtomSpace* as, Handle himplication)
{
	// Perform the search.
	PatternMatch pm;
	DefaultImplicator impl(as);
	pm.do_imply(himplication, impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = as->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
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
static inline Handle crisp_logic_imply(AtomSpace* as, Handle himplication)
{
	// Perform the search.
	PatternMatch pm;
	CrispImplicator impl(as);
	pm.do_imply(himplication, impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = as->addLink(LIST_LINK, impl.result_list);
	return gl;
}


