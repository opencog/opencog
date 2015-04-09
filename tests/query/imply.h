
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atoms/bind/BindLink.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/query/DefaultImplicator.h>

using namespace opencog;

/**
 * Default evaluator of implication statements.  Does not consider
 * the truth value of any of the matched clauses; instead, looks
 * purely for a structural match.
 */
static inline Handle imply(AtomSpace* as, Handle himplication)
{
	LinkPtr limp(LinkCast(himplication));
	OC_ASSERT(limp != NULL, "Bad implication link");

	Handle hclauses = limp->getOutgoingAtom(0);

	// Extract the variables; they were not specified.
	FindAtoms fv(VARIABLE_NODE);
	fv.search_set(hclauses);

	HandleSeq vars;
	for (Handle h : fv.varset)
	{
		vars.push_back(h);
	}

	// Stuff the variables into a proper variable list.
	Handle hvars(createLink(VARIABLE_LIST, vars));

	HandleSeq oset;
	oset.push_back(hvars);
	oset.push_back(himplication);

	BindLinkPtr bl(createBindLink(oset));

	// Now perform the search.
	DefaultImplicator impl(as);
	impl.implicand = limp->getOutgoingAtom(1);

	bl->imply(&impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = as->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * Pattern Matcher. Just run the matcher against the indicated
 * variables and clauses, using eh indicated callback.
 */
static inline void match(PatternMatchCallback* pmcb,
                         const std::set<Handle> &vars,
                         const std::vector<Handle> &clauses)
{
	SatisfactionLinkPtr slp(createSatisfactionLink(vars, clauses));
	slp->satisfy(pmcb);
}
