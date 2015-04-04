/*
 * Composition.cc
 *
 * Copyright (C) 2015 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  aPRIL 2015
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

#include <opencog/atoms/bind/BetaRedex.h>

#include "PatternMatchEngine.h"
#include "PatternMatchCallback.h"

using namespace opencog;

// Uncomment below to enable debug print
// #define DEBUG
#ifdef DEBUG
	#define dbgprt(f, varargs...) printf(f, ##varargs)
#else
	#define dbgprt(f, varargs...)
#endif

/* ================================================================= */

/* Reset the current variable grounding to the last grounding pushed
 * onto the stack. */
#define POPGND(soln,stack) {         \
	OC_ASSERT(not stack.empty(), "Unbalanced grounding stack"); \
	soln = stack.top();               \
	stack.pop();                      \
}

bool PatternMatchEngine::redex_compare(const LinkPtr& lp,
                                       const LinkPtr& lg)
{
	// If we are here, the pattern is defined in a DefineLink. We
	// must match to that. There seem to be two strategies for doing
	// that:  Method A: rename all of the variables in the defined
	// pattern to be the variables we are actually using in the
	// top-level search.  This seems easy, but it is wrong, for two
	// reasons. One reason is that, after renaming, we will have
	// created a pattern that is probably not in the atomspace.
	// That means that the pattern will have atoms with invalid UUID's
	// in them, causing trouble down the line. The other problem is
	// that the variables in the defined target now look like perfectly
	// good grounding candidates, and so get found and reported as valid
	// grounds. So, for these two reasons, the simple, "obvious" method
	// A is out. Instead, we implement method B: we rename the variables
	// that the match engine is carrying, to correspond with the variable
	// names that are native to the definition. This way, insde the body
	// of the definition, everything looks "normal", and should thus
	// proceed as formal.  Of course, on exit, we have to unmasquerade. 

	BetaRedexPtr cpl(BetaRedexCast(lp));

	// First, we masquerade: replace the redex vars by the vars
	// that they wrap.
// XXX TODO respect  the type definitions, too!!!!
	var_solutn_stack.push(var_grounding);
	const HandleSeq& local_args(cpl->get_local_args());
	const HandleSeq& redex_args(cpl->get_args());
	size_t sz = redex_args.size();
	for (size_t i=0; i< sz; i++)
	{
		auto iter = var_grounding.find(redex_args[i]);
		if (iter == var_grounding.end()) continue;
		var_grounding.insert({local_args[i], iter->second});
		var_grounding.erase(redex_args[i]);
	}
	bool have_match = tree_compare(Handle(lp), Handle(lg));

	// No match; restore original grounding and quit
	if (not have_match)
	{
		POPGND(var_grounding, var_solutn_stack);
		return false;
	}

	// If there is a match, then maybe we grounded some variables.
	// If so, we need to unmasquerade them.
	SolnMap tmp;
	for (size_t i=0; i< sz; i++)
	{
		auto iter = var_grounding.find(local_args[i]);
		if (iter != var_grounding.end())
			tmp.insert({redex_args[i], iter->second});
	}
	POPGND(var_grounding, var_solutn_stack);

	// Now, copy them into the grounding
	for (const std::pair<Handle, Handle> pare : tmp)
		var_grounding.insert(pare);

	return true;
}

/* ===================== END OF FILE ===================== */
