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

	// First, we masquerade
// XXX TODO respect  the type definitions, too!!!!
#if 0
	var_solutn_stack.push(var_grounding);
	const HandleSeq& local_args(cpl->get_local_args
	for (const Handle& arg : cpl->get_args())
	{
		
	}
#endif


	Handle expanded_pattern(cpl->beta_reduce());
	return tree_compare(expanded_pattern, Handle(lg));
}

/* ===================== END OF FILE ===================== */
