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

#include <opencog/atoms/bind/ComposeLink.h>

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

bool PatternMatchEngine::compose_compare(const LinkPtr& lp,
                                         const LinkPtr& lg)
{
	// If the pattern is defined elsewhere, not here, then we have
	// to go to where it is defined, and pattern match things there.
	// The tricky part is that we have to pass on our current state,
	// i.e. the variables that we do have, the groundings we already
	// have, and see if that makes things work.
	ComposeLinkPtr cpl(ComposeLinkCast(lp));

	Handle expanded_pattern(cpl->compose());
	return tree_compare(expanded_pattern, Handle(lg));
#if 0
	// Get the args that the compose link is expecting...
	const HandleSeq& args = cpl->getArgs();
	HandleSeq grounds;
	for (const Handle& harg : args)
	{
		const Handle& gnd(var_grounding[harg]);
		if (Handle::UNDEFINED != gnd)
			grounds.push_back(gnd);
		else
			grounds.push_back(harg);
	}
	cpl->satisfy(pmc, grounds);
	return true; // XXX this is totally wrong
#endif
}

/* ===================== END OF FILE ===================== */
