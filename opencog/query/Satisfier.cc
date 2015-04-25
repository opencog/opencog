/*
 * Satisfier.cc
 *
 * Copyright (C) 2015 Linas Vepstas
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atoms/bind/SatisfactionLink.h>

#include "BindLink.h"
#include "Satisfier.h"

using namespace opencog;

bool Satisfier::grounding(const std::map<Handle, Handle> &var_soln,
                           const std::map<Handle, Handle> &term_soln)
{
	// PatternMatchEngine::print_solution(term_soln, var_soln);
	_result = TruthValue::TRUE_TV();

	// Look for more groundings.
	return false;
}

TruthValuePtr opencog::satisfaction_link(AtomSpace* as, const Handle& hlink)
{
	Satisfier sater(as);

	SatisfactionLinkPtr bl(SatisfactionLinkCast(hlink));
	if (NULL == bl)
	{
		// If it is a BindLink (for example), we want to use that ctor
		// instead of the default ctor.
		if (classserver().isA(hlink->getType(), SATISFACTION_LINK))
			bl = createSatisfactionLink(*LinkCast(hlink));
		else
			bl = createSatisfactionLink(hlink);
	}

	bl->satisfy(sater);

	return sater._result;
}

/* ===================== END OF FILE ===================== */
