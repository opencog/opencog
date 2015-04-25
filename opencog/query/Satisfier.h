/*
 * Satisfier.h
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas
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

#ifndef _OPENCOG_SATISFIER_H
#define _OPENCOG_SATISFIER_H

#include <vector>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/TruthValue.h>

#include <opencog/query/DefaultPatternMatchCB.h>

namespace opencog {

/**
 * class Satisfier -- pattern matching callback for checking satsifaction.
 *
 * This class is meant to be used with the pattern matcher. When the
 * pattern matcher calls the callback, it will do so with a particular
 * grounding of the search pattern.
 *
 * This will set the result TV to TRUE_TV if a grounding is found. More
 * sophisticated TV calculations can be obtained by overloading this class.
 */

class Satisfier :
	public virtual DefaultPatternMatchCB
{
	public:
		Satisfier(AtomSpace* as) :
			DefaultPatternMatchCB(as),
			_result(TruthValue::FALSE_TV()) {}
		TruthValuePtr _result;

		// Return true if a satisfactory grounding has been
		// found. Note that in case where you want all possible
		// groundings, this will usually return false, so the
		// patternMatchEngine can keep looking for ever more
		// groundings.
		virtual bool grounding(const std::map<Handle, Handle> &var_soln,
		                       const std::map<Handle, Handle> &term_soln);
};

/**
 * class SatisfactionSet -- pattern matching callback for finding satsifaction.
 *
 * This class is meant to be used with the pattern matcher. When the
 * pattern matcher calls the callback, it will do so with a particular
 * grounding of the search pattern.
 *
 * This will record every grounding that is found. Thus, after running,
 * the SatisfyingSet can be examined to see all the groundings that were
 * found.
 */

class SatisfactionSet :
	public virtual DefaultPatternMatchCB
{
	public:
		SatisfactionSet(AtomSpace* as) :
			DefaultPatternMatchCB(as) {}
		Handle _body;
		HandleSeq _satisfying_set;

		// Return true if a satisfactory grounding has been
		// found. Note that in case where you want all possible
		// groundings, this will usually return false, so the
		// patternMatchEngine can keep looking for ever more
		// groundings.
		virtual bool grounding(const std::map<Handle, Handle> &var_soln,
		                       const std::map<Handle, Handle> &term_soln);
};

}; // namespace opencog

#endif // _OPENCOG_SATISFIER_H
