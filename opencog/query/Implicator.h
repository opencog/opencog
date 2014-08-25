/*
 * Implicator.h
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

#ifndef _OPENCOG_IMPLICATOR_H
#define _OPENCOG_IMPLICATOR_H

#include <opencog/atomspace/AtomSpace.h>

#include "AttentionalFocusCB.h"
#include "CrispLogicPMCB.h"
#include "DefaultPatternMatchCB.h"
#include "Instantiator.h"
#include "PatternMatchCallback.h"


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
 * PLN specific PatternMatchCallback implementation
 */
class PLNImplicator:
	public virtual Implicator,
	public virtual AttentionalFocusCB
{
	public:
		PLNImplicator(AtomSpace* asp) : 
			Implicator(asp), DefaultPatternMatchCB(asp), AttentionalFocusCB(asp) {}
};

}; // namespace opencog

#endif // _OPENCOG_IMPLICATOR_H
