/*
 * DefaultImplicator.h
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

#ifndef _OPENCOG_DEFAULT_IMPLICATOR_H
#define _OPENCOG_DEFAULT_IMPLICATOR_H

#include "AttentionalFocusCB.h"
#include "CrispLogicPMCB.h"
#include "DefaultPatternMatchCB.h"
#include "Implicator.h"
#include "PatternMatchCallback.h"


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

#endif // _OPENCOG_DEFAULT_IMPLICATOR_H
