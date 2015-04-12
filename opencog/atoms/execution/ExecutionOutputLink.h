/*
 * opencog/atoms/execution/ExecutionOutputLink.h
 *
 * Copyright (C) 2013,2015 Linas Vepstas
 * All Rights Reserved
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

#ifndef _OPENCOG_EXECUTION_OUTPUT_LINK_H
#define _OPENCOG_EXECUTION_OUTPUT_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/execution/FreeLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class ExecutionOutputLink : public FreeLink
{
public:
	ExecutionOutputLink(const HandleSeq& oset,
	     TruthValuePtr tv = TruthValue::NULL_TV(),
	     AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ExecutionOutputLink(const Handle& schema, const Handle& args,
	     TruthValuePtr tv = TruthValue::NULL_TV(),
	     AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ExecutionOutputLink(Link& l);

	Handle execute(AtomSpace* as) {
	    return do_execute(as, Handle(shared_from_this()));
	}

	static Handle do_execute(AtomSpace*, const Handle&);
	static Handle do_execute(AtomSpace*, Type, const HandleSeq& schema_and_args);
	static Handle do_execute(AtomSpace*, const Handle& schema,
	                                     const Handle& args);
};

typedef std::shared_ptr<ExecutionOutputLink> ExecutionOutputLinkPtr;
static inline ExecutionOutputLinkPtr ExecutionOutputLinkCast(const Handle& h)
   { AtomPtr a(h); return std::dynamic_pointer_cast<ExecutionOutputLink>(a); }
static inline ExecutionOutputLinkPtr ExecutionOutputLinkCast(AtomPtr a)
   { return std::dynamic_pointer_cast<ExecutionOutputLink>(a); }

// XXX temporary hack ...
#define createExecutionOutputLink std::make_shared<ExecutionOutputLink>

/** @}*/
}

#endif // _OPENCOG_EXECUTION_OUTPUT_LINK_H
