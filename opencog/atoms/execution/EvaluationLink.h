/*
 * opencog/atoms/execution/EvaluationLink.h
 *
 * Copyright (C) 2013,2014,2015 Linas Vepstas
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

#ifndef _OPENCOG_EVALUTATION_LINK_H
#define _OPENCOG_EVALUTATION_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/execution/FreeLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class EvaluationLink : public FreeLink
{
public:
    EvaluationLink(const HandleSeq& oset,
         TruthValuePtr tv = TruthValue::NULL_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

    EvaluationLink(Handle schema, Handle args,
         TruthValuePtr tv = TruthValue::NULL_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

    EvaluationLink(Link& l);

    TruthValuePtr evaluate(AtomSpace* as) {
        return do_evaluate(as, Handle(shared_from_this()));
    }

    static TruthValuePtr do_evaluate(AtomSpace*, Handle);
    static TruthValuePtr do_evaluate(AtomSpace*, const HandleSeq& schema_and_args);
    static TruthValuePtr do_evaluate(AtomSpace*, Handle schema, Handle args);
};

typedef std::shared_ptr<EvaluationLink> EvaluationLinkPtr;
static inline EvaluationLinkPtr EvaluationLinkCast(const Handle& h)
   { AtomPtr a(h); return std::dynamic_pointer_cast<EvaluationLink>(a); }
static inline EvaluationLinkPtr EvaluationLinkCast(AtomPtr a)
   { return std::dynamic_pointer_cast<EvaluationLink>(a); }

// XXX temporary hack ...
#define createEvaluationLink std::make_shared<EvaluationLink>

/** @}*/
}

#endif // _OPENCOG_EVALUTATION_LINK_H
