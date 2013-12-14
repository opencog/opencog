/*
 * opencog/atomspace/ExectuionLink.h
 *
 * Copyright (C) 2013 Linas Vepstas
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

#ifndef _OPENCOG_EXECUTION_LINK_H
#define _OPENCOG_EXECUTION_LINK_H

#include <opencog/atomspace/Link.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class ExecutionLink : public Link
{
public:
    ExecutionLink(const HandleSeq& oset,
         TruthValuePtr tv = TruthValue::NULL_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

    ExecutionLink(Handle schema, Handle args,
         TruthValuePtr tv = TruthValue::NULL_TV(),
         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

    Handle execute() { return do_execute(Handle(shared_from_this())); }

    static Handle do_execute(Handle);
    static Handle do_execute(const HandleSeq& schema_and_args);
    static Handle do_execute(Handle schema, Handle args);
};

/** @}*/
}

#endif // _OPENCOG_EXECUTION_LINK_H
