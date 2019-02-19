/*
 * opencog/attentionbank/bank/AVUtils.h
 *
 * Copyright (C) 2017 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_AVUTILS_H
#define _OPENCOG_AVUTILS_H

#include <opencog/attentionbank/avalue/AttentionValue.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Handy utilities to get the attention value of an atom.
 */
AttentionValuePtr get_av(const Handle&);
void set_av(AtomSpace*, const Handle&, const AttentionValuePtr&);

static inline AttentionValue::sti_t get_sti(const Handle& h)
{
    return get_av(h)->getSTI();
}

static inline AttentionValue::lti_t get_lti(const Handle& h)
{
    return get_av(h)->getLTI();
}

static inline AttentionValue::vlti_t get_vlti(const Handle& h)
{
    return get_av(h)->getVLTI();
}

/** @}*/
} //namespace opencog

#endif // _OPENCOG_AVUTILS_H
