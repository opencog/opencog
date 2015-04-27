/*
 * opencog/util/lazy_normal_selector.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
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

#ifndef _OPENCOG_LAZY_NORMAL_SELECTOR_H
#define _OPENCOG_LAZY_NORMAL_SELECTOR_H

#include <opencog/util/lazy_selector.h>
#include <opencog/util/oc_assert.h>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

//! apply lazy_selector but the select method returns always the same
//! number, created for testing
struct lazy_normal_selector : public lazy_selector {
    lazy_normal_selector(unsigned int n, unsigned int s = 0) :
        lazy_selector(n), _s(s) {
        OC_ASSERT(s < n);
    }
protected:
    unsigned int select();
private:
    unsigned int _s;
};

/** @}*/
} //~namespace opencog

#endif
