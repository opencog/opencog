/*
 * opencog/util/lazy_random_selector.h
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

#ifndef _OPENCOG_LAZY_RANDOM_SELECTOR_H
#define _OPENCOG_LAZY_RANDOM_SELECTOR_H

#include <opencog/util/lazy_selector.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/mt19937ar.h>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

//! a lazy random selector without replacement
/**
 * lets you select m random integers in [l,u) without replacement
 * (i.e. it never selects twice the same number)
 * each in O(1) and only uses O(m) memory - useful where u-l is much larger than m
 */
struct lazy_random_selector : public lazy_selector {
    lazy_random_selector(unsigned int u,
                         opencog::RandGen& _rng = randGen());
    lazy_random_selector(unsigned int u, unsigned int l,
                         opencog::RandGen& _rng = randGen());
protected:
    unsigned int select();
private:
    opencog::RandGen& rng;
};

/** @}*/
} //~namespace opencog

#endif
