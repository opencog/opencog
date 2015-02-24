/*
 * opencog/util/lazy_random_selector.cc
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

#include "lazy_random_selector.h"

namespace opencog
{

lazy_random_selector::lazy_random_selector(unsigned int u,
                                           opencog::RandGen& _rng)
    : lazy_selector(u), rng(_rng) {}

lazy_random_selector::lazy_random_selector(unsigned int u, unsigned int l,
                                           opencog::RandGen& _rng)
    : lazy_selector(u, l), rng(_rng) {}

unsigned int lazy_random_selector::select()
{
    return rng.randint(_u);
}

} //~namespace opencog
