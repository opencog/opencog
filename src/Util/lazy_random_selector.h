/*
 * src/Util/lazy_random_selector.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#include "platform.h"
#include "RandGen.h"

namespace opencog
{

//a lazy random selector without replacement -
//lets you select m random integers in [0,n) without replacement each in O(1)
//and only uses O(m) memory - useful where n is much larger than m
struct lazy_random_selector {
    lazy_random_selector(int n, opencog::RandGen& _rng) : _n(n), _v(-1), rng(_rng) { }
    bool empty() const {
        return (_n == 0);
    }
    int operator()();
private:
    int _n;
    hash_map<int, int> _map;
    int _v;
    opencog::RandGen& rng;
};

} //~namespace opencog

#endif
