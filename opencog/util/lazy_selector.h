/*
 * opencog/util/lazy_selector.h
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

#ifndef _OPENCOG_LAZY_SELECTOR_H
#define _OPENCOG_LAZY_SELECTOR_H

#include "hash_map.h"

namespace opencog
{


/**
 * That class allows to select integers in [0,n) but never select twice the same
 * When the operator is called more than n times an assertion is raised
 */
class lazy_selector
{
public:
    lazy_selector(int n) : _n(n), _v(-1) { }
    virtual ~lazy_selector() {}
    bool empty() const {
        return _n == 0;
    }
    int operator()();
protected:
    virtual int select() = 0;
    int _n;
    hash_map<int, int> _map;
    int _v;
};

} //~namespace opencog

#endif
