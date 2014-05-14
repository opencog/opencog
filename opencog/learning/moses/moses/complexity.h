/*
 * opencog/learning/moses/moses/complexity.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _COMBO_COMPLEXITY_H
#define _COMBO_COMPLEXITY_H

// Various routines dealing with computing the (algorithmic) complexity
// of particular combo programs.

#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace moses {

    // Right now, the algorithmic complexity of any combo program
    // is always an (unsigned) int.  I guess it could be made a float,
    // if need be... there's no fundamental reason for an int here.
    typedef unsigned complexity_t;

    static const complexity_t least_complexity = 0;

    complexity_t tree_complexity(combo::combo_tree::iterator, 
                        bool (*)(const combo::combo_tree::iterator&) = NULL);

    complexity_t tree_complexity(const combo::combo_tree&,
                        bool (*)(const combo::combo_tree::iterator&) = NULL);

} //~namespace moses
} //~namespace opencog

#endif
