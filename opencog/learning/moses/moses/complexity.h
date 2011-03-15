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

//various routines dealing with computin the (algorithmic) complexity of
//particular combo programs

#include <opencog/comboreduct/combo/vertex.h>
#include <limits>

namespace moses {
    typedef int complexity_t ;
  
    static const complexity_t max_complexity =
        std::numeric_limits<complexity_t>::max();

    complexity_t complexity(combo::combo_tree::iterator);
    
    complexity_t complexity(const combo::combo_tree&);
} //~namespace combo

#endif
