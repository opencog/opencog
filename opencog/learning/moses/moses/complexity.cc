/*
 * opencog/comboreduct/combo/complexity.cc
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
#include <opencog/util/exceptions.h>

#include "complexity.h"
#include "using.h"

namespace combo {

  // for a Boolean formula, the complexity is the # of literals

  complexity_t complexity(combo_tree::iterator it) {
    if (*it==id::logical_true || *it==id::logical_false || *it==id::null_vertex)
      return 0;

    if (is_argument(*it))
      return -1;

    if (is_builtin_action(*it)) // PJ
      return -1;

    if (*it==id::logical_not)
      return complexity(it.begin());

//    OC_ASSERT(*it==id::logical_and || *it==id::logical_or, 
//            "combo_tree node should be of logical types 'id::logical_and' or 'id::logical_or'.");
// PJ now complexity for action trees can also be computed

    int c=0;
    for (combo_tree::sibling_iterator sib=it.begin();sib!=it.end();++sib)
      c+=complexity(sib);
    return c;
  }

} //~namespace combo
