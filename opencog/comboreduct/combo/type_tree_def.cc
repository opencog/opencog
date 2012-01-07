/*
 * opencog/comboreduct/combo/type_tree_def.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#include "type_tree_def.h"
#include <opencog/util/oc_assert.h>

namespace opencog { namespace combo {

bool is_argument_type(type_node n)
{
    return ((int)n) >= id::argument_type;
}

unsigned int arg_to_idx(type_node n)
{
    OC_ASSERT(is_argument_type(n),
              "Cannot find the idx of a non-argument type");
    return (unsigned int)((int)n-(int)id::argument_type+1);
}

}} // ~namespaces combo opencog
