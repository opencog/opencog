/*
 * AtomSpaceUtils.h
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#ifndef _OPENCOG_ATOMSPACE_UTILS_H
#define _OPENCOG_ATOMSPACE_UTILS_H

#include "AtomSpace.h"

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * General purpose utilities for processing atoms in the AtomSpace.
 *
 * Contains methods and algorithms which might be useful to other processes.
 */
class AtomSpaceUtils
{

public:
    static HandleSeq getAllNodes(AtomSpace* pAS, Handle h);

    static UnorderedHandleSet getAllUniqueNodes(AtomSpace* pAS, Handle h);

};

/** @}*/
}

#endif // _OPENCOG_ATOMSPACE_UTILS_H
