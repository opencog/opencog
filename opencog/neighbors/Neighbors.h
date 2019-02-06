/*
 * Neighbors.h
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

#ifndef _OPENCOG_NEIGHBORS_H
#define _OPENCOG_NEIGHBORS_H

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/atom_types/types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Returns neighboring atoms, following incoming links and
 * returning their outgoing sets.
 *
 * @param h Get neighbours for the atom this handle points to.
 * @param linkType Follow only these types of links.
 */
HandleSeq get_target_neighbors(const Handle& h, Type desiredLinkType,
                               bool match_subtype = false);
HandleSeq get_source_neighbors(const Handle& h, Type desiredLinkType,
                               bool match_subtype = false);
HandleSeq get_all_neighbors(const Handle& h, Type desiredLinkType);


/**
 * Return all atoms connected to h up to a given distance. Both
 * incomings and outgoings are considered (unlike getNeighbors).
 *
 * @param h     the center atom
 * @param dist  the maximum distance, or none if negative
 * @return      an UnorderedHandleSet of neighbors
 *
 * XXX FIXME -- this function is curently not used anywhere. Perhaps
 * it should be deleted?
 */
UnorderedHandleSet get_distant_neighbors(const Handle& h, int dist = 1);


/** @}*/
}


#endif // _OPENCOG_NEIGHBORS_H
