/** makeVirtualAtom.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_MAKEVIRTUALATOM_H
#define _OPENCOG_MAKEVIRTUALATOM_H

#include <boost/variant.hpp>

#include <opencog/atomspace/types.h>
#include <opencog/util/tree.h>

namespace opencog
{

typedef boost::variant<Handle, Type, int, unsigned int, float, bool,
                       unsigned char, char, short int> Vertex;

typedef std::vector<Vertex> VertexSeq;
typedef tree<Vertex> atom_tree;
typedef atom_tree::iterator atom_tree_it;

/// @todo the following constructors are redundant with the ones
/// defined in opencog/learning/pln/utils/mva.h
atom_tree* makeVirtualAtom(Handle h, ...);
atom_tree* makeVirtualAtom(Vertex vertex, ...);

} // ~namespace opencog

#endif // _OPENCOG_MAKEVIRTUALATOM_H
