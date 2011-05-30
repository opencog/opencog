/*
 * src/AtomSpace/utils.h
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

#ifndef OPENCOG_MVA_H
#define OPENCOG_MVA_H

#include <opencog/util/tree.h>
#include <opencog/atomspace/types.h>

namespace opencog
{

typedef tree<Vertex> vtree; // Vertex defined in atomspace/types.h


#define mva MakeVirtualAtom_slow

/// Handles are actually mostly types, here. The Handle/Type ambiguity
/// will be resolved soon enough. (says Ari, March 20, 2006)

tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4, const tree<Vertex>& t5);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3, const tree<Vertex>& t4);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2, const tree<Vertex>& t3);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1, const tree<Vertex>& t2);
tree<Vertex> MakeVirtualAtom_slow(Vertex v, const tree<Vertex>& t1);
tree<Vertex> MakeVirtualAtom_slow(Vertex v);


/// Convert a real atom into vtree in which only NODEs are left as real atoms
/// while all links become virtual, ie. tree branches

struct less_tree_vertex : public std::binary_function<tree<Vertex>, tree<Vertex>, bool> {
    bool operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const;
    bool operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs,
                    tree<Vertex>::iterator ltop,
                    tree<Vertex>::iterator rtop) const;
};

struct less_vtree : public std::binary_function<tree<Vertex>, tree<Vertex>, bool> {
    bool operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const;
};

} // namespace opencog

#endif /* OPENCOG_UTILS_H */
