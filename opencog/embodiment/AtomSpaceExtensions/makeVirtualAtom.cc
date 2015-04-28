/** makeVirtualAtom.cc --- 
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

#include "makeVirtualAtom.h"

namespace opencog
{

static void internalMakeVirtualAtom(atom_tree* ret, atom_tree_it& head, va_list args)
{
    for (void* vp = va_arg(args, void*); vp != NULL; vp = va_arg(args, void*)) {
        const atom_tree* t = ( const atom_tree* ) vp;
        ret->replace(ret->append_child(head), t->begin());
        delete t;
    }
}

atom_tree* makeVirtualAtom(Handle h, ...)
{
    atom_tree* ret = new atom_tree();
    ret->set_head(Vertex(h));
    atom_tree_it head = ret->begin();

    va_list args;
    va_start(args, h);
    internalMakeVirtualAtom(ret, head, args);
    va_end(args);

    return ret; 
}

atom_tree* makeVirtualAtom(Vertex vertex, ...)
{
    atom_tree* ret = new atom_tree();
    ret->set_head(vertex);
    atom_tree_it head = ret->begin();

    va_list args;
    va_start(args, vertex);
    internalMakeVirtualAtom(ret, head, args);
    va_end(args);

    return ret; 
}

} // ~namespace opencog
