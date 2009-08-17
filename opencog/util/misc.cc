/*
 * src/util/misc.h
*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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

#include "misc.h"

using namespace opencog;

// TODO: Review this method for both 32/64-bit processor compatibility
unsigned int opencog::bitcount(unsigned long n)
{
    /* works for 32-bit numbers only    */
    /* fix last line for 64-bit numbers */
    register unsigned long tmp;

    tmp = n - ((n >> 1) & 033333333333)
          - ((n >> 2) & 011111111111);
    return ((tmp + (tmp >> 3)) & 030707070707) % 63;
}

static void internalMakeVirtualAtom(tree<Vertex>* ret, tree<Vertex>::iterator& head, va_list args)
{
    for (void* vp = va_arg(args, void*); vp != NULL; vp = va_arg(args, void*)) {
        const tree<Vertex>* t = ( const tree<Vertex>* ) vp;
        ret->replace(ret->append_child(head), t->begin());
        delete t;
    }
}

tree<Vertex>* opencog::makeVirtualAtom(Handle h, ...)
{
    tree<Vertex>* ret = new tree<Vertex>();
    ret->set_head(Vertex(h));
    tree<Vertex>::iterator head = ret->begin();

    va_list args;
    va_start(args, h);
    internalMakeVirtualAtom(ret, head, args);
    va_end(args);

    return ret; 
}

tree<Vertex>* opencog::makeVirtualAtom(Vertex vertex, ...)
{
    tree<Vertex>* ret = new tree<Vertex>();
    ret->set_head(vertex);
    tree<Vertex>::iterator head = ret->begin();

    va_list args;
    va_start(args, vertex);
    internalMakeVirtualAtom(ret, head, args);
    va_end(args);

    return ret; 
}

#ifndef WIN32
std::string opencog::demangle(const std::string& mangled)
{
    int status = 0;
    char* demangled_name = abi::__cxa_demangle(mangled.c_str(), 0, 0, &status);
    if (status == 0 && demangled_name) {
        std::string s(demangled_name);
        return demangled_name;
    } else return "";
}
#endif

