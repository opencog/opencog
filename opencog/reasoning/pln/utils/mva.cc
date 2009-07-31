/*
 * src/AtomSpace/utils.cc
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

#include <stdlib.h>
#include <stdarg.h>
#include <string>

#include "mva.h"
#include <opencog/util/platform.h>

using namespace opencog;

namespace opencog {

vtree MakeVirtualAtom_slow(Vertex v, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4)
{
    vtree ret;
	try
	{
		ret.set_head(v);
		ret.append_child(ret.begin(), t1.begin());
		ret.append_child(ret.begin(), t2.begin());
		ret.append_child(ret.begin(), t3.begin());
		ret.append_child(ret.begin(), t4.begin());
	} catch(...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
    return ret;
}

vtree MakeVirtualAtom_slow(Vertex v, const vtree& t1, const vtree& t2, const vtree& t3)
{
    vtree ret;
	try {
		ret.set_head(v);
		ret.append_child(ret.begin(), t1.begin());
		ret.append_child(ret.begin(), t2.begin());
		ret.append_child(ret.begin(), t3.begin());
	} catch(...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
    return ret;
}

vtree MakeVirtualAtom_slow(Vertex v, const vtree& t1, const vtree& t2)
{
    vtree ret;
	try {
		ret.set_head(v);
		ret.append_child(ret.begin(), t1.begin());
		ret.append_child(ret.begin(), t2.begin());
	} catch(...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
    return ret;
}

vtree MakeVirtualAtom_slow(Vertex v, const vtree& t1)
{
    vtree ret;
	try {
		ret.set_head(v);
		ret.append_child(ret.begin(), t1.begin());
	} catch(...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
    return ret;
}

vtree MakeVirtualAtom_slow(Vertex v)
{
    vtree ret;
	try {
		ret.set_head(v);
	} catch(...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
    return ret;
}

}

bool less_tree_vertex::operator()(const vtree& lhs, const vtree& rhs) const
{
    return (*this)(lhs, rhs, lhs.begin(), rhs.begin());
}
bool less_tree_vertex::operator()(const vtree& lhs, const vtree& rhs,
                                  vtree::iterator ltop,
                                  vtree::iterator rtop) const
{
    if (*ltop < *rtop)
        return true;
    if (*rtop < *ltop)
        return false;

    if (ltop.number_of_children() < rtop.number_of_children())
        return true;
    if (ltop.number_of_children() > rtop.number_of_children())
        return false;

    vtree::sibling_iterator r_i = rhs.begin(rtop);
    for (vtree::sibling_iterator i = lhs.begin(ltop);
            i != lhs.end(ltop);
            i++) {
        if (less_tree_vertex()(lhs, rhs, i, r_i))
            return true;
        else
            if (less_tree_vertex()(rhs, lhs, r_i, i))
                return false;

        r_i++;
    }

    return false;
}

/* MISC UTILITIES */

bool less_vtree_it( const vtree & lhs_t, const vtree & rhs_t,
                    vtree::sibling_iterator ltop, vtree::sibling_iterator rtop)
{
    if ((*ltop) < (*rtop))
        return true;
    if ((*rtop) < (*ltop))
        return false;

    vtree::sibling_iterator rit = rhs_t.begin(rtop);

    for (vtree::sibling_iterator lit = lhs_t.begin(ltop);
            lit != lhs_t.end  (ltop);
            lit++, rit++)
        if (less_vtree_it(lhs_t, rhs_t, lit, rit))
            return true;
        else if (less_vtree_it(lhs_t, rhs_t, rit, lit))
            return false;

    return false;
}

bool less_vtree::operator()(const vtree& lhs, const vtree& rhs) const
{
    if (lhs.size() < rhs.size())
        return true;
    if (lhs.size() > rhs.size() || lhs.empty())
        return false;

    return less_vtree_it(lhs, rhs, lhs.begin(), rhs.begin());
}

//this typedef is put is the .cc not the .h because that provokes a vtree clash
//with combo::vtree
//typedef vtree vtree;
typedef vtree::iterator pre_it;

#if NOT_USED_ANYMORE_DELETE_ME_WHENEVER

vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4, const vtree& t5)
{
    //printf("MakeVirtualAtom_slow Handle, vtree, vtree, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex(T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());
        ret.replace(ret.append_child(head_it), t3.begin());
        ret.replace(ret.append_child(head_it), t4.begin());
        ret.replace(ret.append_child(head_it), t5.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin); return vtree();
    }
}
vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4)
{
    //printf("MakeVirtualAtom_slow Handle, vtree, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex(T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());
        ret.replace(ret.append_child(head_it), t3.begin());
        ret.replace(ret.append_child(head_it), t4.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin); return vtree();
    }
}
vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3)
{
    //printf("MakeVirtualAtom_slow Handle, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex(T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());
        ret.replace(ret.append_child(head_it), t3.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}

vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2)
{
    //printf("MakeVirtualAtom_slow Handle, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex(T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}

vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1)
{
    //printf("MakeVirtualAtom_slow Handle, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex(T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}

vtree opencog::MakeVirtualAtom_slow(Handle T)
{
    //printf("MakeVirtualAtom_slow Handle\n");
    try {
        vtree ret;
        ret.set_head(Vertex(T));

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}
vtree opencog::MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4, const vtree& t5)
{
    //printf("MakeVirtualAtom_slow Vertex, vtree, vtree, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(T);
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());
        ret.replace(ret.append_child(head_it), t3.begin());
        ret.replace(ret.append_child(head_it), t4.begin());
        ret.replace(ret.append_child(head_it), t5.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}
vtree opencog::MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4)
{
    //printf("MakeVirtualAtom_slow Vertex, vtree, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(T);
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());
        ret.replace(ret.append_child(head_it), t3.begin());
        ret.replace(ret.append_child(head_it), t4.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}
vtree opencog::MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2, const vtree& t3)
{
    //printf("MakeVirtualAtom_slow Vertex, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(T);
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());
        ret.replace(ret.append_child(head_it), t3.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}

vtree opencog::MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2)
{
    //printf("MakeVirtualAtom_slow Vertex, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(T);
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}

vtree opencog::MakeVirtualAtom_slow(Vertex T, const vtree& t1)
{
    //printf("MakeVirtualAtom_slow Vertex, vtree\n");
    try {
        vtree ret;
        ret.set_head(T);
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}

vtree opencog::MakeVirtualAtom_slow(Vertex T)
{
    //printf("MakeVirtualAtom_slow Vertex\n");
    try {
        vtree ret;
        ret.set_head(T);

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree();
    }
}

#endif /* NOT_USED_ANYMORE_DELETE_ME_WHENEVER */

