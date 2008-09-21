/*
 * src/AtomSpace/utils2.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#include "utils2.h"

using namespace opencog;

string opencog::i2str(int d)
{
    char temp[20];
    sprintf(temp, "%d", d);
    return temp;
}

bool opencog::visible(char c)
{
    return c != ' ' && c != '\r' && c != '\n' && c != '\t' && c != 0;
}

std::string opencog::XMLembed(const std::string &elem, const std::string &pcdata)
{
    return std::string("<")
           + elem
           + std::string("> ")
           + std::string(pcdata)
           + std::string(" </")
           + std::string(elem)
           + std::string(">");
};

//this typedef is put is the .cc not the .h because that provokes a vtree clash
//with combo::vtree
typedef tree<Vertex> vtree;
typedef vtree::iterator pre_it;

vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4, const vtree& t5)
{
    //printf("MakeVirtualAtom_slow Handle, vtree, vtree, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex((Handle)T));
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
        ret.set_head(Vertex((Handle)T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());
        ret.replace(ret.append_child(head_it), t3.begin());
        ret.replace(ret.append_child(head_it), t4.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin); return tree<Vertex>();
    }
}
vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3)
{
    //printf("MakeVirtualAtom_slow Handle, vtree, vtree, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex((Handle)T));
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
        ret.set_head(Vertex((Handle)T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());
        ret.replace(ret.append_child(head_it), t2.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
    }
}

vtree opencog::MakeVirtualAtom_slow(Handle T, const vtree& t1)
{
    //printf("MakeVirtualAtom_slow Handle, vtree\n");
    try {
        vtree ret;
        ret.set_head(Vertex((Handle)T));
        pre_it head_it = ret.begin();
        ret.replace(ret.append_child(head_it), t1.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
    }
}

vtree opencog::MakeVirtualAtom_slow(Handle T)
{
    //printf("MakeVirtualAtom_slow Handle\n");
    try {
        vtree ret;
        ret.set_head(Vertex((Handle)T));

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
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
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
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
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
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
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
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
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
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
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
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
        puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>();
    }
}
