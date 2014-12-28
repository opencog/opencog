/*
 * opencog/embodiment/AtomSpaceExtensions/CompareAtomTreeTemplate.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller, Ari A. Heljakka
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

#include "CompareAtomTreeTemplate.h"
#include <opencog/util/oc_assert.h>

//------------------------
//is_atom_tree_template_of
//------------------------

bool is_atom_tree_template_of::operator()(const atom_tree& tr1,
        const atom_tree& tr2,
        const opencog::AtomSpace* as, bool ec) const
{
    return is_atom_tree_template_it(tr1, tr1.begin(),
                                    tr2, tr2.begin(), as, ec);
}

bool is_atom_tree_template_of::is_atom_tree_template_it(const atom_tree& tr1,
        atom_tree_it it1,
        const atom_tree& tr2,
        atom_tree_it it2,
        const opencog::AtomSpace* as,
        bool ec) const
{
    OC_ASSERT(tr1.is_valid(it1), "atom_tree tr1 isn't a valid one.");
    OC_ASSERT(tr2.is_valid(it2), "atom_tree tr2 isn't a valid one.");
    Vertex v1 = *it1;
    Vertex v2 = *it2;

    Handle* h1_ptr = boost::get<Handle>(&v1);
    Type* t1_ptr = boost::get<Type>(&v1);
    Handle* h2_ptr = boost::get<Handle>(&v2);
    Type* t2_ptr = boost::get<Type>(&v2);

    //assert that the vertices are Handle
    OC_ASSERT((h1_ptr != NULL) ^ (t1_ptr != NULL), "atom_vertex v1 should be 'Handle' or 'Type' typed.");
    OC_ASSERT((h2_ptr != NULL) ^ (t2_ptr != NULL), "atom_vertex v2 should be 'Handle' or 'Type' typed.");

    //debug print
    //std::cout << "XEQUAL H1 : " << ((h1_ptr != NULL)?as->atomToString(*h1_ptr):"TYPE")
    //  << " : " << ((h1_ptr != NULL)?as->getType(*h1_ptr):*t1_ptr) << std::endl;
    //std::cout << "XEQUAL H2 : " << ((h2_ptr != NULL)?as->atomToString(*h2_ptr):"TYPE")
    //  << " : " << ((h2_ptr != NULL)?as->getType(*h2_ptr):*t2_ptr) << std::endl;
    //~debug print

    //check if h1 == h2 if real or inherits otherwise
    bool h1_TempOf_h2;
    //if h1 is not a type
    if (h1_ptr != NULL) {
        //if h2 is not a type
        if (h2_ptr != NULL)
            h1_TempOf_h2 = *h1_ptr == *h2_ptr;
        //h2 is a type
        else h1_TempOf_h2 = false; //because tr1 is the template and tr2 a instance
    } else {
        h1_TempOf_h2 = classserver().isA(
                (h2_ptr != NULL) ?
                    as->getType(*h2_ptr) : *t2_ptr,
                *t1_ptr);
    }

    //debug print
    //std::cout << "XEQUAL H1_TEMPOF_H2 : " << h1_TempOf_h2 << std::endl;
    //~debug print

    if (h1_TempOf_h2) {
        //check the number of children
        bool correct_nc;
        unsigned n1 = tr1.number_of_children(it1);
        unsigned n2 = tr2.number_of_children(it2);
        if (ec)
            correct_nc = n1 == n2;
        else correct_nc = n1 <= n2;
        if (correct_nc) {
            //check the children recursively
            atom_tree::sibling_iterator sib1 = it1.begin();
            atom_tree::sibling_iterator sib2 = it2.begin();
            for (; sib1 != it2.end(); ++sib1, ++sib2) {
                bool temp = is_atom_tree_template_it(tr1, atom_tree_it(sib1),
                                                     tr2, atom_tree_it(sib2),
                                                     as, ec);
                if (!temp)
                    return false;
            }
            return true;
        } else return false;
    } else return false;
}


//-------------
//fits_template
//-------------

does_fit_template::does_fit_template(const atom_tree& tr, const opencog::AtomSpace* as,
                                     bool ec) : _tr(tr),
        _as(as),
        _enable_childcount(ec) {}

bool does_fit_template::operator()(opencog::Handle h) const
{
    atom_tree h_tr(make_atom_tree(h, _as, false));
    bool res = is_atom_tree_template_of()(_tr, h_tr, _as, _enable_childcount);
    return res;
}


void does_fit_template::makeHandletree(opencog::Handle real, bool fullVirtual,
                                       atom_tree& ret,
                                       const opencog::AtomSpace* as) const
{
    ret.set_head(real);
    expandHandletree(fullVirtual, ret, ret.begin(), as);
}

void does_fit_template::expandHandletree(bool fullVirtual, atom_tree& ret,
        atom_tree_it ret_top,
        const opencog::AtomSpace* as) const
{
    Handle *h_ptr = boost::get<Handle>(&(*ret_top));
    Type T = (h_ptr != NULL) ? as->getType(*h_ptr) : boost::get<Type>(*ret_top);

    /// If link then we keep expanding
    if (classserver().isA(T, LINK)) {
        if (fullVirtual)
            *ret_top = Vertex(T);

        if (h_ptr != NULL) {
            HandleSeq _hs = as->getOutgoing(*h_ptr);
            for (HandleSeq::iterator child_h = _hs.begin(); child_h != _hs.end(); ++child_h) {
                //foreach(Handle child_h, _hs) {
                atom_tree_it next_i = ret.append_child(ret_top, *child_h);
                expandHandletree(fullVirtual, ret, next_i, as);
            }
        }
    }
}

atom_tree does_fit_template::make_atom_tree(opencog::Handle h, const opencog::AtomSpace* as,
        bool fullVirtual) const
{
    atom_tree ret;
    makeHandletree(h, fullVirtual, ret, as);

    return ret;
}
