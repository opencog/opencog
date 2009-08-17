/*
 * src/util/misc-test.cc
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

#include "misc-test.h"

using namespace opencog;

/*
 * These functions appear to be used only by four test cases, and 
 * no-where else.
 */
static Handle addAtomIter(AtomSpace& as, tree<Vertex>& a, tree<Vertex>::iterator it, const TruthValue& tvn)
{
    Handle* head_handle_ptr = boost::get<Handle>(&(*it));
    Type* head_type_ptr = boost::get<Type>(&(*it));
    OC_ASSERT((head_handle_ptr != NULL) ^ (head_type_ptr != NULL), "addAtom(): Vertex should be of 'Handle' or 'Type' type.");

    HandleSeq handles;

    if (head_handle_ptr != NULL) {
        return as.addRealAtom(*(TLB::getAtom(*head_handle_ptr)), tvn);
    }

    for (tree<Vertex>::sibling_iterator i = a.begin(it); i != a.end(it); i++) {
        Handle *h_ptr = boost::get<Handle>(&*i);

        if (h_ptr) {
            handles.push_back(as.addRealAtom(*TLB::getAtom(*h_ptr), TruthValue::NULL_TV()));
        } else {
            handles.push_back(addAtomIter(as, a, i, TruthValue::TRIVIAL_TV()));
        }
    }

    return as.addLink(*head_type_ptr, handles, tvn);
}

Handle opencog::addAtom(AtomSpace& as, tree<Vertex>& a, const TruthValue& tvn)
{
    return addAtomIter(as, a, a.begin(), tvn);
}

