/*
 * misc-test.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008-2010 OpenCog Foundation
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

#ifndef _OPENCOG_MISC_TEST_H
#define _OPENCOG_MISC_TEST_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/util/misc.h>
#include <opencog/util/oc_assert.h>

#include "makeVirtualAtom.h"

namespace opencog
{

Handle addRealAtom(AtomSpace& as, const Atom& atom,
                   const TruthValue& tvn = TruthValue::NULL_TV())
{
    const TruthValue& newTV = (tvn.isNullTv()) ? atom.getTruthValue() : tvn;
    // Check if the given Atom reference is of an atom
    // that was not inserted yet.  If so, adds the atom. Otherwise, just sets
    // result to the correct/valid handle.
   
    Handle result;
    const Node *node = dynamic_cast<const Node *>(&atom);
    if (node) {
        result = as.getHandle(node->getType(), node->getName());
        if (result == Handle::UNDEFINED) {
            return as.addNode(node->getType(), node->getName(), newTV);
        }
    } else {
        const Link *link = dynamic_cast<const Link *>(&atom);
        result = as.getHandle(link->getType(), link->getOutgoingSet());
        if (result == Handle::UNDEFINED) {
            return as.addLink(link->getType(), link->getOutgoingSet(), newTV);
        }
    }
    const TruthValuePtr currentTV = as.getTV(result);
    if (currentTV->isNullTv()) {
        as.setTV(result, newTV);
    } else {
        TruthValue* mergedTV = currentTV->merge(newTV);
        as.setTV(result, *mergedTV);
        delete mergedTV;
    }
    return result;
}




/*
 * These functions appear to be used only by four test cases, and 
 * no-where else.  
 */
static inline Handle addAtomIter(AtomSpace& as, tree<Vertex>& a, tree<Vertex>::iterator it, const TruthValue& tvn)
{
    Handle* head_handle_ptr = boost::get<Handle>(&(*it));
    Type* head_type_ptr = boost::get<Type>(&(*it));
    OC_ASSERT((head_handle_ptr != NULL) ^ (head_type_ptr != NULL), "addAtom(): Vertex should be of 'Handle' or 'Type' type.");

    HandleSeq handles;

    if (head_handle_ptr != NULL) {
        OC_ASSERT(as.isValidHandle(*head_handle_ptr), "head_handle_ptr is not valid");
        as.do_merge_tv(*head_handle_ptr, tvn);
        return *head_handle_ptr;
    }

    for (tree<Vertex>::sibling_iterator i = a.begin(it); i != a.end(it); i++) {
        Handle *h_ptr = boost::get<Handle>(&*i);

        if (h_ptr) {
            handles.push_back(addRealAtom(as, *as.cloneAtom(*h_ptr), TruthValue::NULL_TV()));
        } else {
            handles.push_back(addAtomIter(as, a, i, TruthValue::TRIVIAL_TV()));
        }
    }

    return as.addLink(*head_type_ptr, handles, tvn);
}

static inline Handle addAtom(AtomSpace& as, tree<Vertex>& a, const TruthValue& tvn)
{
    return addAtomIter(as, a, a.begin(), tvn);
}


} // namespace opencog

#endif // _OPENCOG_MISC_TEST_H
