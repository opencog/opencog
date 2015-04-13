/*
 * opencog/embodiment/Learning/FitnessEstimator/DistortedComboSize.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef DISTORTEDCOMBOSIZE_H
#define DISTORTEDCOMBOSIZE_H

#include <opencog/comboreduct/combo/vertex.h>

namespace FitnessEstimator
{

using namespace opencog::combo;

typedef combo_tree::iterator pre_it;

//if DistortedComboSize(tr1)!=size(tr2) then tr1 < tr2
//otherwise lexicographic_subtree_order

struct DistortedComboSizeOrder : opencog::lexicographic_subtree_order<vertex> {
    //constructor, destructor
    DistortedComboSizeOrder(const std::set<definite_object>& dos)
            : _dos(dos) {
    }
    ~DistortedComboSizeOrder() {}
    //operator
    bool operator()(const combo_tree& tr1, const combo_tree& tr2) const;
private:
    const std::set<definite_object>& _dos;
};

struct DistortedComboSize {
    //return the size of the tree but distorted
    //for instance random_object has actually size
    //#definite_objects * RANDOM_DISTOR_FACTOR instead of 1
    static int size(const combo_tree& tr,
                    const std::set<definite_object>& definite_object_set);

    static int vertex_size(const vertex& v,
                           const std::set<definite_object>& definite_object_set);

};
}

#endif
