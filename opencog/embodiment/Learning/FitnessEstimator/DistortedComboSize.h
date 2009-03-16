/**
 * DistortedComboSize.h
 *
 * Author(s):
 *      Nil Geisweiller
 * Created: 05/10/2007
 */
#ifndef DISTORTEDCOMBOSIZE_H
#define DISTORTEDCOMBOSIZE_H

#include "comboreduct/combo/vertex.h"

namespace FitnessEstimator
{

using namespace combo;

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
