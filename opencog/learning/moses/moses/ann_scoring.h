/*
 * opencog/learning/moses/moses/ann_scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Joel Lehman
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
#ifndef _ANN_SCORING_H
#define _ANN_SCORING_H

#include <opencog/util/numeric.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/simple_nn.h>
#include "scoring.h"
#include "scoring_functions.h" 

using namespace combo;
using namespace std;
using namespace moses;
#define MIN_FITNESS -1.0e10

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

struct tree_transform {

    tree_transform() { }

    ann decodify_tree(combo_tree tr) const {
        ann new_ann = ann();

        sib_it it = tr.begin();

        if (get_ann_type(*it).id != id::ann) {
            cout << "root node should be ann" << endl;
        }

        for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
            //add all of the output nodes
            if (get_ann_type(*sib).id != id::ann_node) {
                cout << "child of ann should be output nodes" << endl;
            }

            ann_node* newnode = new ann_node(nodetype_output, get_ann_type(*sib).idx);
            new_ann.add_node(newnode);

            //cout << "looking at subtree..." << endl;
            decodify_subtree(new_ann, newnode, sib);
        }
        return new_ann;
    }

    void decodify_subtree(ann& nn, ann_node* dest_node, sib_it it) const {
        vector<ann_node*> sources;
        sib_it sib;
        
        int count = -1;
        
        for (sib = it.begin(); sib != it.end(); ++sib) {
            ann_nodetype type;
            //cout << "looking at node " << *sib << endl;
            if (!is_ann_type(*sib))
                break;
            //cout << "node is a node" <<endl;
            //see if this node has already been referenced & created
            ann_node* node = nn.find_tag(get_ann_type(*sib).idx);

            if (get_ann_type(*sib).id == id::ann_node)
                type = nodetype_hidden;
            else
                type = nodetype_input;

            if (node == NULL)
            {
                node = new ann_node(type, get_ann_type(*sib).idx);
                nn.add_node(node);
            }
            sources.push_back(node);

            //recurse on hidden nodes
            if (get_ann_type(*sib).id == id::ann_node)
                decodify_subtree(nn, node, sib);
            count++;
        }

        //now add weights
        for ( ; sib != it.end(); ++sib) {
            //cout << "adding weight " << *sib << endl;
            nn.add_connection(sources[count], dest_node,
                              boost::get<combo::contin_t>(*sib));
            count--;
        }
    }
};

struct AnnFitnessFunction : unary_function<combo_tree, double> {

    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    result_type operator()(argument_type tr) const {
        if (tr.empty())
            return MIN_FITNESS;

        tree_transform tt;

        //xor_problem
        double inputs[4][3] = { {0.0, 0.0,1.0}, 
                                {0.0, 1.0,1.0}, 
                                {1.0, 0.0,1.0},
                                {1.0, 1.0,1.0}};
        double outputs[4] = {0.0, 1.0, 1.0, 0.0};

        ann nn = tt.decodify_tree(tr);
        int depth = nn.feedforward_depth();

        double error = 0.0;
        for (int pattern = 0;pattern < 4;pattern++) {
            nn.load_inputs(inputs[pattern]);
            for (int x = 0;x < depth;x++)
                nn.propagate();
            double diff = outputs[pattern] - nn.outputs[0]->activation;
            error += diff * diff;
        }

        return -error;
    }

};

namespace moses
{
struct ann_score {
   ann_score() { }
   double operator()(const combo_tree& tr) const {
       return aff(tr);
   }

   AnnFitnessFunction aff;
};

struct ann_bscore {
    ann_bscore( ) { }

    behavioral_score operator()(const combo_tree& tr) const {
        behavioral_score bs(2);
        bs[0] = -moses::ann_score()(tr);
        bs[1] = tr.size();

        return bs;
    }
};
}
#endif
