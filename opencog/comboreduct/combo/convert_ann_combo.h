/** converter_ann_combo_tree.h --- 
 *
 * Copyright (C) 2010-2011 OpenCog Foundation
 *
 * Author: Joel Lehman
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


#ifndef _COMBO_CONVERTER_ANN_COMBO_TREE_H
#define _COMBO_CONVERTER_ANN_COMBO_TREE_H

#include "simple_nn.h"
#include "vertex.h"

namespace opencog { namespace combo {

/**
 * converter combo_tree <-> ann
 */
struct tree_transform {

    tree_transform() {}

    combo_tree encode_node(ann& the_ann, ann_node* node) const {
        int tag = node->tag;
        
        ann_id id;
        
        if(node->nodetype == nodetype_input)
            id=id::ann_input;
        else
            id=id::ann_node;

        combo_tree tr(ann_type(tag, id));
        
        //only expand a node once
        //that is, one instance of the node
        //will have all of its connections
        //the rest will just be pointers
        if(node->visited)
        {
            return tr;
        }

        //we are expanding the node
        //so, first set the flag so we
        //don't expand it again
        node->visited=true;

        //now handle input nodes, which are easier
        if(node->nodetype == nodetype_input)
        {
            //if its not a memory input, then we are done
            if(!node->memory_ptr)
                return tr;

            //cout << "HANDLING A MEMORY NODE..." << endl;
            //if it is a memory input, then we add the pointer
            //to the memory node (which we do not want to expand)
            bool been_visited = node->memory_ptr->visited;
            node->memory_ptr->visited=true;

            combo_tree encoded_tr = encode_node(the_ann, node->memory_ptr);

            tr.insert_subtree(tr.begin().begin(), encoded_tr.begin());
            node->memory_ptr->visited=been_visited;

            return tr;
        }

        //now handle hidden nodes, which require a little more finesse
        //because now we must handle their connections
        for(ann_connection_it cons = node->in_connections.begin();
            cons != node->in_connections.end(); ++cons)
        {
            combo_tree tmp = encode_node(the_ann, (*cons)->source);

            tr.insert_subtree(tr.begin().begin(), tmp.begin());
            tr.insert_after(tr.begin().last_child(),(*cons)->weight);
        }
        return tr;
    }

    combo_tree encode_ann(ann& the_ann) const {
        //head node is ann
        combo_tree tr(ann_type(0, id::ann)); 

        the_ann.reset_visited();

        //now we want to add each of the output nodes
        //to the tree
        for(ann_node_it node_it = the_ann.outputs.begin(); 
            node_it != the_ann.outputs.end(); ++node_it)
        {
            combo_tree str = encode_node(the_ann, *node_it);
            tr.insert_subtree(tr.begin().begin(),str.begin());
        }
        return tr;
    }

    ann decodify_tree(const combo_tree& tr) const {
        OC_ASSERT(!tr.empty());

        ann new_ann;

        sib_it it = tr.begin();

        OC_ASSERT(get_ann_type(*it).id == id::ann);

        for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
            //add all of the output nodes
            OC_ASSERT(get_ann_type(*sib).id == id::ann_node);

            ann_node* newnode = new ann_node(nodetype_output, 
                                             get_ann_type(*sib).idx);
            new_ann.add_node(newnode);

            decodify_subtree(new_ann, newnode, sib);
        }
        return new_ann;
    }

    ann_node* decodify_node(ann& nn, sib_it sib) const {
         ann_nodetype type;
         
         ann_node* node = nn.find_tag(get_ann_type(*sib).idx);
         
         if (get_ann_type(*sib).id == id::ann_node)
             type = nodetype_hidden;
         else
             type = nodetype_input;

         if (node == NULL) {
             int id = get_ann_type(*sib).idx;
             node = new ann_node(type, id);   
             nn.add_node(node);
         }

          //check to see if input node has a child..   
          if(type==nodetype_input) 
          {
                 ann_node* mem_ptr;
                 //if it has one child, then this is the hidden
                 //node that will supply this input node after a
                 //time delay of 1
                 if (sib.has_one_child())
                 {
                     //cout << "DECODIFYING MEMORY NODE..." << endl;
                     //cout << *sib.begin() << endl;
                     mem_ptr = decodify_node(nn,sib.begin());
                     //cout << "DECODIFYING COMPLETE..." << endl; 

                    node->memory_ptr = mem_ptr;
                }
         }
        
         return node;
    }

    void decodify_subtree(ann& nn, ann_node* dest_node, sib_it it) const {
        std::vector<ann_node*> sources;
        sib_it sib;
        
        int count = -1;
        
        for (sib = it.begin(); sib != it.end(); ++sib) {
            //cout << "looking at node " << *sib << endl;
            if (!is_ann_type(*sib))
                break;
            
            //see if this node has already been referenced & created
            ann_node* node = decodify_node(nn, sib);
            sources.push_back(node);

            //recurse on hidden nodes
            if (get_ann_type(*sib).id == id::ann_node)
                decodify_subtree(nn, node, sib);
            count++;
        }

        //now add weights
        for ( ; sib != it.end(); ++sib) {
            //cout << "adding weight " << *sib << endl;
            nn.add_connection(sources[count], dest_node, get_contin(*sib));
            count--;
        }
    }
};

}} // ~namespaces combo opencog

#endif // _COMBO_CONVERTER_ANN_COMBO_TREE_H
