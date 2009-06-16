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

#include "simple_nn.h"

using namespace combo;
using namespace std;

#define MIN_FITNESS -1.0e10

typedef combo_tree::sibling_iterator sib_it;

struct tree_transform
{

 tree_transform() { }

ann* decodify_tree(combo_tree tr) const
{
	ann* new_ann = new ann();

	sib_it head = tr.begin();
	if(*head != id::ann)
	{
		cout << "root node should be ann" << endl;	
		cout << *head << endl;
	}
	for(sib_it sib = head.begin(); sib != head.end(); ++sib)
	{
		//add all of the output nodes
		if(*sib != id::ann_node)
		{
			cout << "child of ann should be output nodes" << endl;
			cout << *sib << endl;
		}
		ann_node* newnode = new ann_node(nodetype_output);
		new_ann->add_node(newnode);
		decodify_subtree(new_ann,newnode,sib);
	}
	return new_ann;
}

void decodify_subtree(ann* nn, ann_node* dest_node, sib_it it) const
{
	vector<ann_node*> sources;
	sib_it sib;
	for(sib = it.begin(); sib!=it.end(); ++sib)
	{
		ann_nodetype type;
		if(*sib != id::ann_node && *sib != id::ann_input)
			break;	
		ann_node* node = NULL; //nn->find_tag((void*)*sib);

		if(*sib == id::ann_node)
			type=nodetype_hidden;
		else
			type=nodetype_input;

		if(node==NULL)
			node = new ann_node(type);

		sources.push_back(node);

		//recurse on hidden nodes		
		if(*sib == id::ann_node)
			decodify_subtree(nn,node,sib);
	}

	//now add weights
	int count=0;
	for( ; sib!=it.end(); ++sib)
	{
		nn->add_connection(sources[count],dest_node,
				boost::get<combo::contin_t>(*sib));
		count++;
	}
}
};

struct AnnFitnessFunction : unary_function<combo_tree, double> {

    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    result_type operator()(argument_type tr) const {
        if (tr.empty())
            return MIN_FITNESS;
        int sc = 0;
        return (double)sc;
    }

};

#endif
