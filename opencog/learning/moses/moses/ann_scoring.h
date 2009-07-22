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
#include "pole_balancing.h"

using namespace combo;
using namespace std;
using namespace moses;
#define MIN_FITNESS -1.0e10

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

struct tree_transform {

    tree_transform() { }

    combo_tree encode_node(ann &the_ann,ann_node* node) const {
        
        int tag = node->tag;
        
        ann_id id;
        
        if(node->nodetype == nodetype_input)
            id=id::ann_input;
        else
            id=id::ann_node;
       
        combo_tree tr(ann_type(tag,id));
        
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
            if(!node->memory_node)
                return tr;

            cout << "HANDLING A MEMORY NODE..." << endl;
            //if it is a memory input, then we add the pointer
            //to the memory node (which we do not want to expand)
            bool been_visited = node->memory_ptr->visited;
            node->memory_ptr->visited=true;
            tr.insert_subtree(tr.begin().begin(),
                    encode_node(the_ann,node->memory_ptr).begin());
            node->memory_ptr->visited=been_visited;
            cout << tr << endl;
            cout << "DONE WITH MEMORY NODE.." << endl;
            return tr;
        }

        //now handle hidden nodes, which require a little more finesse
        //because now we must handle their connections
        ann_connection_it cons;
        for(cons = node->in_connections.begin();
                cons != node->in_connections.end();
                cons++)
        {
            tr.insert_subtree(tr.begin().begin(),encode_node(the_ann,
                        (*cons)->source).begin());
            tr.insert_after(tr.begin().last_child(),(*cons)->weight);
        }
        return tr;
    }

    combo_tree encode_ann(ann &the_ann) const {
        //head node is ann
        combo_tree tr(ann_type(0,id::ann)); 

        the_ann.reset_visited();

        //now we want to add each of the output nodes
        //to the tree
        ann_node_it node_it;
        for (node_it = the_ann.outputs.begin(); 
                node_it != the_ann.outputs.end();
                node_it++)
        {
            combo_tree str = encode_node(the_ann,*node_it);
            tr.insert_subtree(tr.begin().begin(),str.begin());
        }
        return tr;
    }

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

                    node->memory_node=true;
                    node->memory_ptr = mem_ptr;
                }
         }
        
         return node;
    }

    void decodify_subtree(ann& nn, ann_node* dest_node, sib_it it) const {
        vector<ann_node*> sources;
        sib_it sib;
        
        int count = -1;
        
        for (sib = it.begin(); sib != it.end(); ++sib) {
            //cout << "looking at node " << *sib << endl;
            if (!is_ann_type(*sib))
                break;
            
            //see if this node has already been referenced & created
            ann_node* node = decodify_node(nn,sib);
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

struct AnnPole2FitnessFunction : unary_function<combo_tree, double> {
 result_type operator()(argument_type tr) const {
    bool velocity = true;
    if (tr.empty())
        return MIN_FITNESS;
    tree_transform tt;
    ann nn = tt.decodify_tree(tr);

    CartPole *the_cart;
    the_cart = new CartPole(true,velocity);
    the_cart->nmarkov_long=false;
    the_cart->generalization_test=false;
    double fitness = -100000+the_cart->evalNet(&nn);
    delete the_cart; 
    return fitness;
 }

};

struct AnnPoleFitnessFunction : unary_function<combo_tree, double> {
 result_type operator()(argument_type tr) const {
    if (tr.empty())
        return MIN_FITNESS;

    tree_transform tt;
    ann nn = tt.decodify_tree(tr);

    return go_cart(&nn,100000);
 }

//     cart_and_pole() was take directly from the pole simulator written
//     by Richard Sutton and Charles Anderson.
int go_cart(ann *net,int max_steps) const
{
   float x,			/* cart position, meters */
         x_dot,			/* cart velocity */
         theta,			/* pole angle, radians */
         theta_dot;		/* pole angular velocity */
   int steps=0,y;

   int random_start=1;

   double in[5];  //Input loading array

   double out1;
   double out2;

//     double one_degree= 0.0174532;	/* 2pi/360 */
//     double six_degrees=0.1047192;
   double twelve_degrees=0.2094384;
//     double thirty_six_degrees= 0.628329;
//     double fifty_degrees=0.87266;

   if (random_start) {
     /*set up random start state*/
     x = (lrand48()%4800)/1000.0 - 2.4;
     x_dot = (lrand48()%2000)/1000.0 - 1;
     theta = (lrand48()%400)/1000.0 - .2;
     theta_dot = (lrand48()%3000)/1000.0 - 1.5;
    }
   else 
     x = x_dot = theta = theta_dot = 0.0;
   
   /*--- Iterate through the action-learn loop. ---*/
   while (steps++ < max_steps)
     {
       
       /*-- setup the input layer based on the four iputs --*/
       //setup_input(net,x,x_dot,theta,theta_dot);
       in[0]=1.0;  //Bias
       in[1]=(x + 2.4) / 4.8;;
       in[2]=(x_dot + .75) / 1.5;
       in[3]=(theta + twelve_degrees) / .41;
       in[4]=(theta_dot + 1.0) / 2.0;
       net->load_inputs(in);

       int depth = net->feedforward_depth();
       for(int x=0;x<depth;x++)
           net->propagate();

      /*-- decide which way to push via which output unit is greater --*/
       out1=net->outputs[0]->activation;
       out2=net->outputs[1]->activation;
       
       if (out1 > out2)
	 y = 0;
       else
	 y = 1;
       
       /*--- Apply action to the simulated cart-pole ---*/
       cart_pole(y, &x, &x_dot, &theta, &theta_dot);
       
       /*--- Check for failure.  If so, return steps ---*/
       if (x < -2.4 || x > 2.4  || theta < -twelve_degrees ||
	   theta > twelve_degrees) 
         return steps;             
     }
   
   return steps;
} 


//     cart_and_pole() was take directly from the pole simulator written
//     by Richard Sutton and Charles Anderson.
//     This simulator uses normalized, continous inputs instead of 
//    discretizing the input space.
/*----------------------------------------------------------------------
   cart_pole:  Takes an action (0 or 1) and the current values of the
 four state variables and updates their values by estimating the state
 TAU seconds later.
----------------------------------------------------------------------*/
void cart_pole(int action, float *x,float *x_dot, float *theta, float *theta_dot) const {
  float xacc,thetaacc,force,costheta,sintheta,temp;
  
  const float GRAVITY=9.8;
  const float MASSCART=1.0;
  const float MASSPOLE=0.1;
  const float TOTAL_MASS=(MASSPOLE + MASSCART);
  const float LENGTH=0.5;	  /* actually half the pole's length */
  const float POLEMASS_LENGTH=(MASSPOLE * LENGTH);
  const float FORCE_MAG=10.0;
  const float TAU=0.02;	  /* seconds between state updates */
  const float FOURTHIRDS=1.3333333333333;

  force = (action>0)? FORCE_MAG : -FORCE_MAG;
  costheta = cos(*theta);
  sintheta = sin(*theta);
  
  temp = (force + POLEMASS_LENGTH * *theta_dot * *theta_dot * sintheta)
    / TOTAL_MASS;
  
  thetaacc = (GRAVITY * sintheta - costheta* temp)
    / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta
		 / TOTAL_MASS));
  
  xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;
  
  /*** Update the four state variables, using Euler's method. ***/
  
  *x  += TAU * *x_dot;
  *x_dot += TAU * xacc;
  *theta += TAU * *theta_dot;
  *theta_dot += TAU * thetaacc;
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

struct ann_pole2_score {
   ann_pole2_score() { }
   double operator()(const combo_tree& tr) const {
      return p2ff(tr);
   }
   AnnPole2FitnessFunction p2ff;
};

struct ann_pole2_bscore {
    ann_pole2_bscore( ) { }

    behavioral_score operator()(const combo_tree& tr) const {
        behavioral_score bs(2);
        bs[0] = -moses::ann_pole2_score()(tr);
        bs[1] = tr.size();

        return bs;
    }
};

struct ann_pole_score {
   ann_pole_score() { }
   double operator()(const combo_tree& tr) const {
      return pff(tr);
   }
   AnnPoleFitnessFunction pff;
};

struct ann_pole_bscore {
    ann_pole_bscore( ) { }

    behavioral_score operator()(const combo_tree& tr) const {
        behavioral_score bs(2);
        bs[0] = -moses::ann_pole_score()(tr);
        bs[1] = tr.size();

        return bs;
    }
};

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
