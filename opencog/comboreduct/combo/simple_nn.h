/** simple_nn.h --- 
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


#ifndef _COMBO_SIMPLE_NN_H
#define _COMBO_SIMPLE_NN_H

#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/RandGen.h>

#include "vertex.h"

namespace opencog { namespace combo {

//anns are composed of nodes and connections
class ann;
class ann_node;
class ann_connection;

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

struct compare_connection
{
    bool operator() (ann_connection* lhs, ann_connection* rhs) const;
};

//simple iterator types
typedef std::vector<ann_node*>::iterator ann_node_it;
typedef std::vector<ann_connection*>::iterator ann_connection_it;

//different node types
enum ann_nodetype { nodetype_input, nodetype_hidden, nodetype_output };

//a connection is simply a weight between two nodes
class ann_connection
{
public:
    ann_connection(ann_node* s, ann_node* d, double w):
            source(s), dest(d), weight(w) { }
    ann_node* source;
    ann_node* dest;
    double weight;

    friend std::ostream& operator<<(std::ostream& os, const ann_connection* a) {
        os << "Connection with weight " << a->weight << std::endl;
        return os;
    }

};


//node class
class ann_node
{
public:
    ann_node(ann_nodetype type, int _tag = 0, ann_node* _ptr=NULL)
        : visited(false), memory_ptr(_ptr), activation(0.0), tag(_tag), nodetype(type) {}

    bool visited; //used for constructing combotrees
    int counter; //used for determining network depth
    int id; //internal identifier
    double sort_val; //used for sorting nodes    
    ann_node* memory_ptr; // what hidden node will feed this input,
                          // when NULL there is no such
                          // node. Obviously it is assume that only
                          // input nodes can have non null memory_ptr
    bool memory_neurone; // true if it is used as input of a memory
                         // neurone, that is another neurone n points
                         // to it via n.memory_ptr. Only hidden node
                         // can have this attribute true
  
    //connections
    std::vector<ann_connection*> out_connections;
    std::vector<ann_connection*> in_connections;

    //what is the activation level
    double activation;
    //what is the incoming signal sum?
    double incoming;
    //tag -> corresponds to combo node #s
    int tag;
    //what kind of node is this
    ann_nodetype nodetype;

    void calculate_sort_value(void)
    {
        ann_connection_it it;        
        //reset sort value
        sort_val=0.0;
        //add magnitude of all incoming connections
        for(it=in_connections.begin();it!=in_connections.end();++it)
            sort_val += fabs((*it)->weight);
        //add magnitude of all outgoing connections
        for(it=out_connections.begin();it!=out_connections.end();++it)
            sort_val += fabs((*it)->weight);
    }

    void sort_connections(void)
    {
       sort(in_connections.begin(),in_connections.end(),compare_connection());
    }

    friend std::ostream& operator<<(std::ostream& os, const ann_node* n) {
        os << n->id << ":";
        if (n->nodetype == nodetype_input) os << "input";
        else if (n->nodetype == nodetype_hidden) os << "hidden";
        else if (n->nodetype == nodetype_output) os << "output";
        return os;
    }
};


//ANN class..
class ann
{
public:

    //complete list of nodes
    std::vector<ann_node*> nodes;
    //only the input nodes
    std::vector<ann_node*> inputs;
    //only the output nodes
    std::vector<ann_node*> outputs;
    //only the hidden nodes
    std::vector<ann_node*> hidden;

    //a list of all the connections
    std::vector<ann_connection*> connections;

    ann() {}

    ~ann() {
        ann_node_it iter;
        for (iter = nodes.begin();iter != nodes.end();++iter)
            delete (*iter);
    }
   
    void reduce()
    {
        ann_connection_it iter;
        bool dirty=true;
        while(dirty)
        {
           dirty = false; 
           // remove impact-less connections. @todo: the code can be
           // both simpler and more efficient by having
           // remove_connection return the next iterator
           for (iter = connections.begin();iter != connections.end(); ++iter)
           {
              if((*iter)->weight != 0.0)
                 continue;
              if (!remove_connection(*iter))
                 continue;
              iter = connections.begin();
              if (iter==connections.end())
                 break;
           }

           //remove disconnected hidden neurons
           ann_node_it node_iter;
           for (node_iter = hidden.begin(); node_iter != hidden.end(); 
                   ++node_iter)
           {
               //if there are no outgoing connections and it is not
               //used as input of a memory neurone, then this neuron
               //has no impact
               if((*node_iter)->out_connections.size()==0 
                  && !(*node_iter)->memory_neurone)
               {
                   remove_node(*node_iter);
                   node_iter = hidden.begin();
                   dirty=true;
                   if (node_iter==hidden.end())
                       break;
               }
           }
        }
       
       //now sort nodes (and connections by this sort value so there is a
       //cannonical order of expressing an nn)
       ann_node_it node_iter;
       for (node_iter = nodes.begin(); node_iter != nodes.end();
               ++node_iter)
       {
          (*node_iter)->calculate_sort_value();
       }
       
       for (node_iter = nodes.begin(); node_iter != nodes.end();
               ++node_iter)
       {
          (*node_iter)->sort_connections();
       }

    } 

    //write out file in DOT format
    //common graph visualization format.
    void write_dot(const char* filename)
    {
        std::ofstream outfile(filename);
        ann_connection_it it;
        ann_node_it nit; 
        outfile << "digraph g { " << std::endl;

        for(nit=inputs.begin();nit!=inputs.end();++nit)
        {
            outfile << "N" << (*nit)->tag << " [shape=box]" << std::endl;
        }
        
        for(nit=outputs.begin();nit!=outputs.end();++nit)
        {
            outfile << "N" << (*nit)->tag << " [shape=triangle]" << std::endl;
        }

        for(it=connections.begin();it!=connections.end();++it)
        {
            int n1 = (*it)->source->tag;
            int n2 = (*it)->dest->tag;
            outfile << "N" << n1 << " -> N" << n2 << " ";
            if((*it)->weight > 0.3)
                outfile << "[color=green] ";
            else if ((*it)->weight < -0.3)
                outfile << "[color=red] ";
            outfile << std::endl;
        }

        
        for(nit=inputs.begin();nit!=inputs.end();++nit)
        {
            if((*nit)->memory_ptr)
            {
                int n1 = (*nit)->memory_ptr->tag;
                int n2 = (*nit)->tag;
                outfile << "N" << n1 << " -> N" << n2 << " [style=dotted] ";
                outfile << std::endl;
            }
        }

        outfile << " { rank=same; ";
        for(nit=inputs.begin();nit!=inputs.end();++nit)
        {
            outfile << "N" << (*nit)->tag << " ";
        }
        outfile << " } " << std::endl;


        outfile << " { rank=same; ";
        for(nit=outputs.begin();nit!=outputs.end();++nit)
        {
            outfile << "N" << (*nit)->tag << " ";
        }
        outfile << " } " << std::endl;
        
        outfile << "}" << std::endl;
    }

    //add a new hidden node that is connected to all the outputs
    //at least one input and some hidden nodes
    bool add_new_hidden()
    {
        ann_node_it iter; 
       
        //increment tag for new neuron
        int tag = biggest_tag() + 1;

        //create new hidden node
        ann_node *new_hidden = new ann_node(nodetype_hidden,tag,NULL);

        //connect the new node to all outputs
        for(iter=outputs.begin();iter!=outputs.end();++iter)
        {
            add_connection(new_hidden,(*iter),0.0);
        }

        bool connected=false;

        while(!connected)
        {

            //selectively connect the node from inputs
            int add_chance = 100;
            for(iter=inputs.begin();iter!=inputs.end();++iter)
            {
                if (rand()%100 < add_chance) 
                {
                    connected=true;
                    add_connection((*iter),new_hidden,0.0);
                }
            }

            //connect all hidden nodes to the new hidden node
            for(iter=hidden.begin();iter!=hidden.end();++iter)
            {
                if (rand()%100 < add_chance)
                {
                    connected=true;
                    add_connection((*iter),new_hidden,0.0);
                }
            }
        }
        add_node(new_hidden);
        return true;
    }

    //add a new memory input, which will be
    //connected to a hidden node, feeding it 
    //input
    bool add_memory_input()
    {
        //first enumerate valid hidden nodes 
        ann_node_it iter;
        ann_node_it in_iter;
        std::vector<ann_node*> possible;
        
        //see if there are any viable hidden nodes that are
        //not already being used to feed a memory input
        for (iter = hidden.begin(); iter !=hidden.end();++iter)
        {
            bool memory=false;
            for(in_iter = inputs.begin();in_iter!= inputs.end(); ++in_iter)
            {
                if(!(*in_iter)->memory_ptr)
                    continue;
                if((*in_iter)->memory_ptr == (*iter)) {
                    memory=true;
                    break;
                }
            }
            
            if (!memory)
            {
                possible.push_back(*iter);
            }
        }

        //if no valid nodes, return failure
        if(possible.empty())
        {
            return false;
        }

        //select eligble hidden node
        int selected = rand() % possible.size(); // @todo:replace this by RandGen
        ann_node *hidden_neuron = possible[selected];
        hidden_neuron->memory_neurone = true;

        //increment tag for new neuron
        int tag = biggest_tag() + 2;

        //create new input node
        ann_node *new_input = new ann_node(nodetype_input,tag,hidden_neuron);

        //create possible connections to hidden & output nodes
        bool connected=false;
        while(!connected)
        {
            int add_chance = 100;
            for(iter=hidden.begin();iter!=hidden.end();++iter)
                if (rand()%100 < add_chance)
                {
                    connected=true;
                    add_connection(new_input,(*iter),0.0);
                }

            for(iter=outputs.begin();iter!=outputs.end();++iter)
                if (rand()%100 < add_chance)
                {
                    connected=true;
                    add_connection(new_input,(*iter),0.0);
                }
        }
        add_node(new_input);
        return true;
    }

    //find the biggest tag, so we can add a new one
    int biggest_tag()
    {
        int max = -1;
        ann_node_it iter;
        for(iter = nodes.begin();iter != nodes.end();++iter)
            if ((*iter)->tag > max)
                max = (*iter)->tag;
        return max;
    }
    
    //make all nodes appear unvisited, useful for graph algorithms
    void reset_visited() {
        ann_node_it iter;
        for (iter = nodes.begin();iter != nodes.end();++iter)
            (*iter)->visited=false;
    }

    //return the node with a given tag
    ann_node* find_tag(int t) {
        for(ann_node_it iter = nodes.begin(); iter != nodes.end(); ++iter)
            if ((*iter)->tag == t)
                return *iter;
        return NULL;
    }

    //simple threshold unit
    double inline activation_fn_thresh(double incoming) const {
        if (incoming >= 0.0)
            return 1.0;
        return -1.0;
    }

    //simple sigmoid activation
    double inline activation_fn(double incoming) const {
        return 1.0 / (1.0 + exp(-incoming));
    }

    //load input values into the ann
    //memory inputs are fed by the hidden nodes they
    //are connected to
    void load_inputs(double* vals) {
        unsigned counter = 0;
        for (unsigned x = 0;x < inputs.size();++x) {
            if(inputs[x]->memory_ptr)
                inputs[x]->activation = inputs[x]->memory_ptr->activation;
            else
                inputs[x]->activation = vals[counter++];
        }
    }

    void load_inputs(const std::vector<double>& vals) {
        unsigned counter = 0;
        for (unsigned x = 0;x < inputs.size();++x) {
            if(inputs[x]->memory_ptr)
                inputs[x]->activation = inputs[x]->memory_ptr->activation;
            else
                inputs[x]->activation = vals[counter++];
        }
    }

    //feed activation one timestep through the ANN
    void propagate() {
        ann_node_it iter;

        //propagate signals through net
        for (iter = nodes.begin();iter != nodes.end();++iter) {

            //skip input nodes, their activations are already set
            if ((*iter)->nodetype == nodetype_input)
                continue;

            //reset the incoming signal to 0
            (*iter)->incoming = 0.0;

            //accumulate incoming singal from incoming connections
            for (unsigned int y = 0;y < (*iter)->in_connections.size();++y)
                (*iter)->incoming +=
                    (*iter)->in_connections[y]->weight *
                    (*iter)->in_connections[y]->source->activation;

            //activate the node
            (*iter)->activation = activation_fn((*iter)->incoming);
        }
    }

    //determine the depth of the network so we can
    //propagate the proper amount to get the signal
    //all the way through the net
    int feedforward_depth() {
        
        //reset the node counters
        for (unsigned int x = 0;x < nodes.size();++x)
            nodes[x]->counter = 0;

        //recursively determine longest feedforward path 
        //through net
        for (unsigned int x = 0;x < inputs.size();++x)
            feedforward_depth_recurse(inputs[x]);

        //the output counters will contain the longest
        //path to that particular output, we want the
        //maximum of all of these counters
        int max_depth = 0;
        for (unsigned int x = 0;x < outputs.size();++x)
            if (outputs[x]->counter > max_depth)
                max_depth = outputs[x]->counter;

        return max_depth;
    }

    //recursive function to determine max path
    //through ann
    void feedforward_depth_recurse(ann_node* n) {
        //increment counter by one
        int depth = n->counter + 1;

        //consider all of the outgoing connections
        for (unsigned int x = 0;x < n->out_connections.size();++x) {
            //what is the destination of this particular connection
            ann_node* dest = n->out_connections[x]->dest;
            //what is the longest distance found to the destination node?
            int node_depth = dest->counter;

            //if this path is further, overwrite old maximum
            //and consider the destination
            if (depth > node_depth) {
                dest->counter = depth;
                feedforward_depth_recurse(dest);
            }
        }
    }

    //add a new connection to the ANN
    void add_connection(ann_node* s, ann_node* d, double weight) {
        ann_connection* newconnection = new ann_connection(s, d, weight);
        connections.push_back(newconnection);
        s->out_connections.push_back(newconnection);
        d->in_connections.push_back(newconnection);
    }
    
    void remove_from_vec(ann_node* n, std::vector<ann_node*> & l)
    {
        ann_node_it loc = find(l.begin(), l.end(), n);
        if(loc!=l.end())
            l.erase(loc);
    }

    void remove_from_vec(ann_connection* c, std::vector<ann_connection*>& l)
    {
        ann_connection_it loc = find(l.begin(), l.end(), c);
        if(loc!=l.end())
            l.erase(loc);
    } 
    
    void delete_connections(std::vector<ann_connection*>& c)
    { 
        for(ann_connection_it iter=c.begin(); iter!= c.end();)
        {
            remove_connection(*iter);
            iter=c.begin();
        }
    }

    bool remove_node(ann_node* node)
    {
        //first delete all the connections
        delete_connections(node->out_connections);
        delete_connections(node->in_connections);
        //remove it from all vectors
        remove_from_vec(node,nodes);
        remove_from_vec(node,inputs);
        remove_from_vec(node,hidden);
        remove_from_vec(node,outputs);
        //set n->memory_ptr = NULL for all input nodes using it
        remove_from_memory_ptr(node);
        delete node;
        return true;
    }  

    //set n->memory_ptr = NULL for all input nodes using n
    void remove_from_memory_ptr(ann_node* n)
    {
        for(ann_node_it it=inputs.begin(); it!=inputs.end(); ++it) {
            if((*it)->memory_ptr == n)
                (*it)->memory_ptr = NULL;
        }
    }

    bool remove_connection(ann_connection* conn)
    {
        remove_from_vec(conn,connections);
        remove_from_vec(conn,conn->source->out_connections);
        remove_from_vec(conn,conn->dest->in_connections);
        delete conn;
        return true;   
    }

    //add a new node to the ANN
    void add_node(ann_node* newnode) {
        nodes.push_back(newnode);

        if (newnode->nodetype == nodetype_input)
        {
            inputs.push_back(newnode);
        }
        else if (newnode->nodetype == nodetype_output)
        {
            outputs.push_back(newnode);
        
        }
        else if (newnode->nodetype == nodetype_hidden)
        {
            hidden.push_back(newnode);
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const ann *a) {
        for (unsigned int x = 0;x < a->connections.size();++x)
            std::cout << a->connections[x]->source->id << " -> " <<
                 a->connections[x]->dest->id << " : " <<
                 a->connections[x]->weight << std::endl;
        return os;
    }
};

}} // ~namespaces combo opencog

#endif // _COMBO_SIMPLE_NN_H
