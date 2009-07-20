#ifndef SIMPLEANN
#define SIMPLEANN

#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
using namespace std;

class ann;
class ann_node;
class ann_connection;

typedef vector<ann_node*>::iterator ann_node_it;
typedef vector<ann_connection*>::iterator ann_connection_it;

enum ann_nodetype { nodetype_input, nodetype_hidden, nodetype_output };

class ann_connection
{
public:
    ann_connection(ann_node* s, ann_node* d, double w):
            source(s), dest(d), weight(w) { }
    bool visited;
    ann_node* source;
    ann_node* dest;
    double weight;

    friend ostream& operator<<(ostream& os, const ann_connection* a) {
        os << "Connection with weight " << a->weight << endl;
        return os;
    }
};

class ann_node
{
public:
    ann_node(ann_nodetype type, int _tag = 0, ann_node* _ptr=NULL):
            activation(0.0), nodetype(type), tag(_tag), memory_ptr(_ptr) { 
        if(_ptr != NULL)
            memory_node=true;
        else
            memory_node=false;
    }
    bool visited;
    int counter; //used for determining network depth
    int id;
    
    bool memory_node; //is this a memory node 
    ann_node* memory_ptr; //what node 
  
    vector<ann_connection*> out_connections;
    vector<ann_connection*> in_connections;
    double activation;
    double incoming;
    int tag;
    ann_nodetype nodetype;

    friend ostream& operator<<(ostream& os, const ann_node* n) {
        if (n->nodetype == nodetype_input) os << "input" << endl;
        else if (n->nodetype == nodetype_hidden) os << "hidden" << endl;
        else if (n->nodetype == nodetype_output) os << "output";
        return os;
    }
};

class ann
{
public:
    int node_count;
    vector<ann_node*> nodes;
    vector<ann_node*> inputs;
    vector<ann_node*> outputs;
    vector<ann_node*> hidden;
    vector<ann_connection*> connections;

    ann(): node_count(0) { }

    ~ann() {
        ann_node_it iter;
        for (iter = nodes.begin();iter != nodes.end();iter++)
            delete (*iter);
    }
    
    void write_dot(const char* filename)
    {
        ofstream outfile(filename);
        ann_connection_it it;
        ann_node_it nit; 
        outfile << "digraph g { " << endl;

        for(nit=inputs.begin();nit!=inputs.end();nit++)
        {
            outfile << "N" << (*nit)->tag << " [shape=box]" << endl;
        }
        
        for(nit=outputs.begin();nit!=outputs.end();nit++)
        {
            outfile << "N" << (*nit)->tag << " [shape=triangle]" << endl;
        }

        for(it=connections.begin();it!=connections.end();it++)
        {
            int n1 = (*it)->source->tag;
            int n2 = (*it)->dest->tag;
            outfile << "N" << n1 << " -> N" << n2 << " ";
            if((*it)->weight > 0.3)
                outfile << "[color=green] ";
            else if ((*it)->weight < -0.3)
                outfile << "[color=red] ";
            outfile << endl;
        }

        
        for(nit=inputs.begin();nit!=inputs.end();nit++)
        {
            if((*nit)->memory_node)
            {
                int n1 = (*nit)->memory_ptr->tag;
                int n2 = (*nit)->tag;
                outfile << "N" << n1 << " -> N" << n2 << " [style=dotted] ";
                outfile << endl;
            }
        }

        outfile << " { rank=same; ";
        for(nit=inputs.begin();nit!=inputs.end();nit++)
        {
            outfile << "N" << (*nit)->tag << " ";
        }
        outfile << " } " << endl;


        outfile << " { rank=same; ";
        for(nit=outputs.begin();nit!=outputs.end();nit++)
        {
            outfile << "N" << (*nit)->tag << " ";
        }
        outfile << " } " << endl;
        
        outfile << "}" << endl;
    }

    bool add_new_hidden()
    {
        ann_node_it iter; 
       
        //increment tag for new neuron
        int tag = biggest_tag() + 1;

        //create new hidden node
        ann_node *new_hidden = new ann_node(nodetype_hidden,tag,NULL);

        //connect the new node to all outputs
        for(iter=outputs.begin();iter!=outputs.end();iter++)
        {
            cout << "adding connection from new node to output " << endl;
            add_connection(new_hidden,(*iter),0.0);
        }

        bool connected=false;

        while(!connected)
        {

        //selectively connect the node from inputs
        int add_chance = 30;
        for(iter=inputs.begin();iter!=inputs.end();iter++)
        {
            cout << "considering adding connection from input to new hidden" << endl;
            if (rand()%100 < add_chance)
            {
                connected=true;
                add_connection((*iter),new_hidden,0.0);
                cout << "adding connection from input to new hidden" << endl;
            }
        }

        //connect all hidden nodes to the new hidden node
        for(iter=hidden.begin();iter!=hidden.end();iter++)
        {
            connected=true;
            add_connection((*iter),new_hidden,0.0);
            cout << "adding connection from hidden to new hidden" << endl;
        }
        }
        add_node(new_hidden);
        return true;
    }

    bool add_memory_input()
    {
        //first enumerate valid hidden nodes 
        ann_node_it iter;
        ann_node_it in_iter;
        vector<ann_node*> possible;
        
        cout << "Adding memory node..."  << endl;
        for (iter = hidden.begin(); iter !=hidden.end();iter++)
        {
            bool memory=false;
            cout << "Checking for unmemorized hidden nodes..." << endl;
            for(in_iter = inputs.begin();in_iter!= inputs.end(); in_iter++)
            {
                if(!(*in_iter)->memory_node)
                    continue;
                if((*in_iter)->memory_ptr == (*iter)) {
                    memory=true;
                    break;
                }
            }
            
            if (!memory)
            {
                cout << "Potential node found..." << endl;
                possible.push_back(*iter);
            }
        }

        //if no valid nodes, return failure
        if(possible.size() == 0 )
        {
            cout << "No eligble hidden nodes to feed into memory input" <<endl;
            return false;
        }

        //select eligble hidden node
        int selected = rand() % possible.size();
        ann_node *hidden_neuron = possible[selected];

        //increment tag for new neuron
        int tag = biggest_tag() + 2;

        //create new input node
        ann_node *new_input = new ann_node(nodetype_input,tag,hidden_neuron);

        //create possible connections to hidden & output nodes
        int add_chance = 100;
        for(iter=hidden.begin();iter!=hidden.end();iter++)
            if (rand()%100 < add_chance)
                add_connection(new_input,(*iter),0.0);

        for(iter=outputs.begin();iter!=outputs.end();iter++)
            if (rand()%100 < add_chance)
                add_connection(new_input,(*iter),0.0);
        
        add_node(new_input);
        cout << "New input added..." << endl;
        return true;
    }

    int biggest_tag()
    {
        int max = -1;
        ann_node_it iter;
        for(iter = nodes.begin();iter != nodes.end();iter++)
            if ((*iter)->tag > max)
                max = (*iter)->tag;
        return max;
    }
    void reset_visited() {
        ann_node_it iter;
        for (iter = nodes.begin();iter != nodes.end();iter++)
            (*iter)->visited=false;
    }

    ann_node* find_tag(int t) {
        ann_node_it iter;
        for (iter = nodes.begin();iter != nodes.end();iter++)
            if ((*iter)->tag == t)
                return *iter;
        return NULL;
    }

    //simple threshold unit
    double inline activation_fn_thresh(double incoming) {
        if (incoming >= 0.0)
            return 1.0;
        return -1.0;
    }

    double inline activation_fn(double incoming) {
        return 1.0 / (1.0 + exp(-incoming));
    }

    void load_inputs(double* vals) {
        unsigned int counter=0;
        //memory nodes are loaded by the propagate method
        for (unsigned int x = 0;x < inputs.size();x++)
        {
            if(!inputs[x]->memory_node)
                inputs[x]->activation = vals[counter++];
            else
                inputs[x]->activation = inputs[x]->memory_ptr->activation;
        }
    }

    void load_inputs(vector<double>& vals) {
        for (unsigned int x = 0;x < inputs.size();x++) {
            if(!inputs[x]->memory_node)
                inputs[x]->activation = vals[x];
            else
                inputs[x]->activation = inputs[x]->memory_ptr->activation;
        }
    }

    void propagate() {
        ann_node_it iter;

        //propagate signals through net
        for (iter = nodes.begin();iter != nodes.end();iter++) {
            if ((*iter)->nodetype == nodetype_input)
                continue;
            (*iter)->incoming = 0.0;

            for (unsigned int y = 0;y < (*iter)->in_connections.size();y++)
                (*iter)->incoming +=
                    (*iter)->in_connections[y]->weight *
                    (*iter)->in_connections[y]->source->activation;
            (*iter)->activation = activation_fn((*iter)->incoming);
        }
    }

    int feedforward_depth() {
        for (int x = 0;x < nodes.size();x++)
            nodes[x]->counter = 0;

        for (int x = 0;x < inputs.size();x++)
            feedforward_depth_recurse(inputs[x]);

        int max_depth = 0;
        for (int x = 0;x < outputs.size();x++)
            if (outputs[x]->counter > max_depth)
                max_depth = outputs[x]->counter;

        return max_depth;
    }

    void feedforward_depth_recurse(ann_node* n) {
        int depth = n->counter + 1;
        for (int x = 0;x < n->out_connections.size();x++) {
            ann_node* dest = n->out_connections[x]->dest;
            int node_depth = dest->counter;
            if (depth > node_depth) {
                dest->counter = depth;
                feedforward_depth_recurse(dest);
            }
        }
    }

    void add_connection(ann_node* s, ann_node* d, double weight) {
        ann_connection* newconnection = new ann_connection(s, d, weight);
        connections.push_back(newconnection);
        s->out_connections.push_back(newconnection);
        d->in_connections.push_back(newconnection);
    }

    void add_node(ann_node* newnode) {
        newnode->id = node_count;
        node_count++;
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

    friend ostream& operator<<(ostream& os, const ann *a) {
        for (unsigned int x = 0;x < a->connections.size();x++)
            cout << a->connections[x]->source->id << " -> " <<
                 a->connections[x]->dest->id << " : " <<
                 a->connections[x]->weight << endl;
        return os;
    }
};

/*
void test(double inp_val)
{

 ann nn;
 ann_node input(nodetype_input);
 ann_node hidden1(nodetype_hidden);
 ann_node hidden2(nodetype_hidden);
 ann_node output(nodetype_output);

 nn.add_node(input);
    nn.add_node(hidden1);
 nn.add_node(hidden2);
 nn.add_node(output);

    ann_connection c1(&input,&hidden1,-0.5);
    ann_connection c2(&input,&hidden2,0.3);
    ann_connection c3(&hidden1,&hidden2,0.7);
    ann_connection c4(&hidden2,&output,-0.5);
 nn.add_connection(c1);
 nn.add_connection(c2);
 nn.add_connection(c3);
 nn.add_connection(c4);

 vector<double> inp;
 inp.push_back(inp_val);
 nn.load_inputs(inp);
 nn.propagate();
 nn.propagate();
 cout << "ann input:" << nn.inputs[0]->activation << endl;
 cout << "ann output:" << nn.outputs[0]->activation << endl;
}
int main(int argc,char **argv)
{
    test(0.3);
    return 0;
}
*/
#endif
