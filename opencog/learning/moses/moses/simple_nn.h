#ifndef SIMPLEANN
#define SIMPLEANN

#include <vector>
#include <utility>
#include <iostream>
#include <stdlib.h>
#include <math.h>
using namespace std;

class ann;
class ann_node;
class ann_connection;

enum ann_nodetype { nodetype_input, nodetype_hidden, nodetype_output };

class ann_connection
{
public:
ann_connection(ann_node* s,ann_node* d,double w):source(s),dest(d),weight(w) { }
bool visited;
ann_node* source;
ann_node* dest;
double weight;
};

class ann_node
{
public:
ann_node(ann_nodetype type,void* _tag=NULL):activation(0.0),nodetype(type),tag(_tag) { }
bool visited;
vector<ann_connection*> out_connections; 
vector<ann_connection*> in_connections;
double activation;
double incoming; 
void* tag;
ann_nodetype nodetype;
};

class ann
{
public:
vector<ann_node*> nodes;
vector<ann_node*> inputs;
vector<ann_node*> outputs;
vector<ann_node*> hidden;
vector<ann_connection*> connections;

~ann()
{
for(int x=0;x<connections.size();x++)
	delete connections[x];
for(int x=0;x<nodes.size();x++)
	delete nodes[x];
}

ann_node* find_tag(void* t)
{
for(int x=0;x<nodes.size();x++)
	if (nodes[x]->tag==t)
		return nodes[x];
return NULL;
}

//simple threshold unit
double inline activation_fn_thresh(double incoming)
{
	if(incoming>=0.0)
		return 1.0;
	return -1.0;
}

double inline activation_fn(double incoming)
{
	return 1.0/(1.0+exp(-incoming));
}

void load_inputs(vector<double>& vals)
{
	for(int x=0;x<inputs.size();x++)
	{
		inputs[x]->activation=vals[x];
	}
}

void propagate()
{
	for(int x=0;x<nodes.size();x++)
	{
		if(nodes[x]->nodetype == nodetype_input)
			continue;
		nodes[x]->incoming=0.0;
		for(int y=0;y<nodes[x]->in_connections.size();y++)
			nodes[x]->incoming+=
				nodes[x]->in_connections[y]->weight*
				nodes[x]->in_connections[y]->source->activation;
		nodes[x]->activation = activation_fn(nodes[x]->incoming);	
	}
}

void add_connection(ann_node* s,ann_node* d,double weight)
{
	ann_connection* newconnection = new ann_connection(s,d,weight);
	connections.push_back(newconnection);
	s->out_connections.push_back(newconnection);
	d->in_connections.push_back(newconnection);
}

void add_node(ann_node* newNode)
{
	nodes.push_back(newNode);

	if(newNode->nodetype==nodetype_input)
		inputs.push_back(newNode);
	else if(newNode->nodetype==nodetype_output)
		outputs.push_back(newNode);
	else if(newNode->nodetype==nodetype_hidden);
		hidden.push_back(newNode);
}

};
/*
void test(double inp_val)
{

	ann nn;
	ann_node* input = new ann_node(nodetype_input);
	ann_node* hidden1 = new ann_node(nodetype_hidden);
	ann_node* hidden2 = new ann_node(nodetype_hidden);
	ann_node* output = new ann_node(nodetype_output);

	nn.add_node(input);
	nn.add_node(hidden1);
	nn.add_node(hidden2);
	nn.add_node(output);	

	nn.add_connection(input,hidden1,-0.5);
	nn.add_connection(input,hidden2,0.3);
	nn.add_connection(hidden1,output,0.7);
	nn.add_connection(hidden2,output,-0.5);
	
	vector<double> inp;
	inp.push_back(inp_val);
	nn.load_inputs(inp);
	nn.propagate();
	nn.propagate();
	cout << "ann input:" << nn.inputs[0]->activation << endl;
	cout << "ann output:" << nn.outputs[0]->activation << endl;
	return 0;
}
*/

#endif
