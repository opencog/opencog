	#include "StdAfx.h"


/// \todo This file causes compilation problems from Skeleton.h.
/// \todo Fix when it is actually needed for MOSES testing again.
/// \todo This file must be made to use namespace simple_evaluator
/// otherwise there's strange clashes

#if 0 


#ifndef USE_PSEUDOCORE

#include <iostream>

#include "tree_gen/tree_generation.h"
//#include "tree_gen/selection.h"
#include <boost/lexical_cast.hpp>

#include "../core/classes.h"

#include "../reasoning/PTLEvaluator.h"

#include "BOASimulatorAgent.h"

#include "../reasoning/AtomTableWrapper.h"

#include "RouletteSelector.h"

#include "VertexMetaData.h"
#include "ptl_fitness.h"
#include "fitness_driven.h"
#include "simplify.h"
#include "BDe.h"
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include "PLNShell.h"

void StressTest();
void BOA_PTL_tests();
void make_random_trees(std::vector<tree<Vertex> >& trees);

namespace haxx
{
	extern reasoning::iAtomTableWrapper* defaultAtomTableWrapper;
	extern set<Handle> atomRemoveSet;
};

//extern const int POPULATION_SIZE;
//extern const int N_GENERATIONS;

#if 1

//typedef unsigned int nodetype;

/*int main(int argc,char** argv) {
lexical_cast<int>(argv[1])
}*/

char procedure_names[][30] = {
	"intervalMember",
	"getMomentNode",
	"immediatelyAfter",
	"increasing",
	"decreasing",
	"intervalStart",
	"intervalEnd",
	"intervalLength",
	"observedDistanceTo",
	"observedDirectionTo",
	"observedVectorTo3D",
	"observedVectorTo1D"
};

void selectRandomLinks(int n, std::auto_ptr<Handle>& ret)
{
	///Doesn't check whether n is less than the atom table size...
	
    ret = std::auto_ptr<Handle>(new Handle[n]);
	HandleIterator* hi = MindDBProxy::getInstance()->getHandleIterator(ATOM,true);
	MindDBProxy::getInstance()->add(new Node(PREDICATE_NODE, strdup("myPredicate")));
	
	if (!hi->hasNext())
	{
		cprintf(3, "ERROR!\n");	
		gets(0);
	}	
	
	cprintf(3, "Trying to get random links...\n");	
	int offset = NUMBER_OF_CLASSES;
	
	for (int i = 0; i < n; i++)
	{
		// A rather slow method, but should work.
		offset += random()%50;
		
		for (int j = 0; j < offset; j++)
			if (hi->hasNext())
			{
				ret.get()[i] = hi->next();
				printf("Type: %d\n", ret.get()[i]->getType());
			}
			else //Start from the beginning
			{
				delete hi;
				hi = MindDBProxy::getInstance()->getHandleIterator(ATOM,true);
			}
		for (int k = 0; k < i; k++)
			if (ret.get()[k] == ret.get()[i]) //If exists already, skip back to the beginning
			{
				i--;
				k=i;
			}
		offset = 0;
	}
}

Handle AddAsNumberNode(int N);
Handle AddAsSchemaNode(char *str)
{
    char *buff = new char[128];
	strcpy(buff, str);
	return MindDBProxy::getInstance()->add(new Node(SCHEMA_NODE, buff,
		TruthValue::TRIVIAL_TV()));
}

int __BOAsim = 1;

BOASimulatorAgent::BOASimulatorAgent() {}

void BOASimulatorAgent::execute()
{
/*	if (__BOAsim > 200)
	{
		cprintf(1, "BOASimulatorAgent 400 runs ok. Press a key to clear atom space (if LOCAL_ATW).\n");
		getc(stdin);
#if LOCAL_ATW	
		((reasoning::LocalATW*)haxx::defaultAtomTableWrapper)->ClearAtomSpace();
		cprintf(1, "Atom space clear. All systems functioning. Press a key to continue.\n");
		getc(stdin);
		__BOAsim = 1;
#endif	
	}*/
	
	cprintf(1,"Executing BOASimulatorAgent...");
	
	/// Must initialize PTL before using it. We don't do tests on the same run,
	/// because that would cause PTL axioms invisible.
	if (__BOAsim++ < 3)
		PTLAgent1::getInstance().Initialize();
	else if (__BOAsim++ < 5)
	{
		foreach(Handle h, haxx::atomRemoveSet)
			MindDBProxy::getInstance()->remove(h);
		haxx::atomRemoveSet.clear();
	}
	else
//  while (1)
  	{	
	try
	{
		ThePLNShell.Launch();
		//BOA_PTL_tests();
	  
/*		// Once the more complex PTL functionality is again available,
		// we'll use these guys:
		
		LookupProvider* LP = new LookupProvider(0);
		HandleEntry* ret = PTLEvaluator::evaluate(InferenceTaskParameters(DEPTH1,
		LP,
		t));*/
	
		cprintf(1, "OK BOASimulatorAgent! Run %d\n", __BOAsim++);

	} catch(string s) {
		#ifdef NMDEBUG
			getc(stdin);
		#endif
		printf("Exception in BOASIM: %s\n", s.c_str()); }
	catch(boost::bad_get bg) {
		#ifdef NMDEBUG
			getc(stdin);
		#endif
		printf("Bad boost::get in BOASIM: %s\n", bg.what()); }
	catch(...) {
		#ifdef NMDEBUG
			getc(stdin);
		#endif
		
		puts("Exception in BOASimulatorAgent."); }
  }
}

/// The rest is just test code

void BackwardOsamaProofTest();
char* Type2Name(Type t);

using namespace reasoning;


void make_random_trees(std::vector<tree<Vertex> >& trees)
{
  using namespace trees;
  using namespace boost;
  using namespace std;

  NodeSelector<Vertex> ss;
  ss.add(Vertex((Handle)AND_LINK),3,1);   //second arg is arity, third is (relative) 
  ss.add(Vertex((Handle)AND_LINK),2,1);   //second arg is arity, third is (relative) 
  ss.add(Vertex((Handle)OR_LINK),3,1);   //selection likelihood
  ss.add(Vertex((Handle)OR_LINK),2,1);   //selection likelihood
  ss.add(Vertex((Handle)NOT_LINK),1,1);
  
  
  /*Node* atomA  = new Node(CONCEPT_NODE, strdup("Node A"), new FirstPTLTruthValue(0.36,36.0));
  Node* atomB1 = new Node(CONCEPT_NODE, strdup("Node B1"), new FirstPTLTruthValue(0.49,49.0));
  Node* atomB2 = new Node(CONCEPT_NODE, strdup("Node B2"), new FirstPTLTruthValue(0.42,42.0));
  Node* atomB3 = new Node(CONCEPT_NODE, strdup("Node B3"), new FirstPTLTruthValue(0.45,45.0));
  Node* atomC  = new Node(CONCEPT_NODE, strdup("Node C"), new FirstPTLTruthValue(0.41,41.0));
  MindDBProxy::getInstance()->add(atomA);
  MindDBProxy::getInstance()->add(atomB1);
  MindDBProxy::getInstance()->add(atomB2);
  MindDBProxy::getInstance()->add(atomB3);
  MindDBProxy::getInstance()->add(atomC);
  Handle hA = TLB::getHandle(atomA);
  Handle hB1 = TLB::getHandle(atomB1);
  Handle hB2 = TLB::getHandle(atomB2);
  Handle hB3 = TLB::getHandle(atomB3);
  Handle hC = TLB::getHandle(atomC);*/
  #if LOCAL_ATW
  Handle hA  = ((LocalATW*)haxx::defaultAtomTableWrapper)->addNode(CONCEPT_NODE,"Node A",new FirstPTLTruthValue(0.36,36.0),true,false);
  Handle hB1 = ((LocalATW*)haxx::defaultAtomTableWrapper)->addNode(CONCEPT_NODE,"Node B1",new FirstPTLTruthValue(0.49,49.0),true,false);
  Handle hB2 = ((LocalATW*)haxx::defaultAtomTableWrapper)->addNode(CONCEPT_NODE,"Node B2", new FirstPTLTruthValue(0.42,42.0),true,false);
  Handle hB3 = ((LocalATW*)haxx::defaultAtomTableWrapper)->addNode(CONCEPT_NODE,"Node B3",new FirstPTLTruthValue(0.45,45.0),true,false);
  Handle hC  = ((LocalATW*)haxx::defaultAtomTableWrapper)->addNode(CONCEPT_NODE,"Node C", new FirstPTLTruthValue(0.41,41.0),true,false);
  #else
  Handle hA  = haxx::defaultAtomTableWrapper->addNode(CONCEPT_NODE,"Node A",new FirstPTLTruthValue(0.36,36.0),true);
  Handle hB1 = haxx::defaultAtomTableWrapper->addNode(CONCEPT_NODE,"Node B1",new FirstPTLTruthValue(0.49,49.0),true);
  Handle hB2 = haxx::defaultAtomTableWrapper->addNode(CONCEPT_NODE,"Node B2", new FirstPTLTruthValue(0.42,42.0),true);
  Handle hB3 = haxx::defaultAtomTableWrapper->addNode(CONCEPT_NODE,"Node B3",new FirstPTLTruthValue(0.45,45.0),true);
  Handle hC  = haxx::defaultAtomTableWrapper->addNode(CONCEPT_NODE,"Node C", new FirstPTLTruthValue(0.41,41.0),true);
  #endif

  ss.add(Vertex((Handle)hA),0,1);
  ss.add(Vertex((Handle)hB1),0,1);
  ss.add(Vertex((Handle)hB2),0,1);
  ss.add(Vertex((Handle)hB3),0,1);
  ss.add(Vertex((Handle)hC),0,1);
  
  ramped_half_and_half(trees.begin(),trees.end(),ss,2,6);
  cprintf(3, "made trees!\n");
/*  for (vector<tree<Vertex> >::iterator tr=trees.begin();
       tr!=trees.end();++tr) {
    raw_print(*tr,tr->begin());
//    cout << (*tr) << endl;
    cout << "XXX" << endl;
  }*/
  cprintf(3,"OK\n");
 
  
  //assert(trees.size() == received_nodes);
}

void createRandomTree(tree<Vertex>& t, int d = 0, int maxd=0);
void createRandomTree(tree<Vertex>& t, int d, int maxd)
{
/*	t = atom(EVALUATION_LINK, 2,
							new atom(PREDICATE_NODE, "killed"),
							new atom(LIST_LINK, 2,
									new atom(CONCEPT_NODE, "Moses"),
									new atom(CONCEPT_NODE, "Osama")
								)
						)*/

	t = atom(OR_LINK, 2,
				new atom(EVALUATION_LINK, 2,
							new atom(PREDICATE_NODE, "killed"),
							new atom(LIST_LINK, 2,
									new atom(CONCEPT_NODE, "Moses"),
									new atom(CONCEPT_NODE, "Osama")
								)
						),
				new atom(CONCEPT_NODE, "Moses"))
		.makeHandletree(haxx::defaultAtomTableWrapper);

	tree<Vertex>::iterator ti = t.begin();
	
//	raw_print(t, ti, 3);
}

#define _abs(a) (((a)>0) ? (a) : -(a))

/// pass negative correct_strength to ignore the parameter altogether

const TruthValue& show_tv(tree<Vertex> t, float correct_strength)
{
cprintf(3,"Evaluating...");
Vertex res = simple_evaluator::PTLEvaluator::v_evaluate(t, t.begin(),STRENGTH_CONFIDENCE);

        Handle* p_arg;

        if (!res.empty())
        {
			
                if (!(p_arg = boost::get<Handle>(&res)))
                {
                        cprintf(2,"NO TV!");
                        return FALSE_TV();
                }
                else
                {
//					raw_print(t,t.begin(),0);
puts("Atom:");
printTree(*p_arg,0,0);					
					
                        const TruthValue& tv = getTruthValue(*p_arg);
						string stv = tv->toString();
                        cprintf(2,"%s - %s\n", stv.c_str(),
                                (correct_strength>-0.001 && _abs(tv->getMean()- correct_strength) > 0.01)
                                ? "FAIL" : "OK");
                        return tv;
                }
        }
        else
        {
                cprintf(2, "NO Handle!");
                return FALSE_TV();
        }
}

void show_tv(const atom& a, float correct_strength)
{
	cprintf(3,"TV for atom (type %d):\n", a.T);
	show_tv(a.makeHandletree(haxx::defaultAtomTableWrapper), correct_strength);
}

/*template<typename T>
struct TypeWrapper2
{
	T value;
	explicit TypeWrapper2(T _val) : value(_val) {}
	T operator=(const TypeWrapper2<T>& rhs) { return (value = rhs.value); }
	bool operator==(const TypeWrapper2<T>& rhs) const { return (value == rhs.value); }
	bool operator< (const TypeWrapper2<T>& rhs) const { return (value <  rhs.value); }
};

typedef TypeWrapper2<int> IntegerWrapper2;
*/

void BOA_PTL_tests()
{
	assert(haxx::defaultAtomTableWrapper);
	
	StressTest();
	return;	
	
	Handle interval1 = atom(LIST_LINK, 2,
				new atom(CONCEPT_NODE, "10"),
				new atom(CONCEPT_NODE, "20")
			).attach(haxx::defaultAtomTableWrapper);
	Handle interval2 = atom(LIST_LINK, 2,
				new atom(CONCEPT_NODE, "21"),
				new atom(CONCEPT_NODE, "30")
			).attach(haxx::defaultAtomTableWrapper);
	Handle interval3 = atom(LIST_LINK, 2,
				new atom(CONCEPT_NODE, "32"),
				new atom(CONCEPT_NODE, "33")
			).attach(haxx::defaultAtomTableWrapper);
	Handle interval4 = atom(LIST_LINK, 2,
				new atom(CONCEPT_NODE, "31"),
				new atom(CONCEPT_NODE, "31")
			).attach(haxx::defaultAtomTableWrapper);
	Handle interval5 = atom(LIST_LINK, 2,
				new atom(CONCEPT_NODE, "15"),
				new atom(CONCEPT_NODE, "25")
			).attach(haxx::defaultAtomTableWrapper);
	assert(interval1);
	assert(interval2);
	assert(interval3);
	assert(interval4);
	assert(interval5);
	
LOG(3, "Begin schema tests!");

	Handle h_intervalMember =
		atom(PREDICATE_NODE, "intervalMember").attach(haxx::defaultAtomTableWrapper);
	Handle h_immediatelyAfter =
		atom(PREDICATE_NODE, "immediatelyAfter").attach(haxx::defaultAtomTableWrapper);
	Handle h_getMoment =
		atom(SCHEMA_NODE, "getMomentNode").attach(haxx::defaultAtomTableWrapper);
	Handle h_increasing =
		atom(PREDICATE_NODE, "increasing").attach(haxx::defaultAtomTableWrapper);
	Handle h_decreasing =
		atom(PREDICATE_NODE, "decreasing").attach(haxx::defaultAtomTableWrapper);

	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_increasing),
		new atom(atom(LIST_LINK, 2,
			new atom(h_getMoment),
			new atom(interval1)).attach(haxx::defaultAtomTableWrapper)
		)
	), 1.0f); //true

	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_decreasing),
		new atom(atom(LIST_LINK, 2,
			new atom(h_getMoment),
			new atom(interval1)).attach(haxx::defaultAtomTableWrapper)
		)
	), 0.0f); //false

	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_intervalMember),
		new atom(atom(LIST_LINK, 2,
			new atom(atom(CONCEPT_NODE, "11").attach(haxx::defaultAtomTableWrapper)),
			new atom(interval1)).attach(haxx::defaultAtomTableWrapper)
		)
	), 1.0f); //true

	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_intervalMember),
		new atom(atom(LIST_LINK, 2,
			new atom(atom(CONCEPT_NODE, "9").attach(haxx::defaultAtomTableWrapper)),
			new atom(interval1)).attach(haxx::defaultAtomTableWrapper)
		)
	), 0.0f);
	
	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_intervalMember),
		new atom(atom(LIST_LINK, 2,
			new atom(atom(CONCEPT_NODE, "31").attach(haxx::defaultAtomTableWrapper)),
			new atom(interval4)).attach(haxx::defaultAtomTableWrapper)
		)
	), 1.0f); //true

	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_intervalMember),
		new atom(atom(LIST_LINK, 2,
			new atom(atom(CONCEPT_NODE, "22").attach(haxx::defaultAtomTableWrapper)),
			new atom(interval1)).attach(haxx::defaultAtomTableWrapper)
		)
	), 0.0f);
	
/*	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_immediatelyAfter),
		new atom(atom(LIST_LINK, 2,
			new atom(interval2),
			new atom(interval1)).attach(haxx::defaultAtomTableWrapper)
		)
	), 0.0f);
	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_immediatelyAfter),
		new atom(atom(LIST_LINK, 2,
			new atom(interval3),
			new atom(interval2)).attach(haxx::defaultAtomTableWrapper)
		)
	), 0.0f);
	show_tv(atom(EVALUATION_LINK, 2,
		new atom(h_immediatelyAfter),
		new atom(atom(LIST_LINK, 2,
			new atom(interval5),
			new atom(interval1)).attach(haxx::defaultAtomTableWrapper)
		)
	), 1.0f); //true*/
cprintf(3, "End schema tests!");	

	cprintf(3, "Testing a random tree...\n");
	tree<Vertex> t;
	createRandomTree(t);
	
//	printAtomTree(atom(t,t.begin()), 0,0);

	cprintf(3, "...\n");

	show_tv(t,1.0f);
	cprintf(3, "Rrandom tree test ok!\n");
	getc(stdin);
}

void StressTest()
{
        using namespace boost;
	using namespace rewrite;
	using namespace modeling;
  	assert(haxx::defaultAtomTableWrapper);

	const int POPULATION_SIZE = 25;
	const int N_GENERATIONS=5;
	const int TOURNAMENT_SIZE=2;

	vector<tree<Vertex> > trees(POPULATION_SIZE);

	make_random_trees(trees);
		
	BDe::gainOffset=(log2((gain_t)POPULATION_SIZE)/2.0)*10.0f;

	run_gen(trees.begin(),trees.end(),bind(simplePTLFitness,_1),
		POPULATION_SIZE,N_GENERATIONS,TOURNAMENT_SIZE,
		bind(logical_simplify<Vertex>,_1));
	
		
	/**
	for (int i=0;i<N;i++) {
			cprintf(2, "When Evaluating:\n");
			TruthValue * tv = show_tv(trees[i], -1.0f);
			if (tv->getConfidence() > 0.000001f && tv->getMean() > 0.000001f)
			{
				raw_print(trees[i], trees[i].begin());		
//				getc(stdin);
			}
		}
		}*/
	cprintf(2, "Ended BOA test run.\n");
}

#endif

#endif //#ifndef USE_PSEUDOCORE

#endif
