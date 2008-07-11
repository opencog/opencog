/**

GENERAL PLN SOURCE CODE NOTES:

Bad Performance:
- In various unifier methods> if parent is variable, no need to combinate the children.
- In Rules, input validity check is not currently active.

NOTE:

- FW_VARIABLE_NODEs in o2iMetaExtra() are not memory-managed

- LookUpRule automatically omits elements with confidence 0.
- PostCondition in GenericEvaluator.cpp prevents confidence-0 atoms entering via other rules,
too.
- TheNM is a hack that only functions with a system with a single AtomTable.
- Normalization is non-trivial with fuzzy TVs. Degree of allowable loss-of-info L must be
decided (=policy), and only [TV.strength > 1.0-L] should be considered TRUE, etc.
- No Nested ForAlls => "defined($X) @ node" currently fails.
- ForAllRule's TV determines the TV of the outest link within the ForAll, not affecting
the internal ones. All atoms within ForAll should originally have confidence = 0.
- ORRule is only evaluated to the binary pair exclusion precision (A+B+C-AB-AC-BC).
- Policy: Rule.o2i methods must make sure they don't waste time. ResMan only guards the
blocking Rule method calls on a serial manner - it cannot survive a slow o2i implementation
- Always puts ANDRules before SimpleANDRules, since the latter accept more general output type,
but don't do the flattening magic.
- BW chaining UnorderedLinkPermutationRule only works for ANDLinks. Try it with ORLinks and die.
FW chaining should be ok. The reason is logical...
- CrispUnificationRule & UnorderedLinkPermutationRule use HYPOTHETICAL_LINKs - pseudo atoms that,
given as a parameter to the rule, make the rule output the desired kind of outcome
- Often you can choose to either increase the complexity of a Rule, or make multiple
versions of the rule (eg. ANDRule<N>).
- MP that only tests __INSTANCE_OF property is equivalent to a MP construct that uses
FW_VARIABLE_NODEs, but faster to use (no substitutions needed).
- ATOM-type MP will always be true for any parameter atom.
- A non-MP atom x cast into MetaPredicate will amount to EQUAL(x)
- ListLink TVs don't mean anything.
- Using log slows down a lot, even if loglevel 0!
- Shouldn't Handles/TVs recall the way they were produced? Eg. ImplicationLinks
created by inversion should be rejected if they are later produced another way;
how about updating the TVs when U size changes?
- When addLink splits a FORALL_LINK into multiple parts, a ListLink is returned!
- IF we take a link with arity A & type T be a subtype of link with
arity A'>A & type T,
then Atom equals VariableNode, and variable unification is a special case of
finding the subtype of a given atom.

- predicate <=> type conversion

- Should the TruthValues of any links that contain variables be neglected?
If so, then should they be embedded always in HypLinks? Probably not.
But then, every time we query for a link L, we must check
a) whether it's (_always_) embedded in a HypoLink and
b) whether it contains variables. (Unless it's a ForAll link!)
In either case, the TV is meaningless.

- In CrispUnification of ForAllLinks, subtrees must be proved separately.
Eg. consider
ForAll<tv1>
	List($X)
	Implies
		And
			F
			$X
		$X.

Once I substitute A<tv2> for X, I get
	Implies<tv1>
		And<tv3>
			F
			A<tv2>
		A<tv2>.

But in order to find out tv3, I must also FindTV of
 		And
			F
			A<tv2>.

NOTE 5: On Unification
0. A "simple MP" = an atom A that is not a MP, but will implicitly be considered as the complex MP "EqualTo(A)". Also an MP atom whose outgoing set contains no variables is considered a simple one, since it can no longer be used as a "real" MP.
1. While BW-chaining, the proof of formulas that are simple MPs but contain variables is never attempted.
2. The Unifier rule produces, for the desired output, the non-redundant set of all possible combinations of static and variable structures that would, upon unification, produce the output. All these structures will be simple MPs.
3. No "actual" unification will take place - the results of the inverted Unifier rule will be looked for, and if they aren't found, the proof path is cut.
4. When a query is made for a MP, it is first considered a simple MP and then a complex one. Ie. first we query the atom table for whether the exact atom designated by the MP exists, and if it doesn't, then we try to actually "run" the MP (in case it has a complex aspect, too), ie. seek atoms that fulfill the MP. 

*/


/*
Relevant shell commands:
0 - normal log level
2 - recommended maximally informative log level
c - switch the recording of inference trails ON/OFF (default: OFF)
n - expand the tree's whole next level
e - evaluate the current tree
a #h - Show the plan ie. sequence of 'do' statements pertaining to inference result handle #h
B #i - Show the direct results (by lookup or hypothesis) of BIT node #i
b #i - Show the target atom and pre-bindings of BIT node #i (pre-binding is a binding which has been made when producing the node; normally the bindings are carried around without actually carrying them out before Evaluate() command)
S #n - Execute the #n of the "fittest" BIT nodes
f - Show the current BIT node expansion pool sorted by heuristics fitness and Execute the "fittest" BIT node
P - print the whole current inference (BIT) tree

These should be bug-free, but there's no type checking of parameters, so providing eg. BIT node number instead of Handle number will SegFault.
*/

#include "PLN.h"
#include "Rules.h"

#define BackInferenceTreeRootT BITNodeRoot

//#include "PTLEvaluator.h"

#include "Rules.h"
#include "RuleProvider.h"
#include <boost/foreach.hpp>
#include "AtomTableWrapper.h"
#include "BackInferenceTreeNode.h"
#include "PLNShell.h"

#include <stdlib.h>
#include <time.h>

#pragma warning(disable : 4312)

using namespace reasoning;
int currentDebugLevel;

bool RunPLNtest=true;
int tempar=0;

namespace haxx
{
	//extern multimap<Handle,Handle> childOf;
	//extern bool AllowFW_VARIABLENODESinCore;
	//extern bool ArchiveTheorems;
	//extern bool printRealAtoms;
	extern map<Handle,vector<Handle> >* inferred_from;
    //extern reasoning::iAtomTableWrapper* defaultAtomTableWrapper;
}

namespace haxx
{
	uint maxDepth = 250;
}

namespace reasoning
{
	extern bool RECORD_TRAILS;
	void footest();
	void foo_pretest();

	int varcount=0;
	int addlinks=0;
	int gethandles=0;	
}

namespace test
{
	FILE *logfile=NULL;
	int _test_count = 0;
	bool debugger_control = false;
	int attachs=0;
}

void AgentTest();

void test_core_TVs()
{
    iAtomTableWrapper* defaultAtomTableWrapper;
	Btr<set<Handle> > ts =  ((AtomTableWrapper*) defaultAtomTableWrapper)->getHandleSet((Type)77,"");
	foreach(Handle ti, *ts)
	{
		if (CogServer::getAtomSpace()->getTV(ti).isNullTv())
		{
			puts("NULL TV !!!");
			getc(stdin);
		}
		printTree(ti,0,-5);
//		getc(stdin);
	}			
}

void test_nm_reset()
{
	AtomSpace *nm = CogServer::getAtomSpace();
	//nm->Reset(NULL);
	
	//assert(nm->getHandleSet(EVALUATION_LINK,"").empty());
	//assert(nm->getHandleSet(LIST_LINK,"").empty());
}


void PLNShell_RunLoop(int argc, char** args);

/// PLNShell is intended to be used with PseudoCore. Main run loop is here.
int main(int argc, char** args)
{
	puts("PseudoCore::RunLoop");
 	PLNShell_RunLoop(argc,args);
}

void PLNShell_RunLoop(int argc, char** args)
{
	try {
		puts("Initializing PLN test env...");

#ifdef USE_PSEUDOCORE
		RunPLNtest = (argc>1 && args[1][0] == 't');

		// \todo: check the following
		if (argc>2)
			tempar = atoi(args[2]);
#endif

		RECORD_TRAILS = true;
		//haxx::printRealAtoms = true;

		currentDebugLevel=4;

		foo_pretest();

		printf("Creating AtomTableWrappers...");
		
#if LOCAL_ATW
		//haxx::defaultAtomTableWrapper = &LocalATW::getInstance();
#else
		DirectATW::getInstance();
		//haxx::defaultAtomTableWrapper = &NormalizingATW::getInstance();
#endif
		//AtomTableWrapper& TheNM = *((AtomTableWrapper*)haxx::defaultAtomTableWrapper);
				
/*		if (RunPLNtest)
		{
			reasoning::RunPLNTests();
			exit(0);
		}
*/
//		AgentTest();
		
#if 1 //Loading Osama or set axioms here.

	//haxx::ArchiveTheorems = true;
	iAtomTableWrapper* defaultAtomTableWrapper;
	AtomTableWrapper& TheNM = *((AtomTableWrapper*) defaultAtomTableWrapper);
 
  	bool axioms_ok = TheNM.LoadAxioms("bigdemo.xml");

//	bool axioms_ok = TheNM.LoadAxioms("inverse_binding.xml");

//	bool axioms_ok = TheNM.LoadAxioms("fetch10.xml");

//	bool axioms_ok = TheNM.LoadAxioms("mediumdemo.xml");
//	bool axioms_ok = TheNM.LoadAxioms("smalldemo.xml");
//	 bool axioms_ok = TheNM.LoadAxioms("smalldemo28.xml");
//	 bool axioms_ok = TheNM.LoadAxioms("smalldemo28b.xml");

 	  //bool axioms_ok = TheNM.LoadAxioms("smalldemo8.xml");
// 	  bool axioms_ok = TheNM.LoadAxioms("smalldemo8b.xml");  
//	bool axioms_ok = TheNM.LoadAxioms("smalldemo8c.xml");

// 	  bool axioms_ok = TheNM.LoadAxioms("AnotBdemo.xml");
//	  bool axioms_ok = TheNM.LoadAxioms("fetchdemo5.xml");
// 	  bool axioms_ok = TheNM.LoadAxioms("fetchdemo.xml");
//   	  bool axioms_ok = TheNM.LoadAxioms("woademo.xml");
	  assert(axioms_ok);

	  //haxx::ArchiveTheorems = false;
#endif
	  printf("PTL Initialized.");

		printf("Running footest()...\n");
		footest();
		printf("footest() complete.\n");

  } catch(std::string s)
  {
	  cout << "at root level while RunLoop initializing." << s;
  }
  catch(PLNexception e)
  {
	  cout << "at root level while RunLoop initializing." << string(e.what());
  }
  catch(...)
  {
	  printf("Unknown exception at root level while RunLoop initializing. ");
  }

	try
	{
		//RunPLNTests();
		ThePLNShell.Launch();

		return;
	}
	catch(string s) {
		printf("Exception in ThePLNShell.Launch(): %s\n", s.c_str()); }
	catch(boost::bad_get bg) {
		printf("Bad boost::get in ThePLNShell.Launch(): %s\n", bg.what()); }
	catch(...) {
		puts("Exception in ThePLNShell.Launch()."); }
  
	getc(stdin);
}


/**

StrictCrispUnification is currently buggy and cannot be used!
Anyway, if you use it, bool PREVENT_LOOPS = false must hold!
CrispUnification may be ok, but does not cover all cases!

WHen using exhaustive inference tree expansion, there may arise pathological
Rule combinations. Known examples are:

- CrispTheorem (with theorems in fetchdemo.xml) with ImplicationBreakdown
- AND Rules with ANDBreakdown
- UnorderedLinkPermutation often, esp. with ANDBreakdown / ORBreakdown
- ChildSubstitution almost always



*/

/*
void childOfDump()
{
	using namespace haxx;
	typedef pair<Handle,Handle> hpair;
	foreach(hpair crel, childOf)
	{
		cout << "#" << (int)crel.first << " is child of " << (int)crel.second << "\n";
	}
}*/

void PLNhelp();

void save_log()
{
	fclose(test::logfile);
	test::logfile=fopen("pln.log","at+");
}

map<int, Btr<vtree > > tests;

void PLNShell::Init()
{
	#if LOCAL_ATW
	//haxx::defaultAtomTableWrapper = &reasoning::LocalATW::getInstance();
	//((LocalATW*)haxx::defaultAtomTableWrapper)->SetCapacity(10000);
	#endif	
	
	#if LOG_ON_FILE
	 test::logfile=fopen("pln.log","wt");
	 cout << "LOGGING TO FILE pln.log!\n";
	#endif

	//haxx::printRealAtoms = true;
	//haxx::ArchiveTheorems = false;
}

string printTV (Handle h) {
  char str[500];
  AtomSpace *nm = CogServer::getAtomSpace();
  const TruthValue& tv = nm->getTV(h);
  if (tv.isNullTv()) 
      sprintf (str,"(TruthValue::NULL_TV())");
  else
      sprintf (str,"(%lf,%lf)",nm->getTV(h).getMean(),nm->getTV(h).getCount());
  string s(str);
  return s;
}

void printOutgoing (Handle out) {
  AtomSpace *nm = CogServer::getAtomSpace();
 vector<Handle> list=nm->getOutgoing(out);
  cout<<printTV(out);
  foreach (Handle h,list)
    cout << "<" << nm->getType(h) << "," << nm->getName(h) << ">,";
  cout<<'\n';
  foreach (Handle h,list)
    printOutgoing(h);
}

struct min_conf { 
  bool operator()(Handle h) {
	AtomSpace *nm = CogServer::getAtomSpace();
    return nm->getTV(h).getConfidence()>0.8;
  }
};

struct inhlink { 
  bool operator()(Handle h) {
	AtomSpace *nm = CogServer::getAtomSpace();
  	  return nm->inheritsType (nm->getType(h),INHERITANCE_LINK);
  }
};

void fw_beta (void) {
  //nm->Reset(NULL);
AtomSpace *nm = CogServer::getAtomSpace();
  vector<Handle> nodes=nm->filter_type(NODE);
  for (vector<Handle>::iterator i=nodes.begin(); i!=nodes.end(); i++)
    printf ("_node %d, %s, TV %s\n",nm->getType(*i),nm->getName(*i).c_str(),printTV(*i).c_str());

  vector<Handle> inhlink=nm->filter_type(INHERITANCE_LINK);
  for (vector<Handle>::iterator i=inhlink.begin(); i!=inhlink.end(); i++) {
	vector<Handle> out=nm->getOutgoing(*i);
	printf ("_link %d: %s ",nm->getType(*i),printTV(*i).c_str());
	foreach (Handle h,out)
	  cout << "<" << nm->getType(h) << "," << nm->getName(h) << ">,";
	cout<<'\n';
  }
  printf ("teste: %d\n",nm->inheritsType(INHERITANCE_LINK,LINK));
  printf ("teste: %d\n",nm->inheritsType(INHERITANCE_LINK,NODE));
  return;
  //return ;
  ForwardTestRuleProvider *rp=new ForwardTestRuleProvider();
  iAtomTableWrapper* defaultAtomTableWrapper;
  AtomTableWrapper& TheNM = *((AtomTableWrapper*) defaultAtomTableWrapper);
  SimpleTruthValue tv(0.99,SimpleTruthValue::confidenceToCount(0.99));
      Handle h1=nm->addNode (CONCEPT_NODE,"Human",tv);
      Handle h2=nm->addNode (CONCEPT_NODE,"Mortal",tv);
      Handle h3=nm->addNode (CONCEPT_NODE,"Socrates",tv);
      std::vector<Handle> p1(2),p2(2);
      p1[0]=h1; p1[1]=h2;
      p2[0]=h3; p2[1]=h1;
      Handle L1=nm->addLink(INHERITANCE_LINK,p1,tv);
      Handle L2=nm->addLink(INHERITANCE_LINK,p2,tv);


  Handle out,seed;
  do {
		Rule *r=(*rp)[ 1 ];
		Handle nextH;
		vector<Vertex> args;
		do
		{
			//int bah; cin >> bah;
			//if (args.size()>=2) args.clear();
			args.clear();
			//seed = TheNM.GetRandomHandle(INHERITANCE_LINK);
			args.push_back(L2);
			args.push_back(L1);
		}
		while (!r->validate(args));
		Vertex V=((r->compute(args)).GetValue());
		out=get<Handle>(V);
		const TruthValue& tv=nm->getTV(out);
		cout<<printTV(out)<<'\n';
		printOutgoing(out);
  } while (tv.isNullTv() || tv.getCount()<0.1);

  cout<<"Input:\n";
  printOutgoing(seed);
  cout<<"Output\n";
  printOutgoing(out);
}

void PLNShell::Launch()
{
	Init();
	Launch(NULL);
}

void PLNShell::Launch(vtree *target)
{
	//AtomTableWrapper& TheNM = *((AtomTableWrapper*)haxx::defaultAtomTableWrapper);

/*	vector<Vertex> targs, targs2;
	targs.push_back(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "A"),
					NewNode(CONCEPT_NODE, "B")
			));
	targs.push_back(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "B"),
					NewNode(CONCEPT_NODE, "C")
			));
	targs2.push_back(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "A"),
					NewNode(CONCEPT_NODE, "D")
			));
	targs2.push_back(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "B"),
					NewNode(CONCEPT_NODE, "C")
			));
*/
//	assert(RuleRepository::Instance().rule[Deduction]->validate(targs));
//	assert(!RuleRepository::Instance().rule[Deduction]->validate(targs));

	//AgentTest();

/*	tests[31] = Btr<vtree > (new vtree(
		mva((Handle)IMP,
			mva((Handle)EXECUTION_LINK, CreateVar(haxx::defaultAtomTableWrapper)),
			make_vtree(reward))));
*/
			/// Requires test/reasoning/bigdemo.xml 
AtomSpace *nm = CogServer::getAtomSpace();
int testi=0;
printf("Insert test %d\n", testi++);
			tests[0] = Btr<vtree > (new vtree(mva((Handle)AND_LINK,
					NewNode(CONCEPT_NODE, "Osama"),
					NewNode(CONCEPT_NODE, "terrorist")
			)));

printf("Insert test %d\n", testi++);
			tests[1] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "friendOf"),
					mva((Handle)LIST_LINK,
								NewNode(CONCEPT_NODE, "Amir"),
								NewNode(CONCEPT_NODE, "Osama")
							)
			)));
printf("Insert test %d\n", testi++);
			tests[2] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "wasKilled"),
					mva((Handle)LIST_LINK,
								NewNode(CONCEPT_NODE, "Osama")
							)
			)));
printf("Insert test %d\n", testi++);
			tests[3] = Btr<vtree >(new vtree(mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "friendOf"),
					mva((Handle)LIST_LINK,
								NewNode(FW_VARIABLE_NODE, "$OsamaFriend"),
								NewNode(CONCEPT_NODE, "Osama")
							)
			)));
printf("Insert test %d\n", testi++);
			tests[4] = Btr<vtree > (new vtree(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "Osama"),
					NewNode(CONCEPT_NODE, "AlQaeda")
			)));
printf("Insert test %d\n", testi++);
			tests[5] = Btr<vtree > (new vtree(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "Osama"),
					NewNode(CONCEPT_NODE, "Abu")
			)));
printf("Insert test %d\n", testi++);
			tests[6] = Btr<vtree >(new vtree(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "AlQaeda"),
					NewNode(CONCEPT_NODE, "terrorist")
			)));
printf("Insert test %d\n", testi++);
			tests[7] = Btr<vtree > (new vtree(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "Muhammad"),
					NewNode(CONCEPT_NODE, "Osama")
			)));
			tests[8] = Btr<vtree > (new vtree(mva((Handle)INHERITANCE_LINK,
					NewNode(CONCEPT_NODE, "Muhammad"),
					NewNode(CONCEPT_NODE, "terrorist")
			)));
			
			tests[9] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "killed"),
					mva((Handle)LIST_LINK,
								NewNode(FW_VARIABLE_NODE, "$killeri"),
								NewNode(CONCEPT_NODE, "Osama")
							)
			)));
			
			
			/// Maybe broken:
printf("Insert test %d\n", testi++);
			tests[10] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "+++") /*,
					mva((Handle)LIST_LINK)*/
			)));
			
			/// Requires test/reasoning/fetchdemo.xml 

			tests[11] = Btr<vtree > (new vtree(mva((Handle)IMPLICATION_LINK,
				mva((Handle)AND_LINK,
					mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "teacher_say"),
						mva((Handle)LIST_LINK,
							NewNode(WORD_NODE, "fetch")
						)
					),
					NewNode(FW_VARIABLE_NODE, "$1")					
				),
				mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "+++")
				)
			)));

			tests[12] = Btr<vtree > (new vtree(mva((Handle)IMPLICATION_LINK,
				mva((Handle)AND_LINK,
					NewNode(CONCEPT_NODE, "C"),
					NewNode(CONCEPT_NODE, "A")
				),
				mva((Handle)AND_LINK,
					NewNode(CONCEPT_NODE, "C"),
					NewNode(CONCEPT_NODE, "B")
				)
			)));

			tests[13] = Btr<vtree > (new vtree(mva((Handle)IMPLICATION_LINK,
				mva((Handle)AND_LINK,
					NewNode(CONCEPT_NODE, "A"),
					NewNode(CONCEPT_NODE, "C")
				),
				mva((Handle)AND_LINK,
					NewNode(CONCEPT_NODE, "C"),
					NewNode(CONCEPT_NODE, "B")
				)
			)));
printf("Insert test %d\n", testi++);
			tests[14] = Btr<vtree > (new vtree(mva((Handle)IMPLICATION_LINK,
				mva((Handle)AND_LINK,
					NewNode(CONCEPT_NODE, "A"),
					NewNode(CONCEPT_NODE, "C")
				),
				mva((Handle)AND_LINK,
					NewNode(CONCEPT_NODE, "B"),
					NewNode(CONCEPT_NODE, "C")
				)
			)));

			tests[15] = Btr<vtree > (new vtree(
				mva((Handle)IMPLICATION_LINK,
					NewNode(FW_VARIABLE_NODE, "$1"),
					mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "teacher_say"),
						mva((Handle)LIST_LINK,
							NewNode(WORD_NODE, "fetch")
						)					
					)
			)));

			tests[16] = Btr<vtree > (new vtree(
				mva((Handle)IMPLICATION_LINK,
					NewNode(FW_VARIABLE_NODE, "$1"),
					mva((Handle)AND_LINK,
						mva((Handle)EVALUATION_LINK,
							NewNode(PREDICATE_NODE, "teacher_say"),
							mva((Handle)LIST_LINK,
								NewNode(WORD_NODE, "fetch")
							)					
						),
						mva((Handle)EVALUATION_LINK,
							NewNode(PREDICATE_NODE, "just_done"),
							mva((Handle)LIST_LINK,
								mva((Handle)EVALUATION_LINK,
									NewNode(SCHEMA_NODE, "give"),
									mva((Handle)LIST_LINK,
										NewNode(CONCEPT_NODE, "ball"),
										NewNode(CONCEPT_NODE, "teacher")
								)
							)										
						)
					)
					)					
				)
			));
printf("Insert test %d\n", testi++);
			tests[17] = Btr<vtree > (new vtree(
				mva((Handle)IMPLICATION_LINK,
					NewNode(FW_VARIABLE_NODE, "$1"),
					mva((Handle)AND_LINK,
					mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "teacher_say"),
						mva((Handle)LIST_LINK,
							NewNode(WORD_NODE, "fetch")
						)					
					),
					mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "can_do"),
						mva((Handle)LIST_LINK,
							mva((Handle)EVALUATION_LINK,
								NewNode(SCHEMA_NODE, "walktowards"),
								mva((Handle)LIST_LINK,
									NewNode(CONCEPT_NODE, "teacher")
								)
							)										
						)
					)
					)					
				)
			));

			tests[18] = Btr<vtree > (new vtree(
					mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "just_done"),
						mva((Handle)LIST_LINK,
							mva((Handle)EVALUATION_LINK,
								NewNode(SCHEMA_NODE, "give"),
								mva((Handle)LIST_LINK,
									NewNode(CONCEPT_NODE, "ball"),
									NewNode(CONCEPT_NODE, "teacher")
								)
							)										
						)
					)
			));

			tests[19] = Btr<vtree > (new vtree(
					mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "do"),
						mva((Handle)LIST_LINK,
							mva((Handle)EVALUATION_LINK,
								NewNode(SCHEMA_NODE, "give"),
								mva((Handle)LIST_LINK,
									NewNode(CONCEPT_NODE, "ball"),
									NewNode(CONCEPT_NODE, "teacher")
								)
							)										
						)
					)
			));
			
		tests[20] = Btr<vtree > (new vtree(
				mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "+++")
				)
			));

		tests[21] = Btr<vtree > (new vtree(
				mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "test2"),
					mva((Handle)LIST_LINK,
						NewNode(CONCEPT_NODE, "Osama")
					)
				)
			));
printf("Insert test %d\n", testi++);
		tests[22] = Btr<vtree > (new vtree(
			mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "just_done"),
						mva((Handle)LIST_LINK,
							mva((Handle)EVALUATION_LINK,
								NewNode(SCHEMA_NODE, "give"),
								mva((Handle)LIST_LINK,
									NewNode(CONCEPT_NODE, "ball"),
									NewNode(CONCEPT_NODE, "teacher")
								)
							)										
						)
					)
				));

			tests[23] = Btr<vtree > (new vtree(
					mva((Handle)EVALUATION_LINK,
						NewNode(PREDICATE_NODE, "near"),
						mva((Handle)LIST_LINK,
								NewNode(CONCEPT_NODE, "teacher")
						)
					)					
				)
			);

			tests[24] = Btr<vtree > (new vtree(
					mva((Handle)SIMULTANEOUS_AND_LINK,
						NewNode(WORD_NODE, "blockword"),
						NewNode(FW_VARIABLE_NODE, "$blockword_associatee")
					)					
				)
			);

			tests[25] = Btr<vtree > (new vtree(mva((Handle)IMPLICATION_LINK,
				NewNode(FW_VARIABLE_NODE, "$1"),
				mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "+++")
				)
			)));

			tests[26] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
				NewNode(PREDICATE_NODE, "found_under"),
					mva((Handle)LIST_LINK,
						NewNode(CONCEPT_NODE, "toy_6"),
						NewNode(FW_VARIABLE_NODE, "$1")
					)
			)));
printf("Insert test %d\n", testi++);
			  tests[27] = Btr<vtree > (new vtree(mva((Handle)AND_LINK,
				mva((Handle)INHERITANCE_LINK,				
					NewNode(CONCEPT_NODE, "toy_6"),
					NewNode(CONCEPT_NODE, "toy")),
				mva((Handle)INHERITANCE_LINK,				
					NewNode(CONCEPT_NODE, "red_bucket_6"),
					NewNode(CONCEPT_NODE, "bucket")),
				mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "placed_under"),
						mva((Handle)LIST_LINK,
							NewNode(CONCEPT_NODE, "toy_6"),
							NewNode(CONCEPT_NODE, "red_bucket_6")
						)
				)
			)));

			tests[28] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
					NewNode(PREDICATE_NODE, "friendOf"),
					mva((Handle)LIST_LINK,
								NewNode(CONCEPT_NODE, "Britney"),
								NewNode(CONCEPT_NODE, "Amir")
			))));

			tests[29] = Btr<vtree > (new vtree(mva((Handle)FORALL_LINK,
				mva((Handle)LIST_LINK), //empty dummy
				mva((Handle)INHERITANCE_LINK,
					NewNode(FW_VARIABLE_NODE, "$i"),
					NewNode(CONCEPT_NODE, "terrorist")
			))));

			tests[30] = Btr<vtree > (new vtree(mva((Handle)VARIABLE_SCOPE_LINK,
				mva((Handle)LIST_LINK), //empty dummy
				mva((Handle)INHERITANCE_LINK,
					NewNode(FW_VARIABLE_NODE, "$i"),
					NewNode(CONCEPT_NODE, "terrorist")
			))));

			tests[31] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
						NewNode(CONCEPT_NODE, "Possible"),
						mva((Handle)LIST_LINK,
							NewNode(FW_VARIABLE_NODE, "$elmerist")
						)
					)));

printf("Insert tests init OK\n", testi++);
	Btr<BackInferenceTreeRootT> Bstate (new BITNodeRoot(
		(target ? meta(target) : tests[0]),
		new DefaultVariableRuleProvider));
printf("BITNodeRoot init ok\n");
	BackInferenceTreeRootT* temp_state, *state = Bstate.get();

	int a1T, a2T, bT, tempi=0;
	string a10, a11, a20, a21, b1, b2;
	vtree avt1, avt2, bvt;
	int qrule=NULL;
	Rule::MPs rule_args;
	bindingsT new_bindings;

 try {
		printf("Root = %ld\n", (long)state);
		
	while (1)
	{
		puts("(h = help):");
		char c = getc(stdin);
		char temps[1000];
		long h=0, h2=0;
        int j;
		int test_i=0;
		int s_i=0, tempi;
		bool axioms_ok;
		boost::shared_ptr<set<Handle> > ts;
		Vertex v;
		Handle eh=NULL;
		bool using_root = true;
		
		#if LOG_ON_FILE
			save_log();
		#endif
		
		//haxx::AllowFW_VARIABLENODESinCore = true; //false;
	iAtomTableWrapper* defaultAtomTableWrapper;
	AtomTableWrapper& TheNM = *((AtomTableWrapper*) defaultAtomTableWrapper);
		
		switch (c)
		{
		case 'm': printf("%d\n", test::_test_count); break;
			case 'd':
#if LOCAL_ATW
			//((LocalATW*)haxx::defaultAtomTableWrapper)->DumpCore(CONCEPT_NODE);
#else
			cin >> h;
			//ts = 
			//((AtomTableWrapper*)haxx::defaultAtomTableWrapper)->getHandleSet((Type)h,"");
			foreach(Handle ti, *ts)
			{
				if (nm->getTV(ti).isNullTv())
				{
					puts("NULL TV !!!");
					getc(stdin);
				}
				printTree(ti,0,0);
			}			
#endif
				 break;
			case 'k': state->LoopCheck(); break;
			case 'D': test::debugger_control = (test::debugger_control?false:true);
/*						for (int zz=0;zz<1000;zz++)
							RuleRepository::Instance().rule[ForAll]->o2iMeta(
								meta(new vtree(mva((Handle)EVALUATION_LINK,
								NewNode(PREDICATE_NODE, "friendOf"),
								mva((Handle)LIST_LINK,
								NewNode(CONCEPT_NODE, "Britney"),
								NewNode(CONCEPT_NODE, "Amir")
								)))));

							//nm->getHandle(NODE, "temmpo");
						puts("...");*/
						break;
			case 'a': cin >> h; state->extract_plan((Handle)h); break;
			case 'A': cin >> h;
							((BackInferenceTreeRootT*)h)->printArgs();
							
							break;
			case 'U': cin >> h;
						//((BackInferenceTreeRootT*)state->children[0].begin()->prover)
						state->print_parents((BITNode*)h);
						break;
			case 'b': cin >> h;
						printf("Target:\n");
						((BackInferenceTreeRootT*)h)->printTarget();
						printf("Results:\n");
						((BackInferenceTreeRootT*)h)->printResults();

						printf("parent arg# %d\n", ((BackInferenceTreeRootT*)h)->GetParents().begin()->parent_arg_i);

						 break;
/*			case 'B': cin >> h; cprintf(0, "Node has results & bindings:\n");
						foreach(const BoundVertex& bv, *((BackInferenceTreeRootT*)h)->direct_results)
						{
							cprintf(0,"[%d]\n", v2h(bv.value));
							if (bv.bindings)
								foreach(hpair phh, *bv.bindings)
								{
									printTree(phh.first,0,0);
									cprintf(0,"=>");
									printTree(phh.second,0,0);
								}
							}
						 break;*/
			case 'H': 
				//cin >> h; cprintf(0,"Origin Node #%d\n", state->hsource[(Handle)h]); break;
						fflush(stdin);
						cin >> bT; cin >> b1; cin >> b2;
						cin >> a1T; cin >> a10; cin >> a11;
						cin >> a2T; cin >> a20; cin >> a21;
						puts("Enter Rule #: ");
						cin >> qrule;

						bvt = (mva((Handle)bT, NewNode(CONCEPT_NODE, b1), NewNode(CONCEPT_NODE, b2)));
						avt1 = (mva((Handle)a1T, NewNode(CONCEPT_NODE, a10), NewNode(CONCEPT_NODE, a11)));
						avt2 = (mva((Handle)a2T, NewNode(CONCEPT_NODE, a20), NewNode(CONCEPT_NODE, a21)));

						rawPrint(bvt, bvt.begin(), -2);
						rawPrint(avt1, avt1.begin(), -2);
						rawPrint(avt2, avt2.begin(), -2);

						rule_args.clear();
						rule_args.push_back(BBvtree(new BoundVTree(avt1)));
						if (a2T)
							rule_args.push_back(BBvtree(new BoundVTree(avt2)));
						else
							puts("Passing 1 arg.");

						printf("BITNode %ld.", (long) state->FindNode((Rule*)qrule, meta(new vtree(bvt)), rule_args, new_bindings));

						break;
			case 'F':	tempi = currentDebugLevel;
						currentDebugLevel = 10;
						state->printFitnessPool(); break;
						currentDebugLevel = tempi;
			case 'W':	if (using_root)
							{
								temp_state = state;
								cin >> h;
								state = (BackInferenceTreeRootT*)h;

								using_root = false;
							}
							else
							{
								state = temp_state;

								using_root = true;
							}
							break;

			case 'h': PLNhelp(); break;
			case 'q': exit(0); break;
			case '=': cin >> h; cin >> h2; printf(((BackInferenceTreeRootT*)h)->eq((BackInferenceTreeRootT*)h2) ? "EQ\n" : "IN-EQ\n"); break;
			case 'p': cin >> h; printTree((Handle)h,0,0); break;
			case 'e':	try { state->evaluate(); }
						catch(string s) { printf(s.c_str()); }
						break;
			
			case 'E': cin >> h;
						if (h == 0)
							h = (long)state;
//							h = (int)state->children[0].begin()->prover;
						((BackInferenceTreeRootT*)h)->printResults();
/*						foreach(const set<BoundVertex>& eval_res_set, ((BackInferenceTreeRootT*)h)->GetEvalResults())
							foreach(const BoundVertex& eval_res, eval_res_set)
								printTree(v2h(eval_res.value),0,-10);*/
						break;
			case 'i': cin >> h;
						((BackInferenceTreeRootT*)h)->expandNextLevel();
/*						foreach(const parent_link& p, ((BackInferenceTreeRootT*)h)->GetParents())
							p.link->removeIfFailure((BackInferenceTreeRootT*)h);*/
						break;

			case 'n': state->expandNextLevel(); break;
			case 't': cin >> h; state->print_trail((Handle)h); break;
			case 'f': state->expandFittest(); break;

			case 'P': cin >> h;
						if (h == 0)
							h = (long) state->children[0].begin()->prover;
						((BackInferenceTreeRootT*)h)->print(); break;
			case 'O':	cin >> h;
//						((BITNode*)h)->PrintUsers();
						foreach(const parent_link<BITNode>& p, ((BITNode*)h)->GetParents())
							printf("User Node = %ld\n", p.link);
						break;
//			case 'l': cprintf(0,"%d\n", state->exec_pool.size()); break;
				
			case 's':	cin >> h; //Give max nr of steps that we can take.
                        j = (int) h;
				
						printf("\nTemporarily killing the log with level -3.\n");
			
						tempi = currentDebugLevel;
						currentDebugLevel = -3;
			
						state->infer(j, 0.000001f, 0.01f);
						state->printResults();
						printf("\n%d $ remaining.\n", h);
			
						currentDebugLevel = tempi;

						break;
										
			case 'S':	s_i=0;
						cin >> h;
			
						for (int k=0;k<h;k++)
							state->expandFittest();

						break;
		
			case 'r': cin >> test_i;
						Bstate.reset(new BITNodeRoot(tests[test_i], new DefaultVariableRuleProvider));
						state = Bstate.get();
						using_root = true;

						printf("Now evaluating: ");
						rawPrint(*tests[test_i],tests[test_i]->begin(),0);
						printf("\n");
						
//						state->expandNextLevel();
						break;
			case 'x': //puts("Give the XML input file name: "); 
							cin >> temps;
						//haxx::ArchiveTheorems = true; 
						//nm->Reset(NULL);
					 	  axioms_ok = TheNM.LoadAxioms(temps);
						  //haxx::ArchiveTheorems = false;
						  puts(axioms_ok ? "Input file was loaded." : "Input file was corrupt.");
			
						puts("Next you MUST (re)load a target atom with r command! Otherwise things will break.\n");

						break;
			case 'c': RECORD_TRAILS = !RECORD_TRAILS;
						printf("RECORD_TRAILS %s\n", (RECORD_TRAILS?"ON":"OFF"));
						break;
			case '-': cin >> h; currentDebugLevel = -(int)h; break;

			case 'R':
				fw_beta();
				break;
			default: c = c-'0';
					if (c >= 0 && c <= 10)
						currentDebugLevel = c;
					break;
		}
	}
 } catch( exception& e )
    {
        cout << endl << "Exception: "
             << e.what() << endl;
    }	
}

void PLNhelp()
{
	puts("Relevant shell commands:\n\
\n\
(NOTE THE DIFFERENCE BETWEEN ARG TYPES:\n\
Some commands take Handles, some take BITNodes.\n\
\n\
q - quit\n\
-3 - Minimal log level	\n\
0 - normal log level\n\
2 - recommended maximally informative log level\n\
r #n - Load in a new pre-defined target #n (from PLNShell.cc)\n\
x [path] - Load XML axiom file in 'path'\n\
s #s - Infer until result found with conf>0.01 OR 's' inference steps have been taken \n\
S #n - Execute the #n of the fittest BIT nodes\n\
i #n - Expand BITNode with id #n\n\
E #n - Print out the results of the BITnode #n  (0 = root)\n\
A #n - Print the Rule arguments of BITnode #n\n\
b #n - Print the Rule target of BITnode #n\n\
P #n - print the inference (BIT) tree under node #n (0 = root)\n\
a #h - Show the plan ie. sequence of 'do' statements pertaining to inference result Handle #h\n\
F - Show the current BIT node expansion pool sorted by heuristic fitness\n\
O #n - show the parent of BITNode #n\n\
t #h - print the inference trail for Handle #h\n\
\n\
c - switch the recording of inference trails ON/OFF (default: ON)\n\
B #i - Show the direct results (by lookup or hypothesis) of BIT node #i\n\
b #i - Show the target atom and pre-bindings of BIT node #i\n\
f - Show the current BIT node expansion pool sorted by heuristics fitness and Execute the fittest BIT node\n\
\n\
n - expand the tree's whole next level (usually not recommended)\n\
e - manually evaluate the current tree (usually not recommended)\n\
\n\
These should be bug-free, but there's no type checking of parameters, so providing eg. BIT node number instead of Handle number will SegFault.");
	
}
