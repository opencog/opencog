/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
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
- Normalization is non-trivial with fuzzy TVs. Degree of allowable loss-of-info L must be
decided (=policy), and only [TV.strength > 1.0-L] should be considered TRUE, etc.
- No Nested ForAlls => "defined($X) @ node" currently fails.
- ForAllRule's TV determines the TV of the outest link within the ForAll, not affecting
the internal ones. All atoms within ForAll should originally have confidence = 0.
- OrRule is only evaluated to the binary pair exclusion precision (A+B+C-AB-AC-BC).
- Policy: Rule.o2i methods must make sure they don't waste time. ResMan only guards the
blocking Rule method calls on a serial manner - it cannot survive a slow o2i implementation
- Always puts AndRules before SimpleAndRules, since the latter accept more general output type,
but don't do the flattening magic.
- BW chaining UnorderedLinkPermutationRule only works for AndLinks. Try it with OrLinks and die.
FW chaining should be ok. The reason is logical...
- CrispUnificationRule & UnorderedLinkPermutationRule use HYPOTHETICAL_LINKs - pseudo atoms that,
given as a parameter to the rule, make the rule output the desired kind of outcome
- Often you can choose to either increase the complexity of a Rule, or make multiple
versions of the rule (eg. AndRule<N>).
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
#include "rules/Rules.h"

#define BackInferenceTreeRootT BITNodeRoot

//#include "PLNEvaluator.h"

#include "rules/RuleProvider.h"
#include "AtomSpaceWrapper.h"
#include "BackInferenceTreeNode.h"
#include "PLNShell.h"
#include "ForwardChainer.h"

#include <opencog/util/Logger.h>
#include <opencog/adaptors/tulip/TulipWriter.h>

#include <boost/foreach.hpp>
#include <stdlib.h>
#include <time.h>

#ifdef _MSC_VER
#pragma warning(disable : 4312)
#endif // _MSC_VER

using namespace opencog::pln;
using namespace opencog;

//! debug level, @todo replaced by opencog log system
extern int currentDebugLevel;

/*
//! Run tests at start up
//! @todo move to unit test and allow it to be run from the main shell
//bool RunPLNtest=true;
bool RunPLNtest=false;
*/

namespace haxx
{
    extern multimap<Handle,Handle> childOf;
    extern bool AllowFW_VARIABLENODESinCore;
    extern bool ArchiveTheorems;
    extern bool printRealAtoms;
    extern map<Handle,vector<Handle> >* inferred_from;
    extern opencog::pln::iAtomSpaceWrapper* defaultAtomSpaceWrapper;

//    uint maxDepth = 250;
}

namespace opencog { namespace pln
{
    extern bool RECORD_TRAILS;

    int addlinks=0;
    int gethandles=0;   
}}

namespace test
{
    extern FILE *logfile;
    int _test_count = 0;
    bool debugger_control = false;
//    int attachs=0;
}

void PLNShell_RunLoop(int argc, char** args);

/// PLNShell is intended to be used with PseudoCore. Main run loop is here.
//void PseudoCore::RunLoop(int argc, char** args) const
int main(int argc, char** args)
{
    puts("PseudoCore::RunLoop");
    PLNShell_RunLoop(argc,args);
}

void PLNShell_RunLoop(int argc, char** args)
{
    try {
        puts("Initializing PLN test env...");

        // Initialise CogServer
        CogServer& cogserver = static_cast<CogServer&>(server());
        // Load builtinreqs
        //cogserver.loadModule(std::string("libbuiltinreqs.so"));
        //cogserver.loadModule(std::string("libscheme.so"));
        // Open network port
        //cogserver.enableNetworkServer();

        RECORD_TRAILS = true;
        haxx::printRealAtoms = true;

        currentDebugLevel=100;

        LOG(2, "Creating AtomSpaceWrappers...");
        
#if LOCAL_ATW
        haxx::defaultAtomSpaceWrapper = &LocalATW::getInstance();
#else
        DirectATW::getInstance();
        haxx::defaultAtomSpaceWrapper = &NormalizingATW::getInstance();
#endif
        AtomSpaceWrapper& atw = *GET_ASW;
           
/*        if (RunPLNtest)
        {
            puts("Running PLNTests...");
            opencog::pln::RunPLNTests();
            exit(0);
        }*/
//      AgentTest();
        
#if 1 //Loading Osama or set axioms here.

        haxx::ArchiveTheorems = true;
     
    //  bool axioms_ok = atw.loadAxioms("bigdemo.xml");
    //  bool axioms_ok = atw.loadAxioms("inverse_binding.xml");
    //  bool axioms_ok = atw.loadAxioms("fetch10.xml");
    //  bool axioms_ok = atw.loadAxioms("mediumdemo.xml");
        bool axioms_ok = atw.loadAxioms("smalldemo.xml");
    //  bool axioms_ok = atw.loadAxioms("smalldemo28.xml");
    //  bool axioms_ok = atw.loadAxioms("smalldemo28b.xml");
    //  bool axioms_ok = atw.loadAxioms("smalldemo8.xml");
    //  bool axioms_ok = atw.loadAxioms("smalldemo8b.xml");  
    //  bool axioms_ok = atw.loadAxioms("smalldemo8c.xml");
    //  bool axioms_ok = atw.loadAxioms("AnotBdemo.xml");
    //  bool axioms_ok = atw.loadAxioms("fetchdemo5.xml");
    //  bool axioms_ok = atw.loadAxioms("fetchdemo.xml");
    //  bool axioms_ok = atw.loadAxioms("woademo.xml");
        assert(axioms_ok);

        haxx::ArchiveTheorems = false;
#endif
        logger().debug("PLN Initialized.");
    }
    catch(std::string s) {
        logger().error("at root level while RunLoop initializing.");
    }
    catch(PLNexception e)
    {
        logger().error("at root level while RunLoop initializing.");
    }
    catch(...)
    {
        logger().error("Unknown exception at root level while RunLoop initializing. ");
        cout << "Unknown exception at root level while RunLoop initializing. "<< endl;
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
- And Rules with AndBreakdown
- UnorderedLinkPermutation often, esp. with AndBreakdown / OrBreakdown
- ChildSubstitution almost always



*/

void PLNhelp();

void save_log()
{
    fclose(test::logfile);
    test::logfile=fopen("pln.log","at+");
}

map<int, Btr<vtree > > tests;

void initTests()
{
    AtomSpaceWrapper* atw = GET_ASW;
    int testi = 0;
    tests.clear();
/*  tests[31] = Btr<vtree > (new vtree(
        mva((Handle)IMP,
            mva((Handle)EXECUTION_LINK, CreateVar(haxx::defaultAtomSpaceWrapper)),
            make_vtree(reward))));
*/
            /// Requires test/reasoning/bigdemo.xml 
printf("Insert test %d\n", testi++);
            tests[0] = Btr<vtree > (new vtree(mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "Osama"),
                    NewNode(CONCEPT_NODE, "terrorist")
            )));

printf("Insert test %d\n", testi++);
            tests[1] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "Amir"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )));
printf("Insert test %d\n", testi++);
            tests[2] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
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
printf("Insert test %d\n", testi++);
            tests[8] = Btr<vtree > (new vtree(mva((Handle)INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Muhammad"),
                    NewNode(CONCEPT_NODE, "terrorist")
            )));
printf("Insert test %d\n", testi++);
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
printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
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
            
printf("Insert test %d\n", testi++);
        tests[20] = Btr<vtree > (new vtree(
                mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
                )
            ));

printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
            tests[23] = Btr<vtree > (new vtree(
                    mva((Handle)EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "near"),
                        mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "teacher")
                        )
                    )                   
                )
            );

printf("Insert test %d\n", testi++);
            tests[24] = Btr<vtree > (new vtree(
                    mva((Handle)SIMULTANEOUS_AND_LINK,
                        NewNode(WORD_NODE, "blockword"),
                        NewNode(FW_VARIABLE_NODE, "$blockword_associatee")
                    )                   
                )
            );

printf("Insert test %d\n", testi++);
            tests[25] = Btr<vtree > (new vtree(mva((Handle)IMPLICATION_LINK,
                NewNode(FW_VARIABLE_NODE, "$1"),
                mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
                )
            )));

printf("Insert test %d\n", testi++);
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

printf("Insert test %d\n", testi++);
            tests[28] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "Britney"),
                                NewNode(CONCEPT_NODE, "Amir")
            ))));

printf("Insert test %d\n", testi++);
            tests[29] = Btr<vtree > (new vtree(mva((Handle)FORALL_LINK,
                mva((Handle)LIST_LINK), //empty dummy
                mva((Handle)INHERITANCE_LINK,
                    NewNode(FW_VARIABLE_NODE, "$i"),
                    NewNode(CONCEPT_NODE, "terrorist")
            ))));

printf("Insert test %d\n", testi++);
            tests[30] = Btr<vtree > (new vtree(mva((Handle)BIND_LINK,
                mva((Handle)LIST_LINK), //empty dummy
                mva((Handle)INHERITANCE_LINK,
                    NewNode(FW_VARIABLE_NODE, "$i"),
                    NewNode(CONCEPT_NODE, "terrorist")
            ))));

printf("Insert test %d\n", testi++);
            tests[31] = Btr<vtree > (new vtree(mva((Handle)EVALUATION_LINK,
                        NewNode(CONCEPT_NODE, "Possible"),
                        mva((Handle)LIST_LINK,
                            NewNode(FW_VARIABLE_NODE, "$elmerist")
                        )
                    )));

printf("Insert tests init\n");
}

void PLNShell::Init()
{
    #if LOCAL_ATW
    haxx::defaultAtomSpaceWrapper = &opencog::pln::LocalATW::getInstance();
    ((LocalATW*)haxx::defaultAtomSpaceWrapper)->SetCapacity(10000);
    #endif  
    
    #if LOG_ON_FILE
     test::logfile=fopen("pln.log","wt");
     cout << "LOGGING TO FILE pln.log!\n";
    #endif

    haxx::printRealAtoms = true;
    haxx::ArchiveTheorems = false;
}

string printTV (Handle h) {
  char str[500];
  AtomSpaceWrapper *atw = GET_ASW;
  const TruthValue& tv = atw->getTV(h);
  if (tv.isNullTv()) 
      sprintf (str,"(TruthValue::NULL_TV())");
  else
      sprintf (str,"(%f,%f)",atw->getTV(h).getMean(),atw->getTV(h).getCount());
  string s(str);
  return s;
}

void printOutgoing (Handle out) {
  AtomSpaceWrapper *atw = GET_ASW;
 vector<Handle> list=atw->getOutgoing(out);
  cout<<printTV(out);
  foreach (Handle h,list)
    cout << "<" << atw->getType(h) << "," << atw->getName(h) << ">,";
  cout<<'\n';
  foreach (Handle h,list)
    printOutgoing(h);
}

struct min_conf { 
  bool operator()(Handle h) {
      AtomSpaceWrapper *atw = GET_ASW;
      return atw->getTV(h).getConfidence()>0.8;
  }
};

struct inhlink { 
  bool operator()(Handle h) {
      AtomSpaceWrapper *atw = GET_ASW;
      return atw->inheritsType (atw->getType(h),INHERITANCE_LINK);
  }
};

//! Used by forward chainer 
struct compareStrength {
    // Warning, uses fake atomspace handles in comparison
    bool operator()(const Handle& a, const Handle& b) {
        return GET_ASW->getTV(a).getConfidence() >
            GET_ASW->getTV(b).getConfidence();
    }
};

void fw_beta (void) {
  AtomSpaceWrapper *atw = GET_ASW;

  atw->reset();

  //ForwardComposerRuleProvider *rp=new ForwardComposerRuleProvider();
  SimpleTruthValue tv(0.99,SimpleTruthValue::confidenceToCount(0.99));
#if 0
  Handle h1=atw->addNode (CONCEPT_NODE,string("Human"),tv,true);
  Handle h2=atw->addNode (CONCEPT_NODE,string("Mortal"),tv,true);
  Handle h3=atw->addNode (CONCEPT_NODE,string("Socrates"),tv,true);
  std::vector<Handle> p1(2),p2(2);
  p1[0]=h1; p1[1]=h2;
  p2[0]=h3; p2[1]=h1;
  Handle L1=atw->addLink(ASSOCIATIVE_LINK,p1,tv,true);
  Handle L2=atw->addLink(ASSOCIATIVE_LINK,p2,tv,true);
#endif

  ForwardChainer fw;

  // Load data from xml wordpairs file
  atw->reset();
  bool axioms_ok = atw->loadAxioms(std::string("wordpairs.xml"));
  if (!axioms_ok) {
      cout << "load failed" <<endl;
      exit(1);
  }
  // Remove the dummy list link (added because the xml loader is silly)
  shared_ptr<set<Handle> > ll = atw->getHandleSet(LIST_LINK, "", false);
  foreach(Handle l, *ll) {
      atw->removeAtom(l);
  }


  cout << "FWBETA Adding handles to seed stack" << endl;
  //fw.seedStack.push_back(L1);
  //fw.seedStack.push_back(L2);
  //
// Push all links to seed stack in order of strength
// . get links
    shared_ptr<set<Handle> > linksSet = atw->getHandleSet(LINK, "", true);
    HandleSeq links;
    copy(linksSet->begin(), linksSet->end(), back_inserter(links));
// . sort links based on strength
    std::sort(links.begin(), links.end(), compareStrength());
// . add in order
    foreach(Handle l, links) {
        fw.seedStack.push_back(l);
    }
// Change prob of non seed stack selection to zero
//! @todo make set method so it will normalise probabilities.
  fw.probGlobal = 0.0f;
  fw.probStack = 1.0f;

  cout << "FWBETA adding to seed stack finished" << endl;
  HandleSeq results = fw.fwdChainStack(10000);
  //opencog::logger().info("Finish chaining on seed stack");
  cout << "FWBETA Chaining on seed stack finished, results:" << endl;
  NMPrinter np;
  foreach (Handle h, results) {
      np(h);
  }
  cout << "Removing results that just repeat existing links" << endl;
  int totalSize = results.size();
  HandleSeq::iterator i,j;
  for (i = results.begin(); i != results.end(); i++) {
      vhpair v = atw->fakeToRealHandle(*i);
      if (! (v.second == NULL_VERSION_HANDLE)) {
          j = i;
          i--;
          results.erase(j);
      }
  }
  cout << "Removed " << totalSize - results.size() << " results that just " <<
      "repeat existing links, " << results.size() << " results left." << endl;
  Handle setLink = atw->addLinkDC(SET_LINK, results, tv, false, false);

  TulipWriter tlp(std::string("fwd_chain_result.tlp"));
  tlp.write(Handle::UNDEFINED,-1,atw->fakeToRealHandle(setLink).first);
  
  
}

void PLNShell::Launch()
{
    Init();
    initTests();
    Launch(NULL);
}

void PLNShell::Launch(vtree *target)
{
    AtomSpaceWrapper* atw = GET_ASW;

/*  VertexSeq targs, targs2;
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
//  assert(RuleRepository::Instance().rule[Deduction]->validate(targs));
//  assert(!RuleRepository::Instance().rule[Deduction]->validate(targs));

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
        Handle eh=Handle::UNDEFINED;
        bool using_root = true;
        
        #if LOG_ON_FILE
            save_log();
        #endif
        
        haxx::AllowFW_VARIABLENODESinCore = true; //false;
        
        switch (c)
        {
            case 'm':
                printf("%d\n", test::_test_count); break;
            case 'd':
#if LOCAL_ATW
            ((LocalATW*)atw)->DumpCore(CONCEPT_NODE);
#else
            cin >> h;
            ts = atw->getHandleSet((Type)h,"");
            foreach(Handle ti, *ts)
            {
                if (atw->getTV(ti).isNullTv())
                {
                    puts("NULL TV !!!");
                    getc(stdin);
                }
                printTree(ti,0,0);
            }           
#endif
                 break;
            case 'k': state->loopCheck(); break;
            case 'D': test::debugger_control = (test::debugger_control?false:true);
/*                      for (int zz=0;zz<1000;zz++)
                            RuleRepository::Instance().rule[ForAll]->o2iMeta(
                                meta(new vtree(mva((Handle)EVALUATION_LINK,
                                NewNode(PREDICATE_NODE, "friendOf"),
                                mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "Britney"),
                                NewNode(CONCEPT_NODE, "Amir")
                                )))));

                            //atw->getHandle(NODE, "temmpo");
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
                        cprintf(-10, "Target:\n");
                        ((BackInferenceTreeRootT*)h)->printTarget();
                        cprintf(-10, "Results:\n");
                        ((BackInferenceTreeRootT*)h)->printResults();

                        cprintf(0, "parent arg# %d\n", ((BackInferenceTreeRootT*)h)->getParents().begin()->parent_arg_i);

                         break;
/*          case 'B': cin >> h; cprintf(0, "Node has results & bindings:\n");
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

                        printf("BITNode %ld.", (long) state->findNode((Rule*)qrule, meta(new vtree(bvt)), rule_args, new_bindings));

                        break;
            case 'F':   tempi = currentDebugLevel;
                        currentDebugLevel = 10;
                        state->printFitnessPool(); break;
                        currentDebugLevel = tempi;
            case 'W':   if (using_root)
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
            case '=': cin >> h; cin >> h2; cprintf(0, ((BackInferenceTreeRootT*)h)->eq((BackInferenceTreeRootT*)h2) ? "EQ\n" : "IN-EQ\n"); break;
            case 'p': cin >> h; printTree((Handle)h,0,0); break;
            case 'e':   try { state->evaluate(); }
                        catch(string s) { cprintf(0,s.c_str()); }
                        break;
            
            case 'E': cin >> h;
                        if (h == 0)
                            h = (long)state;
//                          h = (int)state->children[0].begin()->prover;
                        ((BackInferenceTreeRootT*)h)->printResults();
/*                      foreach(const set<BoundVertex>& eval_res_set, ((BackInferenceTreeRootT*)h)->GetEvalResults())
                            foreach(const BoundVertex& eval_res, eval_res_set)
                                printTree(v2h(eval_res.value),0,-10);*/
                        break;
            case 'i': cin >> h;
                        ((BackInferenceTreeRootT*)h)->expandNextLevel();
/*                      foreach(const parent_link& p, ((BackInferenceTreeRootT*)h)->GetParents())
                            p.link->removeIfFailure((BackInferenceTreeRootT*)h);*/
                        break;

            case 'n': state->expandNextLevel(); break;
            case 't': cin >> h; state->printTrail((Handle)h); break;
            case 'f': state->expandFittest(); break;

            case 'P': cin >> h;
                        if (h == 0)
                            h = (long) state->children[0].begin()->prover;
                        ((BackInferenceTreeRootT*)h)->print(); break;
            case 'O':   cin >> h;
//                      ((BITNode*)h)->PrintUsers();
                        foreach(const parent_link<BITNode>& p, ((BITNode*)h)->getParents())
                            cprintf(-10,"User Node = %lu\n", (ulong) p.link);
                        break;
//          case 'l': cprintf(0,"%d\n", state->exec_pool.size()); break;
                
            case 's':   cin >> h; //Give max nr of steps that we can take.
                        j = (long) h;
                
                        //printf("\nTemporarily killing the log with level -3.\n");
                        //tempi = currentDebugLevel;
                        //currentDebugLevel = -3;
            
                        state->infer(j, 0.000001f, 0.01f);
                        state->printResults();
                        printf("\n%ld $ remaining.\n", h);
            
                        //currentDebugLevel = tempi;

                        break;
                                        
            case 'S':   s_i=0;
                        cin >> h;
            
                        for (int k=0;k<h;k++)
                            state->expandFittest();

                        break;
        
            case 'r': cin >> test_i;
                        Bstate.reset(new BITNodeRoot(tests[test_i], new DefaultVariableRuleProvider));
                        state = Bstate.get();
                        using_root = true;

                        cprintf(0,"Now evaluating: ");
                        rawPrint(*tests[test_i],tests[test_i]->begin(),0);
                        cprintf(0,"\n");
                        
//                      state->expandNextLevel();
                        break;
            case 'x': //puts("Give the XML input file name: "); 
                            cin >> temps;
                        haxx::ArchiveTheorems = true; 
                        atw->reset();
                        axioms_ok = atw->loadAxioms(temps);
                        haxx::ArchiveTheorems = false;
                        puts(axioms_ok ? "Input file was loaded." : "Input file was corrupt.");
            
                        puts("Next you MUST (re)load a target atom with r command! Otherwise things will break.\n");
                        // have to recreate the target vtrees to ensure that the
                        // handles are correct after reloading axioms.
                        initTests();

                        break;
            case 'c': RECORD_TRAILS = !RECORD_TRAILS;
                        cprintf(0, "RECORD_TRAILS %s\n", (RECORD_TRAILS?"ON":"OFF"));
                        break;
            case '-': cin >> h; currentDebugLevel = -(int)h; break;

            case 'R':
                int testdone;
                atw->testAtomSpaceWrapper();
                cin >> testdone;
                fw_beta();
                break;
            case 'Y':
                // Temporary for loading data via telnet
                //cout << "running server loop" << endl;
                //static_cast<CogServer&>(server()).unitTestServerLoop(1);
                //cout << "finished server loop" << endl;
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
-3 - Minimal log level  \n\
0 - normal log level\n\
2 - recommended maximally informative log level\n\
r #n - Load in a new pre-defined target #n (from PLNShell.cc)\n\
x [path] - Load XML axiom file in 'path'\n\
s #s - Infer until result found with conf>0.01 Or 's' inference steps have been taken \n\
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
