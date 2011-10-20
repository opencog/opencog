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

#include "PLN.h"
#include "rules/Rules.h"

#define BackInferenceTreeRootT BITNodeRoot

#include "AtomSpaceWrapper.h"
#include "BackInferenceTreeNode.h"
#include "PLNShell.h"

#include "rules/RuleApp.h"

#include <stdlib.h>
#include <time.h>
#ifndef WIN32
    #include <sys/time.h>
#endif

#include "rules/RuleProvider.h"

#include <opencog/adaptors/tulip/TulipWriter.h>

using namespace opencog::pln;  

#if 0
#include <windows.h>
#include <Mmsystem.h>

#if defined(_MSC_VER) || defined(__MINGW32__)
#  include <time.h>
#ifndef _TIMEVAL_DEFINED /* also in winsock[2].h */
#define _TIMEVAL_DEFINED
struct timeval {
    long tv_sec;
    long tv_usec;
};
#endif /* _TIMEVAL_DEFINED */
#else
#  include <sys/time.h>
#endif

#if defined(_MSC_VER) || defined(__MINGW32__)
int gettimeofday(struct timeval* tp, void* tzp) {
    unsigned long t;
    t = timeGetTime();
    tp->tv_sec = t / 1000;
    tp->tv_usec = t % 1000;
    /* 0 indicates that the call succeeded. */
    return 0;
}
#endif
#endif

namespace test
{
    extern FILE *logfile;
    extern double custom_duration;
    extern double custom_duration2;
}

enum FitnessEvalutorT { DETERMINISTIC, RANDOM, SOFTMAX };

namespace opencog { namespace pln {
    void InitAxiomSet(string premiseFile);
    extern FitnessEvalutorT FitnessEvaluator;
    extern bool RECORD_TRAILS;
    extern unsigned long now_interval_len   ;
}}

// These are now in PLNUtils.h
//#define NewNode(_T, _NAME) mva(nm->addNode(_T, _NAME, TruthValue::TRIVIAL_TV(), false,false))
//#define makemeta(atom_description) meta(new tree<Vertex>(atom_description))

#define maketest(test_description,a,b,c,d) RunPLNTest(Btr<PLNTest>(new PLNTest(test_description,a,b,c,d)))

float getCount(float c)
{
    return SimpleTruthValue::confidenceToCount(c);
//    float KKK = IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL;
//    return -KKK*c/(c-1);
}

namespace haxx
{
    extern opencog::pln::iAtomSpaceWrapper* defaultAtomSpaceWrapper;
    extern bool printRealAtoms;
    extern Handle VarTypes[STD_VARS];
    extern multimap<Handle,Handle> childOf;
    extern bool AllowFW_VARIABLENODESinCore;
    extern bool ArchiveTheorems;
    extern bool printRealAtoms;
}

vector< vector<vector<int> > >  INstatsVT;
vector<vector<int> > INstatsV;
vector<int> INstats;
static int AllTestsInferenceNodes=0;

namespace opencog { namespace pln
{
    void foo_pretest() {}
    void footest() {}
    extern map<int, Btr<tree<Vertex> > > tests;

        /**
        The tests
load in various xml axiom sets and run until the target with a
specific confidence minimum has been reached (or max. nr of steps have
been exceeded, resulting in failure.)
        */
        
struct PLNTest
{
    meta target;
    TruthValue* minTV, *maxTV;

    uint minEvalsOfFittestBIT; //Divided by ten
    uint minExhaustiveEvals; //Use either this one or the prev one
    PLNTest(    meta _target,
                TruthValue* _minTV,
                TruthValue* _maxTV,
                uint _minEvalsOfFittestBIT,
                uint _minExhaustiveEvals)
        : target(_target), minTV(_minTV), maxTV(_maxTV),
        minEvalsOfFittestBIT(_minEvalsOfFittestBIT),
        minExhaustiveEvals(_minExhaustiveEvals)
    {}
};

void RunPLNTest(Btr<PLNTest> t);

void finger_print_test(vtree& v)
{
    BoundVTree bvt(v);
    printf("Finger print: %lu\n", bvt.getFingerPrint());
}

set<Btr<PLNTest> > PLNTests;

}} // namespace opencog::pln

namespace goal
{
    void WalkTest();
}

namespace opencog { namespace pln
{

bool foo42=false;

void InitPLNTests()
{
#if LOG_ON_FILE
    test::logfile=fopen("pln.log","wt");
    cout << "LOGGING TO FILE pln.log!\n";
#endif

    //assert(finger_print_test(*tests[0]) != finger_print_test(*tests[1]));
    haxx::printRealAtoms = true;
    haxx::ArchiveTheorems = true;

/*  /// Test atom identity works
    Handle h1 = haxx::defaultAtomSpaceWrapper->addNode(FW_VARIABLE_NODE, "filler",new SimpleTruthValue(0.001f,1.0f), false, false);
    Handle h2 = haxx::defaultAtomSpaceWrapper->addNode(FW_VARIABLE_NODE, "filler",new SimpleTruthValue(0.001f,1.0f), false, false);

    assert(h1 == h2);
*/
    currentDebugLevel=100;
}

bool satSetTest();

#define RUN_FAILURE_TESTS 0

void RunPLNTestsOnce();

float temperature = 0.1f;
float temperatures[] = {    0.00005, 0.00007, 10,
                            0.0001, 0.0003, 0.0005,
                            0.001, 0.003, 0.005,
                            0.01, 0.03, 0.05,
                            0.1, 0.3, 0.5,
                            1, 3, 5 };
const int temperaturesN = 7; //3*5;

void RunPLNTests()
{
//  goal::WalkTest();
//  return;
    
//  satSetTest();

    InitPLNTests();

    /// You can run the tests multiple times, which makes sense if
    /// the heuristics function is not deterministic (eg. SoftMax)

    const int TestRepeats = 1;
    int seed = 11111;

    FILE *f = fopen("results.txt","a");

    int tempi=0;

    unifiesWithVariableChangeTo_TEST();
//  for (int tempi = 0; tempi < temperaturesN; tempi++)
    {
        INstatsV.clear();

        temperature = temperatures[tempi];

        for (int t=0; t < TestRepeats; t++)
        {
            //      time_t seconds;
            //      time(&seconds);
            int seconds = 1;
            srand((unsigned int) seconds+(seed++));
            AllTestsInferenceNodes = 0;

            RunPLNTestsOnce();
            INstatsV[t].push_back(AllTestsInferenceNodes);

            if (FitnessEvaluator != SOFTMAX)
            {
                //No need to play with temperatures or multiple test repeats.
                if (FitnessEvaluator != RANDOM)
                    t = TestRepeats;
                tempi = temperaturesN;
            }
        }
        INstatsVT.push_back(INstatsV);
    }

    for (uint v=0; v < INstatsV[0].size(); v++)
    {
        fprintf(f, "\n");
        for (int t=0; t < ( (FitnessEvaluator!=DETERMINISTIC) ? TestRepeats : 1); t++)
            fprintf(f, "%d ", INstatsV[t][v]);
    }

    fclose(f);

    cout << "\n\nTests complete.\n";
}

void MacroRuleTest()
{
    AtomSpaceWrapper *atw = GET_ASW;
    //typedef InversionRule RuleT1;
    //typedef DeductionRule RuleT2;
    iAtomSpaceWrapper* parent = ::haxx::defaultAtomSpaceWrapper;

    InversionRule<INHERITANCE_LINK> *invR = new InversionRule<INHERITANCE_LINK>(parent);
    DeductionRule<DeductionSimpleFormula, INHERITANCE_LINK> *deduR = new DeductionRule<DeductionSimpleFormula, INHERITANCE_LINK>(parent);

    printf("v0\n");
    vtree v0(mva((Handle)INHERITANCE_LINK,
                    mva(atw->addNode(CONCEPT_NODE, "Abu", SimpleTruthValue(0.05, 0.01))),
                    mva(atw->addNode(CONCEPT_NODE, "Osama",  SimpleTruthValue(0.01, 0.01)))
                    ));
    printf("v1\n");
    vtree v1(mva((Handle)INHERITANCE_LINK,
                    mva(atw->addNode(CONCEPT_NODE, "Osama",  SimpleTruthValue(0.01, 0.01))),
                    mva(atw->addNode(CONCEPT_NODE, "AlQaeda",  SimpleTruthValue(0.1, 0.01)))
                    ));
    printf("v2\n");
    vtree v2(mva((Handle)INHERITANCE_LINK,
                    mva(atw->addNode(CONCEPT_NODE, "AlQaeda",  SimpleTruthValue(0.1, 0.01))),
                    mva(atw->addNode(CONCEPT_NODE, "terrorist",  SimpleTruthValue(0.2, 0.01)))));

    printf("h0\n");
    Handle h0 = atw->addAtom(v0, 
            SimpleTruthValue(0.40f, getCount(0.80f)));
    printf("h1\n");
    Handle h1 = atw->addAtom(v1,
            SimpleTruthValue(0.60f, getCount(0.90f)));
    printf("h2\n");
    Handle h2 = atw->addAtom(v2,
            SimpleTruthValue(0.98f, getCount(0.95f)));

    RuleApp* top    = new RuleApp(deduR);
    RuleApp* child1a= new RuleApp(deduR);

    child1a->Bind(0, new VtreeProviderWrapper(Vertex(h0)));
    child1a->Bind(1, new VtreeProviderWrapper(Vertex(h1)));
    child1a->compute();

    top->Bind(0, child1a);
    top->Bind(1, new VtreeProviderWrapper(Vertex(h2)));

    BoundVertex res1a = top->compute();
    const TruthValue& tvA = atw->getTV(v2h(res1a.value));
    assert(tvA.getMean() > 0.01);
    assert(tvA.getConfidence() > 0.01);

    RuleApp* topb   = new RuleApp(deduR);
//  RuleApp* child1b= vtree(Vertex(h0));
    RuleApp* child2b= new RuleApp(deduR);

    child2b->Bind(0, new VtreeProviderWrapper(Vertex(h1)));
    child2b->Bind(1, new VtreeProviderWrapper(Vertex(h2)));
    topb->Bind(0, new VtreeProviderWrapper(Vertex(h0)));
    topb->Bind(1, child2b);

    BoundVertex res1b = topb->compute();
    assert(res1a.value == res1b.value);

    const TruthValue& tvB = atw->getTV(v2h(res1a.value));
    assert(within(tvB.getMean(), tvA.getMean(), 0.001));
    assert(within(tvB.getConfidence(), tvA.getConfidence(), 0.001));

    RuleApp* top2 = new RuleApp(invR);
    top2->Bind(0, topb);
    RuleApp* top3 = new RuleApp(invR);
    top3->Bind(0, top2);

    const TruthValue& tv2 = atw->getTV(v2h(top2->compute().value));
    assert(!within(tvB.getMean(), tv2.getMean(), 0.001));
//  assert(!within(tvB.getConfidence(), tv2.getConfidence(), 0.001));

    const TruthValue& tv3 = atw->getTV(v2h(top3->compute().value));
    assert( within(tvB.getMean(), tv3.getMean(), 0.001));
    assert( within(tvB.getConfidence(), tv3.getConfidence(), 0.001));


    vector<VtreeProvider*> args;
    args.push_back(new VtreeProviderWrapper(Vertex(h0)));
    args.push_back(new VtreeProviderWrapper(Vertex(h1)));
    args.push_back(new VtreeProviderWrapper(Vertex(h2)));

    RuleApp* toac   = new RuleApp(deduR);
    RuleApp* child2c= new RuleApp(deduR);

    toac->Bind(1, child2c);

    BoundVertex resC = toac->compute(args.begin(), args.end());

    assert(resC.value == res1b.value);
    const TruthValue& tvC = atw->getTV(v2h(resC.value));
    assert( within(tvB.getMean(), tvC.getMean(), 0.001));
    assert( within(tvB.getConfidence(), tvC.getConfidence(), 0.001));


    RuleApp* topd   = new RuleApp(deduR);
    RuleApp* child1d= new RuleApp(deduR);

    topd->Bind(0, child1d);

    BoundVertex resD = topd->compute(args.begin(), args.end());

    assert(resD.value == res1b.value);
    const TruthValue& tvD = atw->getTV(v2h(resD.value));
    assert( within(tvB.getMean(), tvD.getMean(), 0.001));
    assert( within(tvB.getConfidence(), tvD.getConfidence(), 0.001));
    printf("finish MacroRuleTest\n");
}

void RunPLNTestsOnce()
{
    AtomSpaceWrapper *atw = GET_ASW;
    INstats.clear();

    puts("Starting PLN tests. NOTE! 3 first tests are supposed to fail.");

    //MacroRuleTest();
    puts("Testing atomspacewrapper");
    atw->testAtomSpaceWrapper();


#if RUN_FAILURE_TESTS
    /// The test which is supposed to fail

    InitAxiomSet("bigdemo.xml");

    puts("\nShould fail. The test for whether TV checks work.\n");

    maketest(makemeta(mva((Handle)INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "AlQaeda"),
                    NewNode(CONCEPT_NODE, "terrorist")
            )),
            new SimpleTruthValue(0.95f, getCount(0.90f)),
            new SimpleTruthValue(0.999f, getCount(0.999f)),
            50,0);

    printf("maketest 2\n");
    //char *buf = new char[8174+2]; 
InitAxiomSet("smalldemo.xml");

    puts("\nShould fail. The test for whether random TVs come out too high:\n");

    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "killed"),
                    mva((Handle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$killeri"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )),
            new SimpleTruthValue(0.5f, getCount(0.5f)),
            new SimpleTruthValue(0.51f, getCount(0.51f)),
            10,0);
    
    puts("\nShould fail.");

    InitAxiomSet("smalldemo.xml");
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "symmetricRelation"),
                    mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "Amir"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )),
            new SimpleTruthValue(0.0f, getCount(0.0f)),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            10,0);

#endif
    puts("\nInverse Binding test\n");
    InitAxiomSet("inverse_binding.xml");
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "Possible"),
                        mva((Handle)LIST_LINK,
                            NewNode(FW_VARIABLE_NODE, "$elmerist")
                        )
                    )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.0f, getCount(1.0f)), // was conf 1.01???
            200,0);

    for (int i = 0; i < 5; i++)
    {
        printf("\nBasic spawning test %d\n", i);
        /// Basic spawning test
        InitAxiomSet("smalldemo.xml");
        maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "Britney"),
                                NewNode(CONCEPT_NODE, "Amir")
                            ))),
            new SimpleTruthValue(0.78f, getCount(0.39f)),
            new SimpleTruthValue(1.001f, getCount(0.999f)),
            100,0);
    }
    
    printf("\nTest for multiple roots spawning.\n");
    /// Test for multiple roots spawning
    InitAxiomSet("smalldemo.xml");
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "killed"),
                    mva((Handle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$killeri"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )),
            new SimpleTruthValue(0.5f, getCount(0.5f)),
            new SimpleTruthValue(0.999f, getCount(0.999f)),
        10,0);

    printf("\nTest Generalization for BIND_LINK.\n");
    /// Test Generalization for BIND_LINK
    InitAxiomSet("smalldemo.xml");
    maketest(makemeta(mva((Handle)BIND_LINK,
                mva((Handle)LIST_LINK), //empty dummy
                mva((Handle)INHERITANCE_LINK,
                    NewNode(FW_VARIABLE_NODE, "$i"),
                    NewNode(CONCEPT_NODE, "terrorist")
            ))),
            new SimpleTruthValue(0.9f, getCount(0.02f)),
            new SimpleTruthValue(0.999f, getCount(0.999f)),
            10,0);

    printf("\nTest Generalization for FORALL_LINK.\n");
    /// Test Generalization for FORALL_LINK
    InitAxiomSet("smalldemo.xml");
    maketest(makemeta(mva((Handle)FORALL_LINK,
                mva((Handle)LIST_LINK), //empty dummy
                mva((Handle)INHERITANCE_LINK,
                    NewNode(FW_VARIABLE_NODE, "$i"),
                    NewNode(CONCEPT_NODE, "terrorist")
            ))),
            new SimpleTruthValue(0.9f, getCount(0.9f)),
            new SimpleTruthValue(0.999f, getCount(0.999f)),
            15,0);
    //TulipWriter tlp(std::string("small_demo.tlp"));
    //tlp.write(0,0);//,atw->fakeToRealHandle(setLink).first);

    printf("\nTest inheritance Osama/Abu.\n");
    InitAxiomSet("smalldemo.xml");
    maketest(makemeta(mva((Handle)INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Osama"),
                    NewNode(CONCEPT_NODE, "Abu")
            )),
            new SimpleTruthValue(0.0001f, getCount(0.90f)),
            new SimpleTruthValue(0.999f, getCount(1.01f)),
            40,0);

    printf("\nTest inheritance Muhummad->Terrorist.\n");
// Takes a tad too long with bigdemo (but tested, and it works now)
    InitAxiomSet("smalldemo.xml");
//    InitAxiomSet("bigdemo.xml");
    maketest(makemeta(mva((Handle)INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Muhammad"),
                    NewNode(CONCEPT_NODE, "terrorist")
            )),
            new SimpleTruthValue(0.01f, getCount(0.20f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            460,0);
    //TulipWriter tlp2(std::string("big_demo.tlp"));
    //tlp2.write(0,0);//,atw->fakeToRealHandle(setLink).first);

    //foo42=true;
    printf("\nTest fetch demo.\n");
    InitAxiomSet("fetchdemo5.xml");
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
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
            ),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            200,0);

    InitAxiomSet("AnotBdemo.xml");

    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                NewNode(PREDICATE_NODE, "found_under"),
                    mva((Handle)LIST_LINK,
                        NewNode(CONCEPT_NODE, "toy_6"),
                        NewNode(FW_VARIABLE_NODE, "$1")
                    )
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            10000,0);
/*
    maketest(makemeta(mva(
            )),
            new SimpleTruthValue(0.01f, getCount(0.90f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "bigdemo.xml",
            100);
*/

    InitAxiomSet("fetchdemo5.xml");
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
                )
            ),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(0.94f)),
            200,0);
            
/*
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
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
                    ))),
            new SimpleTruthValue(0.01f, getCount(0.90f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "fetchdemo4.xml",
            10,0);
*/

//  InitAxiomSet("smalldemo.xml");
    InitAxiomSet("woademo.xml");
    maketest(makemeta(mva((Handle)SIMULTANEOUS_AND_LINK,
                        NewNode(WORD_NODE, "blockword"),
                        NewNode(FW_VARIABLE_NODE, "$blockword_associatee")
                    )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            500,0);

/*  maketest(makemeta(mva(
            )),
            new SimpleTruthValue(0.01f, getCount(0.90f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "bigdemo.xml",
            100);*/



/*  /// Untested with current PLN implementation:

    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "Amir"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "bigdemo.xml",
            10,0);

    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "wasKilled"),
                    mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "bigdemo.xml",
            10,0);
*/
/*      InitAxiomSet("bigdemo.xml");
        maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$OsamaFriend"),
                                NewNode(CONCEPT_NODE, "Osama")
                            ))),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            10,0);*/
/*  maketest(makemeta(mva((Handle)INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Osama"),
                    NewNode(CONCEPT_NODE, "AlQaeda")
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "bigdemo.xml",
            10,0);
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "killed"),
                    mva((Handle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$killeri"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "bigdemo.xml",
            10,0);
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
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
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "fetchdemo4.xml",
            10,0);
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "fetchdemo4.xml",
            10,0);
    maketest(makemeta(mva((Handle)EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "near"),
                        mva((Handle)LIST_LINK,
                                NewNode(CONCEPT_NODE, "teacher")
                        )
                    )
            ),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "fetchdemo4.xml",
            10,0);
    maketest(makemeta(mva((Handle)IMPLICATION_LINK,
                NewNode(FW_VARIABLE_NODE, "$1"),
                mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
                )
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "fetchdemo4.xml",
            10,0);
*/
/*  maketest(makemeta(mva(
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "fetchdemo4.xml",
            10,0);
    maketest(makemeta(mva(
            )),
            new SimpleTruthValue(0.01f, getCount(0.01f)),
            new SimpleTruthValue(1.01f, getCount(1.01f)),
            "bigdemo.xml",
            10,0);
*/

    INstatsV.push_back(INstats);
}


//  NMCore *base_core = new PseudoCore(*(PseudoCore*)nm);

    void InitAxiomSet(string premiseFile)
    {
        AtomSpaceWrapper *atw = GET_ASW;
        atw->reset();
        
        haxx::ArchiveTheorems = true;
        haxx::AllowFW_VARIABLENODESinCore = true;

        cprintf(-2,"loading...\n");     
        
        bool axioms_ok = atw->loadAxioms(premiseFile);
        
        cprintf(-2,"%s loaded. Next test: ", premiseFile.c_str());
        assert(axioms_ok);      
    }

    void RunPLNTest(Btr<PLNTest> t)
    {
        AtomSpaceWrapper *atw = GET_ASW;
        stats::Instance().ITN2atom.clear();

        currentDebugLevel=0;
        rawPrint(*t->target, t->target->begin(), -2);
        
        clock_t start, finish;
        double duration;

        test::custom_duration = 0.0;
        start = clock();
        
        haxx::ArchiveTheorems = false;
        haxx::AllowFW_VARIABLENODESinCore = true; //false;
        
        cprintf(-1, "axioms loaded");
        fflush(stdout);
        currentDebugLevel=-4;

        Btr<BackInferenceTreeRootT> state(new BITNodeRoot(t->target));

        uint s_i=0;
        Handle eh=Handle::UNDEFINED;
        TruthValue* etv = NULL;
        bool passed=false;

        set<VtreeProvider*> eres;

        t->minEvalsOfFittestBIT *= 100; //Minimum "resolution"

        const int expansions_per_run = 1000;

        if (t->minEvalsOfFittestBIT > 0)
        {
            do
            {

/*              for (int k=0;k<expansions_per_run;k++)
                    state->expandFittest();
                
                eres = state->evaluate();*/

                cprintf(-3, "\n    Evaluating...\n");
                
                if (foo42)
                    currentDebugLevel=4;
                
                int expansions = expansions_per_run;
                eres = state->infer(expansions, 0.000001f, 0.01f);      
                
                if (expansions > 0)
                    cprintf(-3, "Succeeded. Saved $%d / $%d (from the beginning of the cycle).\n", expansions, expansions_per_run);
                else
                    cprintf(2, "Failed for now... Saved $%d / $%d (from the beginning of the cycle).\n", expansions, expansions_per_run);

                currentDebugLevel=-4;

                eh = (eres.empty() ? Handle::UNDEFINED : v2h(*(*eres.rbegin())->getVtree().begin()));

                if (eh != Handle::UNDEFINED )
                    etv = atw->getTV(eh).clone();
                //else
                //    etv = new SimpleTruthValue(0.0f,0.0f);

               /* float c1=t->minTV->getConfidence();
                float c2=t->maxTV->getConfidence();
                float m1=t->minTV->getMean();
                float m2=t->maxTV->getMean(); */

                if (etv)
                {
                    printf("%f / %f\n", etv->getConfidence() , t->minTV->getConfidence());
                    printf("%f / %f\n", etv->getMean()          , t->minTV->getMean());
                    printf("%f / %f\n", etv->getConfidence() , t->maxTV->getConfidence());
                    printf("%f / %f\n", etv->getMean()          , t->maxTV->getMean());
                }
                passed = (eh != Handle::UNDEFINED && etv &&
                etv->getConfidence() >= t->minTV->getConfidence() &&
                etv->getMean()          >= t->minTV->getMean() &&
                etv->getConfidence() <= t->maxTV->getConfidence() &&
                etv->getMean()          <= t->maxTV->getMean()
                );
                
        
                cprintf(-4, "TEST Expansion phase %d over.\n", s_i);
            }
            while ((++s_i)*expansions_per_run < t->minEvalsOfFittestBIT && !passed);
        }
        else if (t->minExhaustiveEvals > 0)
        {
            assert(0);
            /// This should be updated to reflect the new BITNode interface
            /*
            for (uint L=0;L<t->minExhaustiveEvals;L++)
                state->expandNextLevel();

            eres = state->evaluate();
            eh = (eres.empty() ? NULL : v2h(eres.rbegin()->value));
            if (eh) {
                if (etv != NULL) delete etv;
                etv = atw->TV(eh).clone();
            }

            passed = (eh && etv &&
                etv->getConfidence() > t->minTV->getConfidence() &&
                etv->getMean()          > t->minTV->getMean()
                );*/
        }
        else
            puts("ERROR IN TEST SETTINGS");

        if (passed) {
            printf("\n**********************************************\n"
                   "passed: %s.\n"
                   "**********************************************\n",
                (etv?etv->toString().c_str():"(null TV)"));

            finish = clock();
            duration = (double)(finish - start) / CLOCKS_PER_SEC;
            printf( "Test took %2.2f seconds TOTAL.\n", duration );
            printf( "Custom test time was %3.3f seconds.\n", test::custom_duration );
            printf( "Custom test time was %3.3f seconds.\n", test::custom_duration2 );
        }
        else {
            printf("\n**********************************************\n"
                   "FAILED: %s!\n"
                   "**********************************************\n",
                (etv?etv->toString().c_str():"(null TV)"));
        }

        printf("Test results: [");
        foreach(VtreeProvider* bv, eres) // state->child_results[0])
        {
            const TruthValue& tv = atw->getTV(vt2h(*bv));
            if (!tv.isNullTv() && tv.getConfidence()>0.0001f)
                printf("%d ", (int)vt2h(*bv).value());
        }
        printf("]\n");

        if (passed)
        {
            AllTestsInferenceNodes += state->InferenceNodes;
            INstats.push_back(state->InferenceNodes);

            //printf("\n\n\nExec pool size: %d\n", state->exec_pool.size());
            printf("\n\n\nInferenceNodes: %ld / %d\n", state->InferenceNodes, AllTestsInferenceNodes);
        }
        else
            INstats.push_back(0);

/*      if (etv)
        {
            string stv(etv->toString());
            puts(stv.c_str());
        }*/

        stats::Instance().print(stats::triviality_filterT());

        if (etv != NULL) delete etv;
        
    }

}} //namespace opencog::pln



#if 0

#include "Rules.h"
#include "PLNEvaluator.h"
#include "InferenceMindAgent.h"
#include "Rules.h"
#include "RouletteSelector.h"
#include "BackwardInferenceTask.h"
#include "spacetime.h"

#include <boost/variant/static_visitor.hpp>
#include <boost/scoped_array.hpp>
#include <boost/foreach.hpp>
#include "AtomSpaceWrapper.h"

#include "../core/ClassServer.h"
#include "BackInferenceTree.h"
#include "BackInferenceTreeNode.h"

using namespace opencog::pln;

struct run1
{
    bool echo_process,  echo_result,  target;

    run1(bool _target, bool _echo_process, bool _echo_result)
    : echo_process(_echo_process), echo_result(_echo_result), target(_target)
    {
    }

    bool operator()(tree<Vertex> target1)
    {
        tree<boost::shared_ptr<InferenceNode> > treetrail;
        
        Handle res=0;       
    
        if (echo_result)
        {
            LOG(0,"Proving:");
            rawPrint(target1, target1.begin(),0);
        }

        try {
            if  (res = complexEvaluator->Evaluate(
                    target1.maketree(), treetrail, echo_process))
            {
                if (echo_result)
                {
                    LOG(0, "\nSuccess!\n");
                    rawPrint(res, res.begin(),0);
                    LOG(0, "\n\nIs producible by the chain:\n");
            
//                  for_each<tree<boost::shared_ptr<InferenceNode> >::post_order_iterator, printInferenceNode>
//                      (treetrail.begin_post(), treetrail.end_post(), printInferenceNode());
                }
            }
            else
            {
                if (echo_result)
                    puts("\nFalse. Targetnproducible.\n");
            }
        } catch(InferenceException e) { puts(e.msg.c_str()); }
        catch(...) { puts("UNKNOWN EXCEPTION!!!"); }

        bool result = (res!=NULL);
    
        if (result != target)
        {
            LOG(0, "Unexpected proof result for:");
            rawPrint(target1, target1.begin(),0);
        }

/*      if (echo_result || (result != target))
        {
            char t[20];
            gets(t);
        }*/

        return (result == target);
    }
};

struct Tester : public map<tree<Vertex>, bool>
{
    void run_all(bool echo_process, bool echo_result)
    {
        for (map<tree<Vertex>, bool>::iterator entry = begin(); entry != end(); entry++)
            run1(entry->second, echo_process, echo_result)(entry->first);
    }
    void run_one(tree<Vertex> a, bool target, bool echo_process, bool echo_result)
    {
        run1(target, echo_process, echo_result)(a);
    }
};

void BackwardOsamaProofTest()
{
puts("BackwardOsamaProofTest running...");

//  assert(STLhas(TheNM.LoadedFiles, "bigdemo.xml"));

    Tester tester;
    
    tester  [tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)CONCEPT_NODE, "Amir"),
                                mva((Handle)CONCEPT_NODE, "Amir")
                            )
            ))] = false;

    tester  [tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "wasKilled"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)CONCEPT_NODE, "Osama")
                            )
            ))] = true;

    tester  [tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)CONCEPT_NODE, "Amir"),
                                mva((Handle)CONCEPT_NODE, "Osama")
                            )
            ))] = true;
    tester  [tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)FW_VARIABLE_NODE, "$OsamaFriend"),
                                mva((Handle)CONCEPT_NODE, "Osama")
                            )
            ))] = true;

    tester  [tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)CONCEPT_NODE, "Britney"),
                                mva((Handle)CONCEPT_NODE, "Amir")
                            )
            ))] = true;

    tester  [tree<Vertex>(mva((Handle)INHERITANCE_LINK,
                    mva((Handle)CONCEPT_NODE, "Osama"),
                    mva((Handle)CONCEPT_NODE, "AlQaeda")
            ))] = true;

    tester  [tree<Vertex>(mva((Handle)INHERITANCE_LINK,
                    mva((Handle)CONCEPT_NODE, "AlQaeda"),
                    mva((Handle)CONCEPT_NODE, "terrorist")
            ))] = true;

    tester  [tree<Vertex>(mva((Handle)INHERITANCE_LINK,
                    mva((Handle)CONCEPT_NODE, "Muhammad"),
                    mva((Handle)CONCEPT_NODE, "terrorist")
            ))] = true;

    tester.run_all(false, false);
return;
    for (int _t=0; _t<1; _t++)
    {
        LOG(1, "Run #" + i2str(_t));
        tester.run_one(tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)CONCEPT_NODE, "Amir"),
                                mva((Handle)CONCEPT_NODE, "Osama")
                            )
            ), true, true, true);

/*      tester.run_one(tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)CONCEPT_NODE, "Amir"),
                                mva((Handle)CONCEPT_NODE, "Amir")
                            )
            ), true, true, true);
*/
/*      tester.run_one(tree<Vertex>(mva((Handle)INHERITANCE_LINK,
//                  mva((Handle)CONCEPT_NODE, "Muhammad"),
                    mva((Handle)CONCEPT_NODE, "AlQaeda"),
                    mva((Handle)CONCEPT_NODE, "terrorist") //"Osama")
            ), true, true, true);
*/
/*      tester.run_one(tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    mva((Handle)PREDICATE_NODE, "friendOf"),
                    mva((Handle)LIST_LINK,
                                mva((Handle)CONCEPT_NODE, "Britney"),
                                mva((Handle)CONCEPT_NODE, "Amir")
                            )
            ), true, true, true);*/

/*      tester.run_one(tree<Vertex>(mva((Handle)INHERITANCE_LINK,
                    mva((Handle)CONCEPT_NODE, "Britney"),
                    mva((Handle)CONCEPT_NODE, "terrorist")
            ), true, true, true);*/

/*  tester.run_one(tree<Vertex>(mva((Handle)EVALUATION_LINK,
                    atom(PREDICATE_NODE, "wasKilled"),
                    atom(LIST_LINK,
                                atom(CONCEPT_NODE, "Osama")
                            )
            ), true, false, true);*/


//  printf("\nUnprovable atoms: %d\n", opencog::pln::haxx::Unprovable().size());
//  printf("\nProvable atoms: %d\n", opencog::pln::haxx::Provable().size());

        printf("\nReserved atoms: %d\n", atom_alloc_count);
        printf("\nReserved InferenceNodes: %d\n", inode_alloc_count);

        freehaxx();

        puts("After killing the haxx tree");
        printf("\nReserved atoms: %d\n", atom_alloc_count);
        printf("\nReserved InferenceNodes: %d\n", inode_alloc_count);
    }
}

void SetProofTest();
void BOATest();
void ForwardOsamaProofTest();
void BackwardOsamaProofTest();
void RelimTest();
void FIMTest();
void NewChainerTest();

int __ptlc=0;

namespace opencog { namespace pln
{
Handle Ass(iAtomSpaceWrapper *destTable, Handle h, std::vector<Handle>& ret);
}};

void TestAssociatedSets()
{
/*    Testing associated sets*/
    
/*      AtomSpaceWrapper& TheNM = *((AtomSpaceWrapper*)haxx::defaultAtomSpaceWrapper);
    
      Handle h1 = Ass(&TheNM, atom(CONCEPT_NODE, "terrorist").attach(haxx::defaultAtomSpaceWrapper), testv);
      Handle h2 = Ass(&TheNM, atom(CONCEPT_NODE, "Muhammad").attach(haxx::defaultAtomSpaceWrapper), testv);
      Handle hs[] = {h1,h2};

      SubsetEvalRule ser(&TheNM);
      Handle IntInhSet = ser.compute(hs,2);

      printTree(IntInhSet,0,1);*/
}

void InferenceTests()
{
    try {   
      map<string,Handle> bindings;

/*          Handle t5 = atom(IMPLICATION_LINK, 2, 
                    new atom(AND_LINK, 2,
                        new atom(SUBSET_LINK, 2,
                            new atom(CONCEPT_NODE, "D"),
                            new atom(CONCEPT_NODE, "Z")
                        ),
                        new atom(EVALUATION_LINK, 2, 
                            new atom(PREDICATE_NODE, "union"),
                            new atom(LIST_LINK, 3,
                                new atom(CONCEPT_NODE, "W"),
                                new atom(CONCEPT_NODE, "C"),
                                new atom(FW_VARIABLE_NODE, "$1")
                            )
                        )
                    ),
                    new atom(SUBSET_LINK, 2,
                        new atom(FW_VARIABLE_NODE, "$2"),
                        new atom(CONCEPT_NODE, "Z")
                    )
                ).attach(((AtomSpaceWrapper*)haxx::defaultAtomSpaceWrapper));
*/
/*  Handle t0 = atom(IMPLICATION_LINK, 2, 
                    new atom(AND_LINK, 2,
                        new atom(SUBSET_LINK, 2,
                            new atom(CONCEPT_NODE, "D"),
                            new atom(CONCEPT_NODE, "Z")
                        ),
                        new atom(EVALUATION_LINK, 2, 
                            new atom(PREDICATE_NODE, "union"),
                            new atom(LIST_LINK, 3,
                                new atom(CONCEPT_NODE, "Q"),
                                new atom(CONCEPT_NODE, "C"),
                                new atom(FW_VARIABLE_NODE, "$1")
                            )
                        )
                    ),
                    new atom(SUBSET_LINK, 2,
                        new atom(FW_VARIABLE_NODE, "$2"),
                        new atom(CONCEPT_NODE, "Z")
                    )
                ).attach(((AtomSpaceWrapper*)haxx::defaultAtomSpaceWrapper));
        
    Handle t1 = atom(IMPLICATION_LINK, 2, 
                    new atom(AND_LINK, 2,
                        new atom(SUBSET_LINK, 2,
                            new atom(CONCEPT_NODE, "D"),
                            new atom(CONCEPT_NODE, "Z")
                        ),
                        new atom(EVALUATION_LINK, 2, 
                            new atom(PREDICATE_NODE, "union"),
                            new atom(LIST_LINK, 3,
                                new atom(CONCEPT_NODE, "Q"),
                                new atom(CONCEPT_NODE, "C"),
                                new atom(CONCEPT_NODE, "Z")
                            )
                        )
                    ),
                    new atom(SUBSET_LINK, 2,
                        new atom(CONCEPT_NODE, "D"),
                        new atom(CONCEPT_NODE, "Z")
                    )
                ).attach(((AtomSpaceWrapper*)haxx::defaultAtomSpaceWrapper));
        Handle t2 = t1;
      
//  bool s1 = substitutableTo(t1, t2, bindings);
//  bool s5 = substitutableTo(t5, t2, bindings);

LOG(0,"\n-------\n");
    
    bool s0 = substitutableTo(t0, t2, bindings);
    */
    //if (s1) puts("True!");
    //else puts("0!"); 
    /*if (s5) puts("True!");
    else puts("0!"); */
/*  if (s0) puts("True!");
    else puts("0!");    */
  } catch(std::string s)
  {
      LOG(0, s + " at root level.");
  }
  catch(...)
  {
      LOG(0, "Unknown exception at root level. ");
  }
}

void GeneralTests()
{
/*  LOG(2, "Dumping Core...");
    
     TheNM.DumpCoreLinks(1);*/
/*
puts("Osama:terrorist");
      TableGather fa(atom(INHERITANCE_LINK, 2,
                    new atom(CONCEPT_NODE, "Osama"),
                    new atom(CONCEPT_NODE, "terrorist") //"Osama")
            ));

      for_each<std::vector<Handle>::iterator, handle_print<0> >(fa.begin(), fa.end(), handle_print<0>());

atom(INHERITANCE_LINK, 2,
                                new atom(CONCEPT_NODE, "Osama"),
                                new atom(CONCEPT_NODE, "Amir")
            ).attach(haxx::defaultAtomSpaceWrapper);
      for_each<std::vector<Handle>::iterator, handle_print<0> >(fa.begin(), fa.end(), handle_print<0>());
    */
/*puts("LL2:");         
      TableGather LL(atom(LIST_LINK, 2,
                                new atom(CONCEPT_NODE, "Amir"),
                                new atom(CONCEPT_NODE, "Amir")
            ));
      for_each<std::vector<Handle>::iterator, handle_print<0> >(LL.begin(), LL.end(), handle_print<0>());
puts(":");

    atom(CONCEPT_NODE, "Gibson").attach(haxx::defaultAtomSpaceWrapper);
    Node *node = new Node(CONCEPT_NODE, strdup("Gibson"));
    MindDBProxy::getInstance()->add(node, true);
    printTree(haxx::defaultAtomSpaceWrapper->getHandle(CONCEPT_NODE, "Gibson"),0,0);
      TableGather gg(atom(CONCEPT_NODE, "Gibson"));
      for_each<std::vector<Handle>::iterator, handle_print<0> >(gg.begin(), gg.end(), handle_print<0>());
getc(stdin);*/

//  BackwardOsamaProofTest();

      //TheNM.DumpCore(FORALL_LINK);

/* // Dump universally-quantified expressions:
      TableGather fa(atom(__INSTANCEOF_N,1,
                            new atom(FORALL_LINK,0)
                    ));
      for_each<TableGather::iterator, handle_print<0> >(fa.begin(), fa.end(), handle_print<0>());*/

//    BOATest();
//    ForwardOsamaProofTest();

    //FIMTest();

//  NewChainerTest();
}

void AgentTest2()
{
    using namespace opencog::pln;
    
    InitAxiomSet("fetch.xml");
#if 1
    set<MindAgent*> agents;
    AtomSpaceWrapper& TheNM = *((AtomSpaceWrapper*)haxx::defaultAtomSpaceWrapper);

    vtree rewardt(mva((Handle)EVALUATION_LINK,
                NewNode(PREDICATE_NODE, "+++")));
    Handle reward = NewAtom(rewardt);

    /// the addAtom interface needs improvement!
    vtree mediatort(mva((Handle)EVALUATION_LINK, NewNode(PREDICATE_NODE,"mediator")));
//  vtree walkt(mva((Handle)EXECUTION_LINK, NewNode(PREDICATE_NODE,"move.forward"), NewNode(NUMBER_NODE, "2.0")));
//  vtree walkbackt(mva((Handle)EXECUTION_LINK, NewNode(PREDICATE_NODE,"move.backward"), NewNode(NUMBER_NODE, "2.0")));
    vtree walkt(mva((Handle)EVALUATION_LINK,
                        //NewNode(PREDICATE_NODE,"just_done"),
                        NewNode(PREDICATE_NODE,"do"),
                        mva((Handle)LIST_LINK,
                            mva((Handle)EXECUTION_LINK, NewNode(PREDICATE_NODE,"move.forward 2.0")))
                ));
    vtree walkbackt(mva((Handle)EVALUATION_LINK,
                        NewNode(PREDICATE_NODE,"do"),
//                      NewNode(PREDICATE_NODE,"just_done"),
                        mva((Handle)LIST_LINK,
                            mva((Handle)EXECUTION_LINK, NewNode(PREDICATE_NODE,"move.backward 2.0")))
                    ));
    vtree runt(mva((Handle)EXECUTION_LINK, NewNode(PREDICATE_NODE,"move.forward 0.1")));
    Handle mediator = NewAtom(mediatort);
//  Handle walk     = NewAtom(walkt);
    Handle walk_back= TheNM.addAtom(
            walkbackt,
            new SimpleTruthValue(0.5,0.5), true, false);
    Handle walk = TheNM.addAtom(
            walkt,
            new SimpleTruthValue(0.5,0.5), true, false);
    //NewAtom(walkbackt);
    Handle run = NewAtom(runt);

/*  vtree walk_to_reward(mva((Handle)IMP,
                mva(walk),
                mva(reward)));
    vtree run_tp_mediate(mva((Handle)IMP,
                mva(run),
                mva(mediator)));
    vtree mediate_to_reward(mva((Handle)IMP,
                mva(mediator),
                mva(reward)));
        
    printTree(TheNM.addAtom(
i           walk_to_reward,
            new SimpleTruthValue(0.2,0.2), true, false), 0,-3);*/
/*
    printTree(TheNM.addAtom(
            run_tp_mediate,
            new SimpleTruthValue(0.1,0.9), true, false), 0,-3);
    
    printTree(TheNM.addAtom(
            mediate_to_reward,
            new SimpleTruthValue(0.1,0.9), true, false), 0,-3);
*/

#ifndef WIN32
    /*  timeval currentTime;
    gettimeofday(&currentTime, NULL);
    long now_stamp = currentTime.tv_sec*1000 + currentTime.tv_usec/1000;*/
    long now_stamp = getElapsedMillis();
    long now_interval = 1000;
#else
    long now_stamp = 3000;
    long now_interval = 1000;
#endif
    char end_stamp_s[100], begin_stamp_s[100];
    sprintf(begin_stamp_s, "%ul", now_stamp-now_interval);
    sprintf(end_stamp_s, "%ul", now_stamp);

    char startNodeName[100], endNodeName[100];
    sprintf(startNodeName, "%ul", 0);
    sprintf(endNodeName, "%ul", 1000000);

    /**
Timestamps denote the number of milliseconds since (00:00:00 UTC, January
1, 1970), which is the Epoch used by time_t time(time_t *t) C function.
See the manual for it by typing "man 2 time" in your Linux prompt.
*/

            vtree heard_sound(
                mva((Handle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "atInterval"),
                    mva((Handle)LIST_LINK,
                    mva((Handle)EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "AGISIM_quality"),
                        mva((Handle)LIST_LINK,
                            NewNode(AGISIM_SOUND_NODE, ""),
                            NewNode(NUMBER_NODE, "1000")
                            )
                        ),
                    NewNode(NUMBER_NODE, startNodeName),
//                  NewNode(NUMBER_NODE, endNodeName),                  
//                  NewNode(CONCEPT_NODE, "!whileago"),
                    NewNode(CONCEPT_NODE, "!now")
                    )
                ));
                
    vtree back_when_sound(mva((Handle)IMP,
            mva((Handle)AND_LINK,
                heard_sound,
                mva(walk_back)),
            mva(reward)));

    vtree target(mva((Handle)EVALUATION_LINK,
            NewNode(PREDICATE_NODE, "+++")));

/*  
    vtree forward_when_no_sound(mva((Handle)IMP,
            mva((Handle)AND_LINK,
                mva((Handle)NOT_LINK,
                    heard_sound),
                mva(walk_back)),
            mva(reward)));
    
    printTree(TheNM.addAtom(
        forward_when_no_sound,
        new SimpleTruthValue(0.95,0.95), true, false)
    ,0,-3);
*/
    printTree(TheNM.addAtom(
        back_when_sound,
        new SimpleTruthValue(0.95,0.95), true, false)
    ,0,-3);

    //  puts("start agents");
    //  while (1)
    //  for_each(agents.begin(), agents.end(), mem_fun(&MindAgent::execute));
#endif
}

#endif
