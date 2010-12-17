// This was a previous version of Tester.cc; the more recent version of
// Tester.cc became PLNUTest.cxxtest.
#include "Rules.h"
#include "PLNEvaluator.h"
#include "HandleEntry.h"
#include "InferenceMindAgent.h"
#include "Rules.h"
#include "RouletteSelector.h"
#include "BackwardInferenceTask.h"
#include "spacetime.h"

#include <boost/variant/static_visitor.hpp>
#include <boost/scoped_array.hpp>
#include <boost/foreach.hpp>
#include "AtomTable.h"
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

namespace opencog { namespace pln {
Handle Ass(iAtomSpaceWrapper *destTable, Handle h, std::vector<Handle>& ret);
}};

//void TestAssociatedSets()
{
/*    Testing associated sets*/
    
/*      AtomSpaceWrapper& TheNM = *((AtomSpaceWrapper*)ASW());
    
      Handle h1 = Ass(&TheNM, atom(CONCEPT_NODE, "terrorist").attach(ASW()), testv);
      Handle h2 = Ass(&TheNM, atom(CONCEPT_NODE, "Muhammad").attach(ASW()), testv);
      Handle hs[] = {h1,h2};

      SubsetEvalRule ser(&TheNM);
      Handle IntInhSet = ser.compute(hs);

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
                ).attach(((AtomSpaceWrapper*)ASW()));
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
                ).attach(((AtomSpaceWrapper*)ASW()));
        
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
                ).attach(((AtomSpaceWrapper*)ASW()));
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
            ).attach(ASW());
      for_each<std::vector<Handle>::iterator, handle_print<0> >(fa.begin(), fa.end(), handle_print<0>());
    */
/*puts("LL2:");         
      TableGather LL(atom(LIST_LINK, 2,
                                new atom(CONCEPT_NODE, "Amir"),
                                new atom(CONCEPT_NODE, "Amir")
            ));
      for_each<std::vector<Handle>::iterator, handle_print<0> >(LL.begin(), LL.end(), handle_print<0>());
puts(":");

    atom(CONCEPT_NODE, "Gibson").attach(ASW());
    Node *node = new Node(CONCEPT_NODE, strdup("Gibson"));
    MindDBProxy::getInstance()->add(node, true);
    printTree(ASW()->getHandle(CONCEPT_NODE, "Gibson"),0,0);
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

/*  HandleEntry* result = MindDBProxy::getInstance()->getHandleSet((Type)INHERITANCE_LINK, true);
            printf("%d results direct\n", result->getSize());*/

    //FIMTest();

//  NewChainerTest();
}

void AgentTest2()
{
    using namespace opencog::pln;
    
    initAxiomSet("fetch.xml");
#if 1
    set<MindAgent*> agents;
    AtomSpaceWrapper& TheNM = *((AtomSpaceWrapper*)ASW());

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

