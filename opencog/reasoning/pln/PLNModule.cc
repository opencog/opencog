/*
 * opencog/reasoning/pln/PLNModule.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * Contains adapted code from PLNShell.cc
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

#include "PLNModule.h"

#include "PLN.h"
#include "rules/Rules.h"

#define BackInferenceTreeRootT BITNodeRoot

//#include "PTLEvaluator.h"

#include "rules/RuleProvider.h"
#include "AtomSpaceWrapper.h"
#include "BackInferenceTreeNode.h"
#include <opencog/util/Logger.h>
#include <opencog/atomspace/utils.h>
// #include <opencog/guile/SchemeEval.h>

#include <boost/foreach.hpp>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#include "TestTargets.h"

using namespace opencog;
using namespace opencog::pln;

DECLARE_MODULE(PLNModule)

#ifdef _MSC_VER
#pragma warning(disable : 4312)
#endif // _MSC_VER

//! debug level, @todo replaced by opencog log system
extern int currentDebugLevel;

namespace haxx
{
    extern bool AllowFW_VARIABLENODESinCore;
    extern bool printRealAtoms;
//    extern opencog::pln::iAtomSpaceWrapper* defaultAtomSpaceWrapper;
}

namespace opencog {
namespace pln
{
    extern bool RECORD_TRAILS;
}}

namespace test
{
    extern FILE *logfile;
//    int _test_count = 0;
    // not used (it's set but nothing responds to it)
//    bool debugger_control = false;
}

//void initTestEnv();
//void Init();
std::string RunCommand(std::list<std::string> args);

PLNModule::PLNModule() : Module()
{
    logger().info("[PLNModule] constructor");
	do_pln_register();	
	cogserver().registerAgent(BackChainingAgent::info().id, &backChainingFactory);
}

PLNModule::~PLNModule() {
    logger().info("[PLNModule] destructor");
    do_pln_unregister();
    cogserver().unregisterAgent(BackChainingAgent::info().id);
}

// state variables for running multiple PLNShell commands.
Btr<BackInferenceTreeRootT> Bstate;
BackInferenceTreeRootT* temp_state, *state;

void PLNModule::init()
{
    logger().info("[PLNModule] init");
//    CogServer& cogserver = static_cast<CogServer&>(server());

//! @todo this should be moved to a separate method so it can
// also be accessed from PLNUTest
//	initTestEnv();
    RECORD_TRAILS = true;

    currentDebugLevel=100;
	
	// Make sure that the ASW is initialized on module load
	iAtomSpaceWrapper* asw = ASW();
	
//    Init();
    #if LOCAL_ATW
//    haxx::defaultAtomSpaceWrapper = &opencog::pln::LocalATW::getInstance();
    ((LocalATW*)asw)->SetCapacity(10000);
    #endif  
    
    #if LOG_ON_FILE
     test::logfile=fopen("pln.log","wt");
     cout << "LOGGING TO FILE pln.log!\n";
    #endif

    haxx::printRealAtoms = true;
    ((AtomSpaceWrapper*)asw)->archiveTheorems = false;

    // no longer done at module load - it would be inappropriate
    // for contexts other than testing PLN
    /*initTests();*
    Bstate = Btr<BackInferenceTreeRootT>(new BITNodeRoot(tests[0],
        new DefaultVariableRuleProvider));
    printf("BITNodeRoot init ok\n");
    temp_state = Bstate.get();
    state = Bstate.get();*/
}

std::string PLNModule::do_pln(Request *dummy, std::list<std::string> args)
{
/*	if (args.size() != 3)
		return "sql-load: Wrong num args";*/

/*	std::string dbname   = args.front(); args.pop_front();
	std::string username = args.front(); args.pop_front();
	std::string auth	   = args.front(); args.pop_front();*/

//    initTestEnv(args);

    std::string output = RunCommand(args);

	return output;
}

void opencog::setTarget(Handle h) {
    pHandleSeq fakeHandles = ((AtomSpaceWrapper*)ASW())->realToFakeHandle(h);
    pHandle fakeHandle = fakeHandles[0];
//    vtree target* = new vtree(fakeHandle);
    Btr<vtree> target(new vtree(fakeHandle));
    
//    opencog::pln::printTree(h,0,0);
//    rawPrint(ret, ret.begin(), 0);

    Bstate.reset(new BITNodeRoot(target, new DefaultVariableRuleProvider));

    printf("BITNodeRoot init ok\n");
//    temp_state = Bstate.get();
    state = Bstate.get();
}

void opencog::infer(Handle h, int &steps)
{
    Btr<BackInferenceTreeRootT> Bstate;
    BackInferenceTreeRootT *state;

    pHandleSeq fakeHandles = ((AtomSpaceWrapper*)ASW())->realToFakeHandle(h);
    pHandle fakeHandle = fakeHandles[0];
    Btr<vtree> target(new vtree(fakeHandle));

    Bstate.reset(new BITNodeRoot(target, new DefaultVariableRuleProvider));

    printf("BITNodeRoot init ok\n");
    state = Bstate.get();
    state->infer(steps, 0.000001f, 0.01f);
    state->printResults();
//    ss << "\n" << j << " $ remaining.\n";

}

#if 0
void initTestEnv()
{
    try {
        puts("Initializing PLN test env...");

        RECORD_TRAILS = true;
        haxx::printRealAtoms = true;

        currentDebugLevel=100;

//        LOG(2, "Creating AtomSpaceWrappers...");
        
/*
#if LOCAL_ATW
        haxx::defaultAtomSpaceWrapper = &LocalATW::getInstance();
#else
        DirectATW::getInstance();
        haxx::defaultAtomSpaceWrapper = &NormalizingATW::getInstance();
#endif*/
//        AtomSpaceWrapper& atw = *GET_ATW;
        
#if 0 //Loading Osama or set axioms here.

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

#endif
        logger().debug("PTL Initialized.");
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
//        ThePLNShell.Launch();

        return;
    }
    catch(string s) {
        printf("Exception in initTestEnv(): %s\n", s.c_str()); }
    catch(boost::bad_get bg) {
        printf("Bad boost::get in initTestEnv(): %s\n", bg.what()); }
    catch(...) {
        puts("Exception in initTestEnv()."); }
  
    getc(stdin);
}
#endif

#if 0
void Init()
{
    #if LOCAL_ATW
//    haxx::defaultAtomSpaceWrapper = &opencog::pln::LocalATW::getInstance();
    ((LocalATW*)ASW())->SetCapacity(10000);
    #endif  
    
    #if LOG_ON_FILE
     test::logfile=fopen("pln.log","wt");
     cout << "LOGGING TO FILE pln.log!\n";
    #endif

    haxx::printRealAtoms = true;
}
#endif

template <typename T>
T input(T& a, std::list<std::string>& args)
{
//    T a;
    std::stringstream ss;
    if (args.begin() != args.end()) {
        ss << args.front();
        args.pop_front();
    }
    ss >> a;
    
    return a;
}

std::string RunCommand(std::list<std::string> args)
{
    AtomSpaceWrapper* atw = GET_ATW;
    // All commands/strings in requiresRoot need for the state
    // BITNodeRoot to be non-NULL
    static const char* requiresRoot[] = {
        "infer", "plan", "trail", "pool", "pool-size", "pool-fittest",
        "pool-expand", "bit-next-level", "bit-eval", "bit-find-node", 
        "loop-check", 0 };
    std::set<string> setRequiresRoot;
    for (int i = 0; requiresRoot[i]; i++) {
        setRequiresRoot.insert(requiresRoot[i]);
    }

    std::stringstream ss;
    try {
        int a1T, a2T, bT, tempi=0;
        string a10, a11, a20, a21, b1, b2;
        vtree avt1, avt2, bvt;
        int qrule=0;
        Rule::MPs rule_args;
        bindingsT new_bindings;

        string c;
        std::string temps;
        long h=0, h2=0;
        int j;
        int test_i=0;
        int s_i=0;
        bool axioms_ok;
        boost::shared_ptr<set<pHandle> > ts;
        Vertex v;
        Handle eh=Handle::UNDEFINED;

        // Get command
        input(c, args);
        
#if LOG_ON_FILE
        save_log();
#endif
        haxx::AllowFW_VARIABLENODESinCore = true; //false;

        // Check whether command requires root to be set...
        set<string>::iterator rs_it;
        rs_it = setRequiresRoot.find(c);
        if (state == NULL && rs_it != setRequiresRoot.end()) {
            ss << "error: No root BIT. Set a target first?\n";
            return ss.str();
        }
        
        // Please keep these in the same order as the help
        // for the "pln" command!
        //! @todo Need to add commands for exploring the AtomSpace as viewed
        //! through the AtomSpaceWrapper.
        //! @todo Need to add commands for controlling multiple roots/targets.
        if (c == "log") {
            input(h, args); currentDebugLevel = (int) h;
            ss << "PLN log level now " << currentDebugLevel << endl;
        }
        else if (c == "record-trails") {
            RECORD_TRAILS = !RECORD_TRAILS;
            ss << "Recording trails now " << (RECORD_TRAILS?"ON":"OFF") << endl;
        }
        else if (c == "infer") {
            // Give max nr of steps that we can take.
            input(h, args);
            j = (long) h;
            state->infer(j, 0.000001f, 0.01f);
            state->printResults();
            ss << "\n" << j << " $ remaining.\n";
        }
        else if (c == "atom") {
            input(h, args);
            printTree((pHandle)h,0,0);
        }
        else if (c == "plan") {
            input(h, args);
            state->extract_plan((pHandle)h);
        }
        else if (c == "trail") {
            input(h, args); state->printTrail((pHandle)h);
        }
        // Pool commands
        else if (c == "pool") {
            tempi = currentDebugLevel;
            currentDebugLevel = 10;
            state->printFitnessPool();
            currentDebugLevel = tempi;
        }
        else if (c == "pool-size") {
            ss << "Pool size = " << state->getExecPoolSize() << endl;
        }
        else if (c == "pool-fittest") {
            state->expandFittest();
            ss << "Expanded fittest, pool size now = " <<
                state->getExecPoolSize() << endl;
        }
        else if (c == "pool-expand") {
            s_i=0;
            input(h, args);
            for (int k=0;k<h;k++)
                state->expandFittest();
        }
        // BIT commands
        else if (c == "bit") {
            input(h, args);
            if (h == 0) h = (long) state->children[0].begin()->prover;
            ((BackInferenceTreeRootT*)h)->print(); 
        }
        else if (c == "bit-expand") {
            input(h, args);
            if (h == 0) h = (long) state;
            ((BackInferenceTreeRootT*)h)->expandNextLevel();
/*          foreach(const parent_link& p, ((BackInferenceTreeRootT*)h)->GetParents())
                p.link->removeIfFailure((BackInferenceTreeRootT*)h);*/
        }
        else if (c == "bit-results") {
            input(h, args);
            if (h == 0) h = (long)state;
//          h = (int)state->children[0].begin()->prover;
            ((BackInferenceTreeRootT*)h)->printResults();
/*          foreach(const set<BoundVertex>& eval_res_set, ((BackInferenceTreeRootT*)h)->GetEvalResults())
            foreach(const BoundVertex& eval_res, eval_res_set)
                printTree(v2h(eval_res.value),0,-10);*/
        }
        else if (c == "bit-parents") {
            input(h, args);
            foreach(const parent_link<BITNode>& p, ((BITNode*)h)->getParents()) {
                ss << "User Node = " << (ulong) p.link << endl;
            }
            // Use print_parents when it's changed to return a string
        }
        else if (c == "bit-rule-args") {
            input(h, args);
            ((BackInferenceTreeRootT*)h)->printArgs();
        }
        else if (c == "bit-rule-target") { input(h, args);
            cprintf(-10, "Target:\n");
            ((BackInferenceTreeRootT*)h)->printTarget();
            cprintf(-10, "Results:\n");
            ((BackInferenceTreeRootT*)h)->printResults();
            cprintf(0, "parent arg# %d\n", ((BackInferenceTreeRootT*)h)->getParents().begin()->parent_arg_i);
        }
        //! @todo work this out?
        else if (c == "bit-direct-results") {
            input(h, args);
            ss << "Disabled" << endl;
            /*cprintf(0, "Node has results & bindings:\n");
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
            }*/
        }
        // Testing
        else if (c == "load-axioms") {
            input(temps, args);
            if (temps == "") {
                ss << "error: expected xml input filename\n";
                return ss.str();
            }
            atw->reset();
            atw->archiveTheorems = true;
            axioms_ok = atw->loadAxioms(temps);
            atw->archiveTheorems = false;
            ss << (axioms_ok ? "Input file was loaded."
                    : "Input file was corrupt.");
            ss << " Next you MUST (re)load a target atom with the pln target"
                  " command! Otherwise things will break." << endl;
            // have to recreate the target vtrees to ensure that the
            // handles are correct after reloading axioms.
            initTests();
        }
        else if (c == "test-count") {
            ss << tests.size() << std::endl;
        }
        else if (c == "test-target") {
            input(test_i, args);
            Bstate.reset(new BITNodeRoot(tests[test_i], new DefaultVariableRuleProvider));
            state = Bstate.get();
            cprintf(0,"Test target set: ");
            rawPrint(*tests[test_i],tests[test_i]->begin(),0);
            cprintf(0,"\n");
        }
        else if (c == "=") { input(h, args); input(h2, args); 
/*          cprintf(0, ((BackInferenceTreeRootT*)h)->eq((BackInferenceTreeRootT*)h2) ? "EQ\n" : "IN-EQ\n"); */
            ss << ( ((BackInferenceTreeRootT*)h)->eq((BackInferenceTreeRootT*)h2) ?
                "EQ" : "IN-EQ" ) << std::endl;
        }
        // Not recommended
        else if (c == "bit-next-level") { state->expandNextLevel(); }
        else if (c == "bit-eval") {
            try {
                state->evaluate();
            } catch(string s) {
                cprintf(0,s.c_str());
            }
        }
        //! @todo deal with arguments to bit-find-node, make rule the first arg.
        else if (c == "bit-find-node") { 
            input(bT, args); input(b1, args); input(b2, args);
            input(a1T, args); input(a10, args); input(a11, args);
            input(a2T, args); input(a20, args); input(a21, args);
//          puts("Enter Rule #: ");
            input(qrule, args);

            bvt = (mva((pHandle)bT, NewNode(CONCEPT_NODE, b1), NewNode(CONCEPT_NODE, b2)));
            avt1 = (mva((pHandle)a1T, NewNode(CONCEPT_NODE, a10), NewNode(CONCEPT_NODE, a11)));
            avt2 = (mva((pHandle)a2T, NewNode(CONCEPT_NODE, a20), NewNode(CONCEPT_NODE, a21)));

            rawPrint(bvt, bvt.begin(), -2);
            rawPrint(avt1, avt1.begin(), -2);
            rawPrint(avt2, avt2.begin(), -2);

            rule_args.clear();
            rule_args.push_back(BBvtree(new BoundVTree(avt1)));
            if (a2T)
                rule_args.push_back(BBvtree(new BoundVTree(avt2)));
            else
//              puts("Passing 1 arg.");
                ss << "Passing 1 arg.\n";

//          printf("BITNode %ld.", (long) state->findNode((Rule*)qrule, meta(new vtree(bvt)), rule_args, new_bindings));
            ss << "BITNode " << (long) state->findNode((Rule*)qrule, meta(new vtree(bvt)), rule_args, new_bindings) << ".\n" ;

        }
        else if (c == "loop-check") { state->loopCheck(); }
/*      else if (c == "W") {
            if (using_root) {
                temp_state = state;
                input(h, args);
                state = (BackInferenceTreeRootT*)h;
                using_root = false;
            }
            else {
                state = temp_state;
                using_root = true;
            }
        }*/
        else {
            ss << "Unknown PLN command: '" << c << "'.\n" <<
                "Use 'help pln' for a list of valid commands." << endl;

        }
    } catch( std::exception& e )
    {
        cout << endl << "Exception: "
             << e.what() << endl;
    }
    return ss.str();
}
