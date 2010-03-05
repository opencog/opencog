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
#include "rules/RuleProvider.h"
#include "AtomSpaceWrapper.h"
#include "BackInferenceTreeNode.h"

#include <opencog/guile/SchemePrimitive.h>
#include <opencog/util/Logger.h>
#include <opencog/util/Config.h>

#include <boost/foreach.hpp>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#include "TestTargets.h"
#include "ForwardChainer.h"

#include <opencog/adaptors/tulip/TulipWriter.h>

//! @todo For PLN testing. Maybe move it to another file used by this module and PLNUTest
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/load-file.h>
//! @todo For PLN testing. Maybe move it to another file used by this module and PLNUTest
#include <boost/filesystem.hpp>
using namespace boost::filesystem;



using namespace opencog;
using namespace opencog::pln;

using std::string;
using std::set;
using std::cout;
using std::endl;


DECLARE_MODULE(PLNModule)

//! @todo replace by opencog log system
extern int currentDebugLevel;

const char* PLNModule::usageInfo = 
    "Usage: pln <command>\n\n"
    "Run the specified PLN command.\n"
    "( NOTE THE DIFFERENCE BETWEEN ARG TYPES:\n"
    "- some commands take PLN Handles, these are different from AtomSpace Handles!\n"
    "- some take BITNode pointers. )\n"
    "\n"
    "---\n"
    " log <[-5..5]> - Set log level (0 = normal log level).\n"
    " record-trails - Switch the recording of inference trails ON/OFF (default: ON)\n"
    " infer <s>     - Infer until result found with conf > 0.01 OR 's' inference steps\n"
    "                 have been taken.\n"
    " atom <h>      - print the atom with PLN Handle h\n"
    " plan <h>      - Show the plan ie. sequence of 'do' statements pertaining to inference\n"
    "                 result of PLN Handle h\n"
    " trail <h>     - print the inference trail for PLN Handle #h\n"
    "\n"
    "--- Pool\n"
    " pool          - Show the current BIT node expansion pool sorted by heuristic fitness\n"
    " pool-size     - Return current BIT node expansion pool size\n"
    " pool-fittest  - Show the current BIT node expansion pool sorted by heuristics\n"
    "                 fitness and expand the fittest BIT node\n"
    " pool-expand <n>- Execute the #n fittest BIT nodes\n"
    "\n"
    "--- BIT\n"
    " bit <n>            - Print the inference (BIT) tree under node n (0 = root)\n"
    " bit-expand <n>     - Expand BITNode with id n\n"
    " bit-results <n>    - Print out the results of BIT node n (0 = root)\n"
    " bit-parents <n>    - Show the parents of BITNode #n\n"
    " bit-rule-args <n>  - Print the Rule arguments of BIT node n\n"
    " bit-rule-target <n>- Print the Rule target of BIT node n\n"
    " bit-direct-results #i - (disabled) Show the direct results (by lookup or hypothesis)\n "
    "                 of BIT node #i\n"
    "\n"
    "--- Testing\n"
    " load-axioms <path> - Load XML axiom file in 'path'\n"
	" load-test <test name> - Load a test, including the dataset and target\n"
	" list-tests - List all the tests available\n"
    " test-count         - count the number of pre-defined inference targets (obsolete)\n"
    " test-target <n>    - Load in a new pre-defined target #n (from TestTargets.h) (obsolete)\n"
    " = <n1> <n1>        - Check if BIT nodes n1 and n2 are equal.\n"
    "\n"
    "--- The following are not recommended unless you know what your doing:\n"
    " bit-next-level     - Expand the tree's whole next level (usually not recommended)\n"
    " bit-eval           - Manually evaluate the current tree (usually not recommended)\n"
    " bit-find-node  bT b1 b2  a1T a10 a11 [a2T a20 a21] <Rule ptr> - find a BITNode \n"
    "                 for the given rule with the given parameters. 3rd parameter depends \n"
    "                 on rule number, but must be specified (needs to be more friendly).\n"
    " loop-check         - check for loops\n";


PLNModule::PLNModule() : Module()
{
    logger().info("[PLNModule] constructor");
    setParameters(DEFAULT());
    do_pln_register();  
    cogserver().registerAgent(BackChainingAgent::info().id, &backChainingFactory);
    cogserver().registerAgent(ForwardChainingAgent::info().id, &forwardChainingFactory);
}

void PLNModule::setParameters(const std::string* params) {
    for (unsigned int i = 0; params[i] != ""; i += 2) {
        if (!config().has(params[i])) {
           config().set(params[i], params[i + 1]);
        }
    }
}

PLNModule::~PLNModule() {
    logger().info("[PLNModule] destructor");
    do_pln_unregister();
    cogserver().unregisterAgent(BackChainingAgent::info().id);
    cogserver().unregisterAgent(ForwardChainingAgent::info().id);
}

// state variables for running multiple PLNShell commands.
Btr<BITNodeRoot> Bstate;
BITNodeRoot* temp_state, *state;

void PLNModule::init()
{
    logger().info("[PLNModule] init");

    recordingTrails = config().get_bool("PLN_RECORD_TRAILS");
    currentDebugLevel = config().get_int("PLN_LOG_LEVEL");
    fitnessEvaluator = getFitnessEvaluator(config().get("PLN_FITNESS_EVALUATOR"));
    
    // Make sure that the ASW is initialized on module load
    AtomSpaceWrapper* asw = ASW(server().getAtomSpace());
#if LOCAL_ATW
    ((LocalATW*)asw)->SetCapacity(10000);
#endif  
    asw->archiveTheorems = false;
    asw->allowFWVarsInAtomSpace = 
        config().get_bool("PLN_FW_VARS_IN_ATOMSPACE");


#ifdef HAVE_GUILE
    // Define a scheme wrapper -- the scheme function pln-bc will
    // call the pln_bc method.
    define_scheme_primitive("pln-bc", &PLNModule::pln_bc, this);

    // Define a scheme wrapper -- the scheme function pln-ar will
    // call the pln_ar method.
    define_scheme_primitive("pln-ar", &PLNModule::pln_ar, this);
#endif 

    // no longer done at module load - it would be inappropriate
    // for contexts other than testing PLN
    /*initTests();*
    Bstate = Btr<BITNodeRoot>(new BITNodeRoot(tests[0],
        new DefaultVariableRuleProvider, recordingTrails));
    printf("BITNodeRoot init ok\n");
    temp_state = Bstate.get();
    state = Bstate.get();*/
}

std::string PLNModule::do_pln(Request *dummy, std::list<std::string> args)
{
    std::string output = runCommand(args);
    return output;
}

/**
 * Specify the target Atom for PLN backward chaining inference.
 * Creates a new BIT for that Atom.
 * Runs steps steps of searching through the BIT.
 * Currently you can also use the cogserver commands on the resulting
 * BIT.
 */
Handle PLNModule::pln_bc(Handle h, int steps)
{
    return infer(h, steps, true);
}

Handle PLNModule::pln_ar(const std::string& ruleName,
                         const HandleSeq& premises) {
    return applyRule(ruleName, premises);
}


#if 0
// This isn't used, but should be replaced with BITrepository methods
void opencog::pln::setTarget(Handle h) {
    pHandleSeq fakeHandles = ((AtomSpaceWrapper*)ASW())->realToFakeHandle(h);
    pHandle fakeHandle = fakeHandles[0];
    Btr<vtree> target(new vtree(fakeHandle));
    
    Bstate.reset(new BITNodeRoot(target, new DefaultVariableRuleProvider,
                recordingTrails));

    printf("BITNodeRoot init ok\n");
    state = Bstate.get();
}
#endif

Handle opencog::pln::infer(Handle h, int &steps, bool setTarget)
{
    // Used by this function, and, if setTarget is true, assigned to the
    // corresponding state variables used by the PLN commands.
    Btr<BITNodeRoot> Bstate_;
    BITNodeRoot *state_;

    // Reuse the BIT if it's the same handle
    static Handle prev_h;

    // TODO make it work without setTarget, if necessary
    if (prev_h == h && setTarget) {
        Bstate_ = Bstate;
        state_ = state;
        printf("reusing BITNodeRoot\n");
    } else {
        prev_h = h;

        pHandleSeq fakeHandles = ASW()->realToFakeHandle(h);
        pHandle fakeHandle = fakeHandles[0];
        
//        Btr<vtree> target(new vtree(fakeHandle));
        
        Btr<vtree> target_(new vtree(fakeHandle));
        
        // The BIT uses real Links as a cue that it has already found the link,
        // so it is necessary to make them virtual
        Btr<vtree> target = ForceAllLinksVirtual(target_);

        bool recordingTrails = config().get_bool("PLN_RECORD_TRAILS");
        Bstate_.reset(new BITNodeRoot(target, new DefaultVariableRuleProvider,
                    recordingTrails, getFitnessEvaluator(PLN_FITNESS_BEST)));

        printf("BITNodeRoot init ok\n");

        state_ = Bstate_.get();
    }

    set<VtreeProvider*> result;
    pHandle eh;
    vhpair vhp;
    Handle ret = Handle::UNDEFINED;

    result = state_->infer(steps, 0.000001f, 1.00f); //, 0.000001f, 0.01f);
    state_->printResults();

    eh = (result.empty() ? PHANDLE_UNDEFINED :
                 _v2h(*(*result.rbegin())->getVtree().begin()));

    if (eh != PHANDLE_UNDEFINED ) {
        vhp = ASW()->fakeToRealHandle(eh);
        ret = vhp.first;
    }

    if (setTarget) {
        Bstate = Bstate_;
        state = state_;
    }
    
    return ret;
}

Handle opencog::pln::applyRule(const string& ruleName,
                               const HandleSeq& premises)
{
    static RuleProvider* rp = new DefaultVariableRuleProvider;
    const Rule* rule = rp->findRule(ruleName);
    if(rule) {
        pHandleSeq phs = ASW()->realToFakeHandles(premises);
        vector<Vertex> vv;
        copy(phs.begin(), phs.end(), back_inserter(vv));
        BoundVertex bv = rule->compute(vv, PHANDLE_UNDEFINED, false);
        vhpair vhp = ASW()->fakeToRealHandle(_v2h(bv.GetValue()));
        return vhp.first;
    }
    else return Handle::UNDEFINED;
}

//! @todo tacky hack implementation (refactor if it works)

// Now uses SCM. Won't search any alternative locations for the file.
void initAxiomSet(string premiseFile)
{
    AtomSpaceWrapper *atw = GET_ASW;
    atw->reset();
    atw->allowFWVarsInAtomSpace = true;

    //cprintf(-2,"loading...\n");
    std::cout << "loading " << premiseFile << std::endl;

#if 0
    atw->archiveTheorems = true;
    bool axioms_ok = atw->loadAxioms(premiseFile);
    if (!axioms_ok) throw std::string("failed to load file");
    atw->archiveTheorems = false;

#elif HAVE_GUILE
    int rc = load_scm_file(premiseFile.c_str());

    atw->archiveTheorems = true;
    atw->makeCrispTheorems();
    atw->archiveTheorems = false;

    // The error-checking is after the call to makeCrispTheorems, because
    // if there was an error, then the AS is now empty and makeCrispTheorems
    // will (correctly) record that there are none.
    if (rc) throw std::string("failed to load file");
#else
    throw std::string("Not allowed to use XML or SCM!");
#endif

    cprintf(-2,"%s loaded. Next test: ", premiseFile.c_str());
}

float getCount(float c)
{ return SimpleTruthValue::confidenceToCount(c); }


//! @todo Replace with an existing method, after tweaking various things to
//! make it suitable.
static SimpleTruthValue* parseSTV(const char* tvStr) {
    float mean, conf;
    sscanf(tvStr, "[%f,%f]", &mean, &conf);
    return new SimpleTruthValue(mean, getCount(conf));
}



//! @todo tacky hack implementation (refactor if it works)
void runSCMTarget(const char* test_name)
{
#ifdef HAVE_GUILE
	//!@ todo haxx
	FitnessEvaluatorT testFitnessEvaluator = BEST;

	// Find the path to the test file with that name.
	//!@ todo haxx
	std::string testDir = "../tests/reasoning/pln/targets/";
	std::string tmp = test_name + std::string("_test.conf");
	path conf_filename(tmp);
	std::string conf_file;

    for (recursive_directory_iterator end, dir(testDir);
          dir != end; ++dir) {
        //if (!is_directory(dir->status()) && dir->path() == conf_filename)
    	std::cout << dir->filename() << " " << conf_filename.filename() << std::endl;
    	//! @todo sigh. why doesn't this work?
    	//if (equivalent(dir->path(), conf_filename))
    	if (dir->path().filename() == conf_filename.filename())
            conf_file = dir->path().file_string();
    }

    //! @todo
    if (conf_file.empty()) return;

    Config test;
    std::cout << conf_file << std::endl;
    test.load(conf_file.c_str());
    //std::cout << test.to_string() << std::endl;

    std::cout << test["COMMENT"] << std::endl;

    //std::string axiomsFile = std::string(PLN_TEST_DIR)+test["LOAD"]+".xml";
    //! @todo define this somewhere else.
    std::string PLN_TEST_DIR = "../tests/reasoning/pln/";
    std::string axiomsFile = PLN_TEST_DIR+"scm/"+test["LOAD"]+".scm";

    initAxiomSet(axiomsFile);

    std::string targetScheme = test["TARGET"];
	std::cout << "Target: " << targetScheme << std::endl;

    // get target atom
    SchemeEval& eval = SchemeEval::instance();
    Handle h = eval.eval_h(targetScheme);

    pHandleSeq fakeHandles =
        ((AtomSpaceWrapper*)ASW())->realToFakeHandle(h);
    pHandle fakeHandle = fakeHandles[0];
    //meta target(new vtree(fakeHandle));

    meta target_(new vtree(fakeHandle));

    meta target = ForceAllLinksVirtual(target_);

    // Set parameters
    uint maxSteps = test.get_int("MAX_STEPS");
/*
    // Make sure FC doesn't take too long / only test it on simpler tests.
    // It takes disproportionately long once the AtomSpace gets large, due
    // to the indexes probably.
    if (!TESTING_BACKWARD_CHAINING && maxSteps > 201) {
        std::cout << "Skipping test due to time constraints" << std::endl;
        return; //minEvalsOfFittestBIT = 500;
    }
*/
    uint minEvalsOfFittestBIT = maxSteps/100;

    // Changed because we don't want to have to specify the count
    // as well, in the test's .conf file
    //TruthValue* minTV = SimpleTruthValue::fromString(test["MIN_TV"].c_str());
    //TruthValue* maxTV = SimpleTruthValue::fromString(test["MAX_TV"].c_str());
    TruthValue* minTV = parseSTV(test["MIN_TV"].c_str());
    TruthValue* maxTV = parseSTV(test["MAX_TV"].c_str());

    std::cout << "Loaded parameters of test OK" << std::endl;



    state = new BITNodeRoot(target, new DefaultVariableRuleProvider(), config().get_bool("PLN_RECORD_TRAILS"), testFitnessEvaluator);
    Bstate.reset(state);


/*    // Run the test using runPLNTest (via the maketest macro)
    maketest(target,
             minTV,
             maxTV,
             minEvalsOfFittestBIT,
             0); // Alternative (not used) method for limiting test time*/
#endif // HAVE_GUILE
}

//! Used by forward chainer 
struct compareStrength {
    // Warning, uses fake atomspace handles in comparison
    bool operator()(const pHandle& a, const pHandle& b) {
        return GET_ASW->getTV(a).getConfidence() >
            GET_ASW->getTV(b).getConfidence();
    }
};

void fw_beta (void) {
  AtomSpaceWrapper *atw = GET_ASW;

//  atw->reset();

  //ForwardChainerRuleProvider *rp=new ForwardChainerRuleProvider();
  const SimpleTruthValue tv(0.99,SimpleTruthValue::confidenceToCount(0.99));
#if 0
  pHandle h1=atw->addNode (CONCEPT_NODE,string("Human"),tv,true);
  pHandle h2=atw->addNode (CONCEPT_NODE,string("Mortal"),tv,true);
  pHandle h3=atw->addNode (CONCEPT_NODE,string("Socrates"),tv,true);
  std::vector<pHandle> p1(2),p2(2);
  p1[0]=h1; p1[1]=h2;
  p2[0]=h3; p2[1]=h1;
  pHandle L1=atw->addLink(ASSOCIATIVE_LINK,p1,tv,true);
  pHandle L2=atw->addLink(ASSOCIATIVE_LINK,p2,tv,true);
#endif

  ForwardChainer fw;

  // Load data from xml wordpairs file
//  atw->reset();
//  bool axioms_ok = atw->loadAxioms(std::string("wordpairs.xml"));
//  if (!axioms_ok) {
//      cout << "load failed" <<endl;
//      exit(1);
//  }
//  // Remove the dummy list link (added because the xml loader is silly)
//  Btr<set<pHandle> > ll = atw->getHandleSet(LIST_LINK, "", false);
//  foreach(pHandle l, *ll) {
//      atw->removeAtom(l);
//  }

#if 0
  cout << "FWBETA Adding handles to seed stack" << endl;
  //fw.seedStack.push_back(L1);
  //fw.seedStack.push_back(L2);
  //
// Push all links to seed stack in order of strength
// . get links
    Btr<set<pHandle> > linksSet = atw->getHandleSet(LINK, "", true);
    pHandleSeq links;
    copy(linksSet->begin(), linksSet->end(), back_inserter(links));
// . sort links based on strength
    std::sort(links.begin(), links.end(), compareStrength());
// . add in order
    foreach(pHandle l, links) {
        fw.seedStack.push_back(l);
    }
// Change prob of non seed stack selection to zero
//! @todo make set method so it will normalise probabilities.
  fw.probGlobal = 0.0f;
  fw.probStack = 1.0f;

  cout << "FWBETA adding to seed stack finished" << endl;
#endif
  cout << "FWBETA Starting chaining:" << endl;
  //pHandleSeq results = fw.fwdChainStack(10000);
  pHandleSeq results = fw.fwdChain(FWD_CHAIN_MAX_APPS);
  //pHandleSeq results = fw.fwdChain(10000);
  //opencog::logger().info("Finish chaining on seed stack");
  cout << "FWBETA Chaining finished, results:" << endl;
  NMPrinter np;
  foreach (pHandle h, results) {
      np(h);
  }
#ifdef NOW_REDUNDANT
  cout << "Removing results that just repeat existing links" << endl;
  int totalSize = results.size();
  pHandleSeq::iterator i,j;
  for (i = results.begin(); i != results.end(); i++) {
      vhpair v = atw->fakeToRealHandle(*i);
      if (! (v.second == NULL_VERSION_HANDLE)) {
          j = i;
          i--;
          results.erase(j);
      }
  }

  //HandleSeq results_realHandles = 
  
  cout << "Removed " << totalSize - results.size() << " results that just " <<
      "repeat existing links, " << results.size() << " results left." << endl;
#endif
  cout << results.size() << " results" << endl;

  Type t = SET_LINK;
  //pHandle setLink = atw->addLinkDC(t, results, tv, false, false);
  //Handle setLink = atomspace().addLink(t, results, tv);

  //TulipWriter tlp(std::string("fwd_chain_result.tlp"));
  //tlp.write(PHANDLE_UNDEFINED,-1,atw->fakeToRealHandle(setLink).first);
  
  
}

template <typename T>
T input(T& a, std::list<std::string>& args)
{
    std::stringstream ss;
    if (args.begin() != args.end()) {
        ss << args.front();
        args.pop_front();
    }
    ss >> a;
    
    return a;
}

std::string PLNModule::runCommand(std::list<std::string> args)
{
    AtomSpaceWrapper* asw = GET_ASW;
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

    // For result...
    std::stringstream ss;

    try {
        int a1T, a2T, bT, tempi=0;
        string a10, a11, a20, a21, b1, b2;
        vtree avt1, avt2, bvt;
        int qrule=0;
        Rule::MPs rule_args;
        bindingsT new_bindings;

        std::string temps;
        long h=0, h2=0;
        int j;
        int test_i=0;
        int s_i=0;
        bool axioms_ok;
        boost::shared_ptr<set<pHandle> > ts;
        Vertex v;
        Handle eh=Handle::UNDEFINED;

        // Command string
        string c;
        // Get command
        input(c, args);
        
        // Check whether command requires root to be set...
        set<string>::iterator rs_it;
        rs_it = setRequiresRoot.find(c);
        if (state == NULL && rs_it != setRequiresRoot.end()) {
            ss << "error: No root BIT. Set a target first?\n";
            return ss.str();
        }
        
        // Please keep these in the same order as the help
        // for the "pln" command!
        if (c == "log") {
            input(h, args); currentDebugLevel = (int) h;
            ss << "PLN log level now " << currentDebugLevel << endl;
        }
        else if (c == "record-trails") {
            recordingTrails = !recordingTrails;
            ss << "Recording trails now " << (recordingTrails?"ON":"OFF") << endl;
            state->setRecordingTrails(false);
        }
        else if (c == "infer") {
            // Give max nr of steps that we can take.
            input(h, args);
            j = (long) h;
            state->infer(j, 0.000001f, 0.01f);
            ss << state->printResults();
            ss << "\n" << j << " $ remaining.\n";
        }
        else if (c == "atom") {
            input(h, args);
            ss << printTree((pHandle)h,0,0);
        }
        else if (c == "plan") {
            // I'm not sure if it makes sense to have plan?
            // Looking at the code for extract plan, it uses a custom
            // "do_template" vtree that doesn't look very general.
            input(h, args);
            ss << state->extract_plan((pHandle)h);
        }
        else if (c == "trail") {
            input(h, args);
            
            ss << state->printTrail((pHandle)h);
        }
        // Pool commands
        else if (c == "pool") {
            ss << state->printFitnessPool();
        }
        else if (c == "pool-size") {
            ss << "Execution pool size = " << state->getExecPoolSize() << endl;
        }
        else if (c == "pool-fittest") {
            state->expandFittest();
            ss << "Expanded fittest, pool size now = " <<
                state->getExecPoolSize() << endl;
        }
        else if (c == "pool-expand") {
            input(h, args);
            for (int k=0;k<h;k++)
                state->expandFittest();
            ss << "Expanded fittest " << h << "times, pool size now = " <<
                state->getExecPoolSize() << endl;
        }
        // BIT commands
        else if (c == "bit") {
            input(h, args);
            if (h == 0) h = (long) state->children[0].begin()->prover;
            ss << ((BITNodeRoot*)h)->print(); 
        }
        else if (c == "bit-expand") {
            input(h, args);
            if (h == 0) h = (long) state;
            ((BITNodeRoot*)h)->expandNextLevel();
            ss << "Expanded BIT node " << h << ", pool size now = " <<
                state->getExecPoolSize() << endl;
        }
        else if (c == "bit-results") {
            input(h, args);
            if (h == 0) h = (long) state;
            ss << ((BITNodeRoot*)h)->printResults();
/*          foreach(const set<BoundVertex>& eval_res_set, ((BITNodeRoot*)h)->GetEvalResults())
            foreach(const BoundVertex& eval_res, eval_res_set)
                printTree(v2h(eval_res.value),0,-10);*/
        }
        else if (c == "bit-parents") {
            input(h, args);
            if (h == 0) h = (long)state;
            ss << "Parents of " << (long) h << ":" << endl;
            ss << ((BITNode*)h)->getBITRoot().printParents((BITNode*)h);
            //foreach(const parent_link<BITNode>& p, ((BITNode*)h)->getParents()) {
            //    ss << "User Node = " << (ulong) p.link << endl;
            //}
            // Use print_parents when it's changed to return a string
        }
        else if (c == "bit-rule-args") {
            input(h, args);
            if (h == 0) h = (long)state;
            ss << ((BITNodeRoot*)h)->printArgs();
        }
        else if (c == "bit-rule-target") { input(h, args);
            if (h == 0) h = (long)state;
            ss << "Target:\n";
            ss << ((BITNodeRoot*)h)->printTarget();
            ss << "Results:\n";
            ss << ((BITNodeRoot*)h)->printResults();
            ss << "Parent arg# "
               << ((BITNodeRoot*)h)->getParents().begin()->parent_arg_i
               << endl;
        }
        //! @todo work this out?
        else if (c == "bit-direct-results") {
            input(h, args);
            if (h == 0) h = (long)state;
            ss << "Disabled" << endl;
            /*cprintf(0, "Node has results & bindings:\n");
            foreach(const BoundVertex& bv, *((BITNodeRoot*)h)->direct_results)
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
            asw->reset();
            asw->archiveTheorems = true;
            axioms_ok = asw->loadAxioms(temps);
            asw->archiveTheorems = false;
            ss << (axioms_ok ? "Input file was loaded."
                    : "Input file was corrupt or missing.") << endl;
            ss << " Next you MUST (re)load a target atom with the "
               << "'pln test-target' command or set a target manually."
               << endl;
            // have to recreate the target vtrees to ensure that the
            // handles are correct after reloading axioms.
            initTests();
        }
        else if (c == "test-count") {
            ss << tests.size() << std::endl;
        }
        else if (c == "test-target") {
            input(test_i, args);
            Bstate.reset(new BITNodeRoot(tests[test_i], new DefaultVariableRuleProvider,
                        recordingTrails, fitnessEvaluator));
            state = Bstate.get();
            ss << "Test target set:" << endl;
            ss << rawPrint(*tests[test_i],tests[test_i]->begin(),0);
        }
        else if (c == "load-test") {
        	input(temps, args);
        	runSCMTarget(temps.c_str());
        }
        else if (c == "=") { input(h, args); input(h2, args); 
/*          cprintf(0, ((BITNodeRoot*)h)->eq((BITNodeRoot*)h2) ? "EQ\n" : "IN-EQ\n"); */
            ss << ( ((BITNodeRoot*)h)->eq((BITNodeRoot*)h2) ?
                "EQ" : "IN-EQ" ) << std::endl;
        }
        // Not recommended
        else if (c == "bit-next-level") { state->expandNextLevel(); }
        else if (c == "bit-eval") {
            try {
                state->evaluate();
            } catch(string s) {
                ss << s.c_str();
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

            ss << "B =";
            ss << rawPrint(bvt, bvt.begin(), -2);
            ss << "A1 =";
            ss << rawPrint(avt1, avt1.begin(), -2);
            ss << "A2 =";
            ss << rawPrint(avt2, avt2.begin(), -2);

            rule_args.clear();
            rule_args.push_back(BBvtree(new BoundVTree(avt1)));
            if (a2T)
                rule_args.push_back(BBvtree(new BoundVTree(avt2)));
            else
                ss << "Passing 1 arg.\n";
            ss << "BITNode " << (long) state->findNode((Rule*)qrule,
                    meta(new vtree(bvt)), rule_args, new_bindings) << ".\n" ;

        }
        else if (c == "loop-check") { state->loopCheck(); }
/*      else if (c == "W") {
            if (using_root) {
                temp_state = state;
                input(h, args);
                state = (BITNodeRoot*)h;
                using_root = false;
            }
            else {
                state = temp_state;
                using_root = true;
            }
        }*/
        else if (c == "fc") {
            fw_beta();
        }
        else if (c == "help") {
            ss << usageInfo;
        }
        else {
            ss << "Unknown PLN command: '" << c << "'.\n" <<
                "Use 'help pln' or 'pln help' for a list of valid commands." << endl;

        }
    } catch( std::exception& e )
    {
        cout << endl << "Exception: "
             << e.what() << endl;
    }
    return ss.str();
}

