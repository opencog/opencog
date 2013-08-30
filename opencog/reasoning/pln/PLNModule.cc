/*
 * opencog/reasoning/pln/PLNModule.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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
#include "Testing.h"
#include "rules/Rules.h"
#include "rules/RuleProvider.h"
#include "AtomSpaceWrapper.h"
#include "BackInferenceTreeNode.h"

#include <opencog/guile/SchemePrimitive.h>
#include <opencog/util/Logger.h>
#include <opencog/util/Config.h>
#include <opencog/util/macros.h>

#include <boost/foreach.hpp>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#include "TestTargets.h"
#include "ForwardChainer.h"

using namespace opencog;
using namespace opencog::pln;
using namespace test;

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
    "- some take BITNode IDs. )\n"
    "\n"
    "---\n"
    " log <[-5..5]> - Set log level (0 = normal log level).\n"
    " record-trails - Switch the recording of inference trails ON/OFF (default: ON)\n"
    " bc|infer <s>  - Infer (backchaining) until result found with conf > 0.01 Or 's' inference steps\n"
    "                 have been taken.\n"
	" fc <s>        - Forward chain until 's' inference steps have been taken.\n"
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
	" test <test name> - Load a test, including the dataset and target\n"
	" list-tests - List all the tests available\n"
    " test-count         - count the number of pre-defined inference targets (obsolete)\n"
//    " test-target <n>    - Load in a new pre-defined target #n (from TestTargets.h) (obsolete)\n"
    " = <n1> <n1>        - Check if BIT nodes n1 and n2 are equal.\n"
    "\n"
    "--- The following are not recommended unless you know what your doing:\n"
    " bit-next-level     - Expand the tree's whole next level (usually not recommended)\n"
    " bit-eval           - Manually evaluate the current tree (usually not recommended)\n"
    " bit-find-node  bT b1 b2  a1T a10 a11 [a2T a20 a21] <Rule ptr> - find a BITNode \n"
    "                 for the given rule with the given parameters. 3rd parameter depends \n"
    "                 on rule number, but must be specified (needs to be more friendly).\n"
    " loop-check         - check for loops\n";


PLNModule::PLNModule(CogServer& cs) : Module(cs)
{
    logger().info("[PLNModule] constructor");
    setParameters(DEFAULT());
    do_pln_register();  
    _cogserver.registerAgent(BackChainingAgent::info().id, &backChainingFactory);
    _cogserver.registerAgent(ForwardChainingAgent::info().id, &forwardChainingFactory);
}

void PLNModule::setParameters(const std::string* params) {
    for (unsigned int i = 0; params[i] != ""; i += 2) {
        if (!config().has(params[i])) {
           config().set(params[i], params[i + 1]);
        }
    }
}

PLNModule::~PLNModule()
{
    logger().info("[PLNModule] destructor");
    do_pln_unregister();
    _cogserver.unregisterAgent(BackChainingAgent::info().id);
    _cogserver.unregisterAgent(ForwardChainingAgent::info().id);
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
    AtomSpaceWrapper* asw = ASW(&_cogserver.getAtomSpace());
#if LOCAL_ATW
    ((LocalATW*)asw)->SetCapacity(10000);
#endif  
    asw->archiveTheorems = true;
    asw->allowFWVarsInAtomSpace = 
        config().get_bool("PLN_FW_VARS_IN_ATOMSPACE");

    // Initialise ruleprovider (because it needs to monitor atom add/remove)
    RuleProvider& rp = referenceRuleProvider();
	OC_UNUSED(rp);
	
#ifdef HAVE_GUILE
    // Define a scheme wrapper -- the scheme function pln-bc will
    // call the pln_bc method.
    define_scheme_primitive("pln-bc", &PLNModule::pln_bc, this);

    // Define a scheme wrapper -- the scheme function pln-ar will
    // call the pln_ar method.
    define_scheme_primitive("pln-ar", &PLNModule::pln_ar, this);
#endif 
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
    cout << "pln_bc" << h << endl;

    return infer(h, steps, true);
}

Handle PLNModule::pln_ar(const std::string& ruleName, const HandleSeq& premises,
                         const HandleSeq& CX) {
    OC_ASSERT(CX.empty() || CX.size() == 1);
    return applyRule(ruleName, premises, (CX.empty()? Handle::UNDEFINED: CX[0]));
}

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
        
        Btr<vtree> target_(new vtree(fakeHandle));
        
        // The BIT uses real Links as a cue that it has already found the link,
        // so it is necessary to make them virtual
        Btr<vtree> target = ForceAllLinksVirtual(target_);

        bool recordingTrails = config().get_bool("PLN_RECORD_TRAILS");
        Bstate_.reset(new BITNodeRoot(target, &referenceRuleProvider(),
                    recordingTrails, getFitnessEvaluator(PLN_FITNESS_BEST)));

        printf("BITNodeRoot init ok\n");

        state_ = Bstate_.get();
    }

    set<VtreeProvider*> result;
    pHandle eh;
    vhpair vhp;
    Handle ret = Handle::UNDEFINED;

    state_->setLoosePoolPolicy(true);

    result = state_->infer(steps, 0.000001f, 0.01f); //, 0.000001f, 1.00f);
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

// Assuming s == prefix.append(string(Handle)), replace string(Handle)
// by string(pHandle)
inline void replaceHandleBypHandle(const string& prefix, string& s, Handle CX) {
    const unsigned int hpos = prefix.size();
    Handle h(boost::lexical_cast<UUID>(s.substr(hpos)));
    pHandle ph = ASW()->realToFakeHandle(h, (CX == Handle::UNDEFINED?
                                             NULL_VERSION_HANDLE
                                             :VersionHandle(CONTEXTUAL, CX)));
    s.replace(hpos, s.size(), boost::lexical_cast<string>(ph));
}

void opencog::pln::correctRuleName(string& ruleName, Handle CX)
{
    if(ruleName.find(ForAllInstantiationRulePrefixStr) == 0)
        replaceHandleBypHandle(ForAllInstantiationRulePrefixStr, ruleName, CX);
    else if (ruleName.find(AverageInstantiationRulePrefixStr) == 0)
        replaceHandleBypHandle(AverageInstantiationRulePrefixStr, ruleName, CX);
}

Handle opencog::pln::applyRule(string ruleName, const HandleSeq& premises,
                               Handle CX)
{
#ifndef CONTEXTUAL_INFERENCE
    OC_ASSERT(CX == Handle::UNDEFINED, "Contextual inference is not allowed");
#endif

    correctRuleName(ruleName, CX);
    RuleProvider& rp = referenceRuleProvider();
    RulePtr rule = rp.findRule(ruleName);

    // result to be overwriten
    vhpair vhp(Handle::UNDEFINED, NULL_VERSION_HANDLE);

    OC_ASSERT(rule.get(), "Apparently the rule %s is undefined", ruleName.c_str());

    pHandleSeq phs = ASW()->realToFakeHandles(premises, CX);
    if(rule->isComposer()) {
        VertexSeq vv(phs.begin(),phs.end());
        //! @todo usually use the version of compute that takes BoundVertexes
        BoundVertex bv = rule->compute(vv, PHANDLE_UNDEFINED, false);
        vhp = ASW()->fakeToRealHandle(_v2h(bv.GetValue()));
    } else { // generator
        // here it is assumed that there is only one premise and
        // one corresponding pHandle (because context are ignored
        // for now)
        OC_ASSERT(phs.size() == 1);
        meta target = meta(new vtree(make_vtree(phs[0])));
        // since the target vtree corresponds a specific pHandle,
        // directResult must have only one element
        Btr<set<BoundVertex> > directResult = 
            rule->attemptDirectProduction(target, false);
        vhp = ASW()->fakeToRealHandle(_v2h(directResult->begin()->GetValue()));
    }
    // std::cout << "VHP = " << vhp << std::endl;
    // std::cout << "VHP.Handle = " << ASW()->getAtomSpace()->atomAsString(vhp.first, false) << std::endl;
    return vhp.first;
}

//! Used by forward chainer 
struct compareStrength {
    // Warning, uses fake atomspace handles in comparison
    bool operator()(const pHandle& a, const pHandle& b) {
        return GET_ASW->getConfidence(a) >
            GET_ASW->getConfidence(b);
    }
};

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

/// Get the corresponding BITNode, or if id == 0, get the current BITNodeRoot.
/// (Or actually, its first root variable scoper).
BITNode* getBITOrScoper(BITNodeID id) {
    BITNode* bitnode;
    if (id == 0) {
        if (state)
            bitnode = (BITNode*)state->children[0].begin()->prover;
        else
            throw InvalidParamException(TRACE_INFO, "No current BIT");
    }
    else bitnode = haxx::getBITNode(id);
    return bitnode;
}

BITNode* getBITOrRoot(BITNodeID id) {
    BITNode* bitnode;
    if (id == 0) {
        if (state)
            bitnode = (BITNode*)state;
        else
            throw InvalidParamException(TRACE_INFO, "No current BIT");
    }
    else bitnode = haxx::getBITNode(id);
    return bitnode;
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
        int a1T, a2T, bT;
        string a10, a11, a20, a21, b1, b2;
        vtree avt1, avt2, bvt;
        Rule::MPs rule_args;
        bindingsT new_bindings;

        std::string temps;
        long h=0, h2=0;
        int j;
        bool axioms_ok;
        boost::shared_ptr<set<pHandle> > ts;
        Vertex v;
        Handle eh=Handle::UNDEFINED;
        BITNode* bitnode;

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
        else if (c == "bc" || c == "infer") {
            // Give max nr of steps that we can take.
            input(h, args);
            j = (long) h;
            state->infer(j, 0.000001f, 0.01f);
            ss << state->printResults();
            ss << "\n" << j << " $ remaining.\n";
        }
        else if (c == "fc") {
			int steps;
			input(steps, args);

			HybridForwardChainer fw;
			cout << "FC Starting chaining:" << endl;
			//pHandleSeq results = fw.fwdChainStack(10000);
			pHandleSeq results = fw.fwdChain(steps);
			cout << "FC Chaining finished, results:" << endl;
			NMPrinter np;
			foreach (pHandle h, results) {
			  np(h,-5);
			}
			cout << results.size() << " results" << endl;
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
            ss << getBITOrScoper(h)->print();
        }
        else if (c == "bit-expand") {
            input(h, args);
            getBITOrRoot(h)->expandNextLevel();
            ss << "Expanded BIT node " << h << ", pool size now = " <<
                state->getExecPoolSize() << endl;
        }
        else if (c == "bit-results") {
            input(h, args);
            ss << getBITOrRoot(h)->printResults();
        }
        else if (c == "bit-parents") {
            input(h, args);
            bitnode = getBITOrRoot(h);
            ss << "Parents of " << (long) h << ":" << endl;
            ss << bitnode->getBITRoot().printParents(bitnode);
            // Use print_parents when it's changed to return a string
        }
        else if (c == "bit-rule-args") {
            input(h, args);
            bitnode = getBITOrRoot(h);
            ss << bitnode->printArgs();
        }
        else if (c == "bit-rule-target") { input(h, args);
            bitnode = getBITOrRoot(h);
            ss << "Target:\n";
            ss << bitnode->printTarget();
            ss << "Results:\n";
            ss << bitnode->printResults();
            ss << "Parent arg# "
               << bitnode->getParents().begin()->parent_arg_i
               << endl;
        }
        //! @todo work this out?
        else if (c == "bit-direct-results") {
            input(h, args);
            if (h == 0) h = (long)state;
            ss << "Disabled" << endl;
#if 0
            cprintf(0, "Node has results & bindings:\n");
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
            }
#endif
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
        else if (c == "test") {
        	input(temps, args);
        	findSCMTarget(temps);
        }
        // How does this work? Comparing two BITNodeRoots?
        else if (c == "=") { input(h, args); input(h2, args);
            BITNodeRoot* bit1 = dynamic_cast<BITNodeRoot*>(getBITOrRoot(h));
            BITNodeRoot* bit2 = dynamic_cast<BITNodeRoot*>(getBITOrRoot(h));
/*          cprintf(0, ((BITNodeRoot*)h)->eq((BITNodeRoot*)h2) ? "EQ\n" : "IN-EQ\n"); */
            ss << ( bit1->eq(bit2) ?
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
            string qrule;
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
            RulePtr rp = referenceRuleProvider().findRule(qrule);
            ss << "BITNode " << (long) state->findNode(rp,
                    meta(new vtree(bvt)), rule_args, new_bindings) << ".\n" ;

        }
        else if (c == "loop-check") { state->loopCheck(); }
        else if (c == "help") {
            ss << usageInfo;
        }
        else {
            ss << "Unknown PLN command: '" << c << "'.\n" <<
                "Use 'help pln' or 'pln help' for a list of valid commands." << endl;

        }
    } catch(std::exception& e)
    {
        ss << endl << "Exception: " << e.what() << endl;
        cout << ss.str();
    }
    return ss.str();
}

