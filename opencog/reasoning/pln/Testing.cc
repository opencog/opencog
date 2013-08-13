#include "Testing.h"

#include "PLNModule.h"
#include "PLN.h"
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <opencog/util/Config.h>
#include <opencog/util/exceptions.h>

#include <opencog/guile/SchemeEval.h>
#include <opencog/guile/load-file.h>

#ifdef HAVE_CYTHON
#include <opencog/cython/PyIncludeWrapper.h>
#include <opencog/cython/logic_wrapper_api.h>
#endif

#include <boost/filesystem.hpp>

#include <string>

#define DPRINTF(...)
//#define DPRINTF printf

using namespace std;
using namespace opencog;
using namespace opencog::pln;
using namespace boost::filesystem;

// state variables for running multiple PLNShell commands.
extern Btr<BITNodeRoot> Bstate;
extern BITNodeRoot* temp_state, *state;

namespace test
{
    extern FILE *logfile;
    extern double custom_duration;
    extern double custom_duration2;
}

namespace test {

FitnessEvaluatorT testFitnessEvaluator;

// the following is used for test statistics
vector< vector<vector<int> > >  INstatsVT;
vector<vector<int> > INstatsV;
vector<int> INstats;
//! A list of test names that fail, to be printed out at the end of testing
vector<string> failedSCMTargets;
int allTestsInferenceNodes=0;
int allTestsExpansions=0;

int tests_passed=0;
int tests_total=0;

float getCount(float c) { return SimpleTruthValue::confidenceToCount(c); }

/** Resets AtomSpaceWrapper (and indrectly the AtomSpace) and loads premiseFile
 * in scm format.
 *
 * @param premiseFile exact relative or full path for file to load.
 */
void initAxiomSet(string premiseFile)
{
    AtomSpaceWrapper *asw = GET_ASW;
    DPRINTF("initAxiomSet: Before reset\n");
    asw->reset();
    DPRINTF("initAxiomSet: After reset\n");
    asw->allowFWVarsInAtomSpace = true;

    DPRINTF("Loading %s\n", premiseFile.c_str());

#if HAVE_GUILE
    int rc = load_scm_file(*(asw->getAtomSpace()), premiseFile.c_str());

    asw->archiveTheorems = true;
    asw->makeCrispTheorems();
    asw->archiveTheorems = false;

    // The error-checking is after the call to makeCrispTheorems, because
    // if there was an error, then the AS is now empty and makeCrispTheorems
    // will (correctly) be updated to indicate that there are none.
    if (rc) throw RuntimeException(TRACE_INFO, "failed to load file");

    cprintf(-2,"%s loaded. Next test: ", premiseFile.c_str());
#else
    throw RuntimeException(TRACE_INFO, "Need Scheme bindings to run PLN tests!");
#endif
}

//! @todo Replace with an existing method, after tweaking various things to
//! make it suitable.
static SimpleTruthValue* parseSTV(const char* tvStr) {
    float mean, conf;
    sscanf(tvStr, "[%f,%f]", &mean, &conf);
    return new SimpleTruthValue(mean, getCount(conf));
}

Btr<PLNTest> setupSCMTarget(std::string conf_file, bool test_bc)
{
#ifdef HAVE_GUILE
    //!@ todo haxx
    FitnessEvaluatorT testFitnessEvaluator = BEST;

    Config test;
    std::cout << conf_file << std::endl;
    test.load(conf_file.c_str());

    std::cout << test["COMMENT"] << std::endl;

    //! @todo define this somewhere else.
    std::string PLN_TEST_DIR = PROJECT_SOURCE_DIR"/tests/reasoning/pln/";
    // Load axioms from scm file
    std::string axiomsFile = PLN_TEST_DIR+"scm/"+test["LOAD"]+".scm";

    initAxiomSet(axiomsFile);

    std::string targetScheme = test["TARGET"];
    std::cout << "Target: " << targetScheme << std::endl;

    // Get target atom
    SchemeEval& eval = SchemeEval::instance();
    Handle h = eval.eval_h(targetScheme);
    if (h == Handle::UNDEFINED || eval.eval_error()) {
        std::cout << "Scheme error while evaluating target: " << targetScheme << std::endl;
        throw RuntimeException(TRACE_INFO,"Scheme error while evaluating target");
    }

    // Get the fake handle for the primary TV of the target
    pHandleSeq fakeHandles =
            ((AtomSpaceWrapper*)ASW())->realToFakeHandle(h);
    // Index 0 is the primary
    pHandle fakeHandle = fakeHandles[0];

    meta target_(new vtree(fakeHandle));
    meta target = ForceAllLinksVirtual(target_);

    // Set parameters
    uint maxSteps = test.get_int("MAX_STEPS");
    uint minEvalsOfFittestBIT = maxSteps/100;

    TruthValue* minTV = parseSTV(test["MIN_TV"].c_str());
    TruthValue* maxTV = parseSTV(test["MAX_TV"].c_str());

    std::cout << "Loaded parameters of test OK" << std::endl;

    state = new BITNodeRoot(target, NULL, config().get_bool("PLN_RECORD_TRAILS"), testFitnessEvaluator);
    Bstate.reset(state);

    Btr<PLNTest> the_test(new PLNTest(
               target,
               h,
               minTV, maxTV,
               minEvalsOfFittestBIT,
               0)); // Alternative for using exhaustive evaluation (obsolete).
    // Even with FC, we want to have a BIT available so that printTrail can be accessed
    the_test->state = Bstate;
    return the_test;

#endif // HAVE_GUILE
}

/**
 * @param test_bc is a flag indicating whether to run the tests
 * with the backward chainer (true) or the forward chainer (false)
 */
bool runSCMTargets(string testDir, bool test_bc) {
    // TODO maybe use regex_iterator

    testDir+= "targets/";
    if (!test_bc) testDir += "both";

    // This method is the main one for current and future tests.
    // Also, there are a lot of different errors that can happen.
    // So, it has/needs somewhat elaborate exception-handling.
    // This is partly because CXXTest doesn't give you any info
    // about an exception, just telling you it happened.
    try
    {

        // TODO apparently there is a way to make iterators only cover
        // either files or directories. The boost::filesystem
        // documentation is very obsolete however.
        for (recursive_directory_iterator end, dir(testDir);
              dir != end; ++dir) {
            // ignore directories and hidden files beginning with "."
            if (!is_directory(dir->status()) &&
                    dir->path().filename().string()[0] != '.') {
                string filename(dir->path().string());
                Btr<PLNTest> _test = setupSCMTarget(filename.c_str(), test_bc);
                if (_test) {
                    // Run the C++ PLN backward chainer first, for comparison.
                    cout << "Running C++ backward chainer for comparison" << endl;
                    runPLNTest_CPP(_test, test_bc);
 
#ifdef HAVE_CYTHON
                    _test = setupSCMTarget(filename.c_str(), test_bc);
                    cout << "Running Python backward chainer" << endl;
                    if (!runPLNTest(_test, test_bc))
                        failedSCMTargets.push_back(filename);
#endif /* HAVE_CYTHON */
                }
            }
        }
    }
    catch(std::string s) {
        std::cerr << "testSCMTargets:" << s;
        throw s;
    }
    catch(StandardException e) {
        std::cerr << "testSCMTargets:" << e.getMessage();
        throw e;
    }
    catch(PLNexception e)
    {
        std::cerr << "testSCMTargets:" << e.what();
        throw e;
    }
    catch(std::exception& e) 
    {
        std::cerr << "Unknown exception during SCM targets" << std::endl;
        throw;
    }

    //! @todo May get some interference from other uses of runPLNTest in PLNUTest.cxxtest.
    if (tests_passed==tests_total) {
        cout << "Passed all " << tests_total << " tests!" << endl;
        return true;
    } else {
        cout << "Failed " << (tests_total - tests_passed) << " out of "
             << tests_total << " tests." << endl;
        vector<string>::const_iterator cit = failedSCMTargets.begin();
        cout << "Tests failed:" << endl;
        for (; cit != failedSCMTargets.end(); cit++) {
            cout << "  " << *cit << endl;
        }
        return false;
    }
}

Btr<PLNTest> findSCMTarget(std::string test_name, bool test_bc) {
    // Find the path to the test file with that name.
    std::string testDir = PROJECT_SOURCE_DIR"/tests/reasoning/pln/targets/";
    std::string tmp = test_name + std::string("_test.conf");
    path conf_filename(tmp);

    std::string conf_file;

    for (recursive_directory_iterator end, dir(testDir);
            dir != end; ++dir) {
        //if (!is_directory(dir->status()) && dir->path() == conf_filename)
        //      std::cout << dir->filename() << " " << conf_filename.filename() << std::endl;
        //! @todo sigh. why doesn't this work?
        //if (equivalent(dir->path(), conf_filename))
        if (dir->path().filename() == conf_filename.filename())
            conf_file = dir->path().filename().string();
    }

    //! @todo
    if (conf_file.empty()) return Btr<PLNTest>();
    else return setupSCMTarget(conf_file, test_bc);
}

bool runPLNTest(Btr<PLNTest> t, bool test_bc)
{
#ifdef HAVE_CYTHON
    clock_t start, finish;
    double duration;

    test::custom_duration = 0.0;
    start = clock();

    // Code to run the Python backward chainer.
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure(); 
    import_logic_wrapper();
    //python_pln_fc(cogserver().getAtomSpace());
    //python_pln_fc();
    Handle eh = python_pln_bc(&cogserver().getAtomSpace(), t->target_handle);
    PyGILState_Release(gstate); 

    TruthValuePtr etv = cogserver().getAtomSpace().getTV(eh, NULL_VERSION_HANDLE);

    if (*etv != TruthValue::NULL_TV()) {
        /* Print resulting truth value compared to test requirements */
        printf("c: %f min: %f\n", etv->getConfidence(),
                t->minTV->getConfidence());
        printf("s: %f min: %f\n", etv->getMean(),
                t->minTV->getMean());
        printf("c: %f max: %f\n", etv->getConfidence(),
                t->maxTV->getConfidence());
        printf("s: %f max: %f\n", etv->getMean(),
                t->maxTV->getMean());

        // TODO: print a trail
    }

    /* Check whether resulting truth value meets test requirements */
    bool passed = (
        eh != Handle::UNDEFINED &&
        etv &&
        etv->getConfidence() >= t->minTV->getConfidence() &&
        etv->getMean()       >= t->minTV->getMean() &&
        etv->getConfidence() <= t->maxTV->getConfidence() &&
        etv->getMean()       <= t->maxTV->getMean()
    );

    if (passed) {
        tests_passed++;

        printf("\n"
               "**********************************************\n"
               "passed: %s.\n"
               "**********************************************\n",
            (etv?etv->toString().c_str():"(null TV)"));
    }
    else {
        printf("\n**********************************************\n"
               "FAILED: %s!\n"
               "**********************************************\n",
        (etv?etv->toString().c_str():"(null TV)"));
    }

    // Still want to show this on failed tests
    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    printf( "Test took %2.2f seconds TOTAL.\n", duration );

    tests_total++;
    
    cout << "So far, failed " << (tests_total - tests_passed) << " out of "
         << tests_total << " tests." << endl;

    return passed;
#else /* HAVE_CYTHON */
    return false;
#endif /* HAVE_CYTHON */
}

bool runPLNTest_CPP(Btr<PLNTest> t, bool test_bc)
{
    AtomSpaceWrapper *asw = GET_ASW;
    stats::Instance().ITN2atom.clear();
    
    rawPrint(*t->target, t->target->begin(), -2);

    clock_t start, finish;
    double duration;

    test::custom_duration = 0.0;
    start = clock();

    asw->allowFWVarsInAtomSpace = true;

    fflush(stdout);

    HybridForwardChainer fc;

    uint s_i=0; // Expansion phase #
    pHandle eh=PHANDLE_UNDEFINED; // expected target handle
    TruthValuePtr etv; // expect target handle TV
    bool passed=false;

    set<VtreeProvider*> eres;

    if (test_bc)
        t->minEvalsOfFittestBIT *= 100; //Minimum "resolution"
    else
        // 12 is the maximum depth the BIT reaches (for now), and each "step"
        // of FC now does one level
        t->minEvalsOfFittestBIT = 7;

    const int expansions_per_run = test_bc ? 1000 : 1;
    int total_expansions = 0;

    if (t->minEvalsOfFittestBIT > 0) {
        do {
/*              for (int k=0;k<expansions_per_run;k++)
                t->state->expandFittest();

            eres = t->state->evaluate();*/

            cprintf(-3, "\n    Evaluating...\n");

            int expansions = expansions_per_run;

            if (test_bc) {
                eres = t->state->infer(expansions, 0.000001f, 0.001f);
            } else {
                eh = fc.fwdChainToTarget(expansions, (t->target));
            }

            total_expansions += expansions_per_run - expansions;

            if (expansions > 0)
                cprintf(-3, "Succeeded. Saved $%d / $%d (from the "
                        "beginning of the cycle).\n", expansions,
                        expansions_per_run);
            else
                cprintf(2, "Failed for now... Saved $%d / $%d (from the "
                        "beginning of the cycle).\n", expansions,
                        expansions_per_run);

            if (test_bc)
                eh = (eres.empty() ? PHANDLE_UNDEFINED :
                        _v2h(*(*eres.rbegin())->getVtree().begin()));

            if (eh != PHANDLE_UNDEFINED )
                etv = asw->getTV(eh);

            if (etv) {
                /* Print resulting truth value compared to test requirements */
                printf("c: %f min: %f\n", etv->getConfidence(),
                        t->minTV->getConfidence());
                printf("s: %f min: %f\n", etv->getMean(),
                        t->minTV->getMean());
                printf("c: %f max: %f\n", etv->getConfidence(),
                        t->maxTV->getConfidence());
                printf("s: %f max: %f\n", etv->getMean(),
                        t->maxTV->getMean());

                // This uses the BIT, but the trails are stored using some
                // global variables so work in FC. Disabled due to an infinite
                // recursion issue in the trails (as of Nov2009) -- JaredW
                if (test_bc) {
	                std::cout << "Inference trail: " << std::endl;
          	        t->state->printTrail(eh);
      	        }
            }

            /* Check whether resulting truth value meets test requirements */
            passed = (
                eh != PHANDLE_UNDEFINED &&
                etv &&
                etv->getConfidence() >= t->minTV->getConfidence() &&
                etv->getMean()       >= t->minTV->getMean() &&
                etv->getConfidence() <= t->maxTV->getConfidence() &&
                etv->getMean()       <= t->maxTV->getMean()
            );

            cprintf(-4, "TEST Expansion phase %d over.\n", s_i);
        }
        while ((++s_i)*expansions_per_run < t->minEvalsOfFittestBIT
                && !passed);
    }
    else if (t->minExhaustiveEvals > 0) {
        assert(0);
        // This should be updated to reflect the new BITNode interface
        /*
        for (uint L=0;L<t->minExhaustiveEvals;L++)
            t->state->expandNextLevel();

        eres = t->state->evaluate();
        eh = (eres.empty() ? NULL : v2h(eres.rbegin()->value));
        if (eh) {
            if (etv != NULL) delete etv;
            etv = asw->TV(eh).clone();
        }

        passed = (eh && etv &&
            etv->getConfidence() > t->minTV->getConfidence() &&
            etv->getMean()          > t->minTV->getMean()
            );*/
    }
    else
        puts("ERROR IN TEST SETTINGS");

    if (passed) {
        printf("\n"
               "**********************************************\n"
               "passed: %s.\n"
               "**********************************************\n",
            (etv?etv->toString().c_str():"(null TV)"));
    }
    else {
        printf("\n**********************************************\n"
               "FAILED: %s!\n"
               "**********************************************\n",
        (etv?etv->toString().c_str():"(null TV)"));
    }

    // Still want to show this on failed tests
    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    printf( "Test took %2.2f seconds TOTAL.\n", duration );

    printf( "Custom test time was %3.3f seconds.\n",
            test::custom_duration );
    printf( "Custom test time was %3.3f seconds.\n",
            test::custom_duration2 );

    if (test_bc) {
        printf("Test results: [");

        foreach(VtreeProvider* bv, eres) {
            TruthValuePtr tv = asw->getTV(vt2h(*bv));
            if (!tv->isNullTv() && tv->getConfidence()>0.0001f)
                printf("%d ", vt2h(*bv));
        }
        printf("]\n");

    }

    if (passed) {
        tests_passed++;
    }
    else
        INstats.push_back(0);

    //! @todo Decide whether this stuff should only be done if it passes (like before)
    allTestsInferenceNodes += t->state->inferenceNodes;
    allTestsExpansions += total_expansions;

    INstats.push_back(t->state->inferenceNodes);

    cout << "Total expansion steps: " << total_expansions <<
            "(" << allTestsExpansions << " in all tests)";
    cout << endl << "Exec pool size: " << t->state->getExecPoolSize();
    cout << endl << "InferenceNodes: " << t->state->inferenceNodes <<
            " (" << allTestsInferenceNodes << " in all tests)" << endl;


    tests_total++;

    //stats::Instance().print(stats::triviality_filterT());

    cout << "So far, failed " << (tests_total - tests_passed) << " out of "
         << tests_total << " tests." << endl;

#if WAIT_KEY_ON_FAILURE
    if (!passed)
        getc(stdin);
#endif
    return passed;
}

// helper for runPLNTest
bool maketest(meta target,
              TruthValue* minTV,
              TruthValue* maxTV,
              uint minEvalsOfFittestBIT,
              uint minExhaustiveEvals,
              bool test_bc) {
    Btr<PLNTest> t = Btr<PLNTest>(new PLNTest(target,
                                        Handle::UNDEFINED,
                                        minTV,
                                        maxTV,
                                        minEvalsOfFittestBIT,
                                        minExhaustiveEvals)
                                        );
    t->state = Btr<BITNodeRoot>(new BITNodeRoot(target, NULL, config().get_bool("PLN_RECORD_TRAILS"), testFitnessEvaluator));
    Bstate.reset(state);
    state = Bstate.get();
    return runPLNTest(t, test_bc);
}

}
