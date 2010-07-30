#include "Testing.h"

#include "PLNModule.h"
#include "PLN.h"
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/util/Config.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/load-file.h>
#include <boost/filesystem.hpp>
#include <string>

using namespace std;
using namespace opencog;
using namespace opencog::pln;
using namespace boost::filesystem;

// state variables for running multiple PLNShell commands.
extern Btr<BITNodeRoot> Bstate;
extern BITNodeRoot* temp_state, *state;

namespace opencog {
namespace pln {
namespace test {

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

void runSCMTarget(const std::string test_name)
{
#ifdef HAVE_GUILE
    //!@ todo haxx
    FitnessEvaluatorT testFitnessEvaluator = BEST;

    // Find the path to the test file with that name.
    //!@ todo haxx
    std::string testDir = "../tests/reasoning/pln/targets/";
    std::string tmp = test_name + std::string("_test.conf");
    path conf_filename(tmp);

    std::cout << path();

    std::string conf_file;

    for (recursive_directory_iterator end, dir(testDir);
            dir != end; ++dir) {
        //if (!is_directory(dir->status()) && dir->path() == conf_filename)
        //      std::cout << dir->filename() << " " << conf_filename.filename() << std::endl;
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

    //! @todo define this somewhere else.
    std::string PLN_TEST_DIR = "../tests/reasoning/pln/";
    // scm is now used instead of xml
    // std::string axiomsFile = PLN_TEST_DIR+"xml/"+test["LOAD"]+".xml";
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

}}}
