#include "PLN.h"
#include "PLNUtils.h"
#include "BackInferenceTreeNode.h"
#include <string>
#include <sys/types.h>


namespace test {

/**
 * The tests load in various scm axiom sets and run until the target with a
 * specific confidence minimum has been reached (or max. nr of steps have been
 * exceeded, resulting in failure.)
 */
struct PLNTest
{
    meta target;
    TruthValue* minTV, *maxTV;

    uint minEvalsOfFittestBIT; //Divided by ten
    uint minExhaustiveEvals; //Use either this one or the previous one
    PLNTest(meta _target,
            Handle _target_handle,
            TruthValue* _minTV,
            TruthValue* _maxTV,
            uint _minEvalsOfFittestBIT,
            uint _minExhaustiveEvals)
        : target(_target), minTV(_minTV), maxTV(_maxTV),
          minEvalsOfFittestBIT(_minEvalsOfFittestBIT),
          minExhaustiveEvals(_minExhaustiveEvals),
          target_handle(_target_handle)
    {}
    Btr<opencog::pln::BITNodeRoot> state;
    
    // The normal Handle. Only used by Python PLN.
    Handle target_handle;
};

float getCount(float c);

void initAxiomSet(std::string premiseFile);
/** Does 1000 runs per expansion phase
 * @param test_bc is a flag indicating whether to run the test
 * with the backward chainer (true) or the forward chainer (false).
 * @return whether the test passed or not.
 */
bool runPLNTest(Btr<PLNTest> t, bool test_bc = true);
bool runPLNTest_CPP(Btr<PLNTest> t, bool test_bc = true);
//! @todo This next function doesn't need to have test_bc
Btr<PLNTest> setupSCMTarget(std::string conf_file, bool test_bc = true);
Btr<PLNTest> findSCMTarget(std::string test_name, bool test_bc = true);
/** Test all Scheme targets (i.e. in *_test.conf files).
 * @param The directory containing the tests.
 * @param Whether to test with the backward chainer (true) or the forward chainer (false).
 * @return Whether all of the tests passed (true) or not (false).
 */
bool runSCMTargets(std::string testDir, bool test_bc = true);

bool maketest(meta target,
              TruthValue* minTV,
              TruthValue* maxTV,
              uint minEvalsOfFittestBIT,
              uint minExhaustiveEvals,
              bool test_bc = true);

}//}}
