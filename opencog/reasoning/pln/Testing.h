#include "PLN.h"
#include "PLNUtils.h"
#include <string>

//namespace opencog {
//namespace pln {
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
            TruthValue* _minTV,
            TruthValue* _maxTV,
            uint _minEvalsOfFittestBIT,
            uint _minExhaustiveEvals)
        : target(_target), minTV(_minTV), maxTV(_maxTV),
          minEvalsOfFittestBIT(_minEvalsOfFittestBIT),
          minExhaustiveEvals(_minExhaustiveEvals)
    {}
};

float getCount(float c);

void initAxiomSet(std::string premiseFile);
void runPLNTest(Btr<PLNTest> t, bool test_bc = true);
//! @todo This next function doesn't need to have test_bc
Btr<PLNTest> setupSCMTarget(std::string conf_file, bool test_bc = true);
Btr<PLNTest> findSCMTarget(std::string test_name, bool test_bc = true);
void runSCMTargets(std::string testDir, bool test_bc = true);

}//}}
