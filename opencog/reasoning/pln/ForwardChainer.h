#ifndef FORWARDCHAINER_H_
#define FORWARDCHAINER_H_

#include "PLN.h"
#include "rules/RuleProvider.h"

#include <RandGen.h>

namespace reasoning
{

const static int FWD_CHAIN_MAX_APPS = 100;
const static float FWD_CHAIN_MIN_CONFIDENCE = 0.4f;
const static float FWD_CHAIN_PROB_STACK = 0.7f;
const static float FWD_CHAIN_PROB_GLOBAL = 0.3f;
const static int FWD_CHAIN_MAX_FILL = 50;

//! Classes that manages forward chaining
//! @warning all Handles are PLN fake handles
class ForwardChainer {
private:

public:

    ForwardChainer();
    ~ForwardChainer();

    ForwardChainerRuleProvider rp;

    //! Rules that have yet to be attempted for forward chaining on seed handle.
    //! Consists of all rules at the beginning.
    std::deque<Handle> seedStack;

    //! minimum confidence to accept a result or for using an atom
    float minConfidence;
    float probStack; 
    float probGlobal;
    //! In future perhaps support a higher chance for handles that have context
    //! specific information?
    //float probContext

    //! Fill the seedStack with most important (high STI) atoms
    //! @param max number of handles to put in stack
    //! @param randomly add atoms instead of using importance
    //! @return actual number of handles put in stack
    int fillStack(int number = FWD_CHAIN_MAX_FILL, bool random = false);

    //! Chain from a single seed Handle
    //! @param maximum number of rule applications before ending
    //! @return return Handles that were created
    HandleSeq fwdChainSeed(Handle h, int maxRuleApps = 1);

    //! Chain to specific target
    //! @param target Handle
    //! @param maximum number of rule applications before ending
    //! @param whether target was reached
    Handle fwdChainToTarget(Handle target = 0, int maxRuleApps = FWD_CHAIN_MAX_APPS);

    //! Chain till (current) entire stack has been processed
    //! @param maximum number of rule applications before ending
    //! @return return Handles that were created
    HandleSeq fwdChainStack(int maxRuleApps = FWD_CHAIN_MAX_APPS);

    //! Get a random handle from the seed stack or global atomspace
    Handle getRandomArgument(const std::vector< Vertex > &args);
    static RandGen* rng;
    RandGen* getRNG();

};

} // namespace reasoning
#endif // FORWARDCHAINER_H_
