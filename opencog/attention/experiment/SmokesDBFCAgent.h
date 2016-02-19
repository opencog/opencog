/**
 *
 *
 *
 *
 */

#ifndef _SMOKESDBFCAGENT_
#define _SMOKESDBFCAGENT_

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspaceutils/AtomSpaceUtils.h>
#include <opencog/guile/load-file.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/rule-engine/forwardchainer/ForwardChainer.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/util/random.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/attention/experiment/tv-toolbox/TVToolBoxCInterface_stub.h>

#include <algorithm>

using namespace opencog;

//class FCAgent: 
//  result_set
//  person_smokes_mean
//  friends_mean
//  run:
//    If {AF} != EMPTY:    
//      fc_step_once(source={RANDOM_FROM_AS}, focus_set={EMPTY})
//    else:
//      fc_step_once(source={AF}, focus_set={AF})
//    
//    for_each result in fc_result and result not in result_set:
//       evaluate_surprisingess(mean_smoke|mean_friendship inheritance,fc_result)
//       if result is surprising_enough:
//          sttimulate(result)

//How much of a stimulus to provide?
//How often to stimulate?
//How to calculate surprisingness?
//	Calculcate the population mean for smokes predicate and Friends predicate
//	Calculate MIs with each Friends and Smokes relation with their mean
//      Boost STI for those with higher MI
// How much of surprisingness value would be enought to stimulate an atom?
// Store surprisingness distribution and check if current values fall in top 5% of the distribution as a decission boundary to stimulate it.

class SmokesDBFCAgent: public Agent {
private:
    UnorderedHandleSet inference_result;
    AtomSpace& _atomspace;
    SchemeEval* _eval;
    Handle rule_base;

    std::set<float> dist_surprisingness;
    const int K_PERCENTILE = 5;

    float friends_mean();

    float smokes_mean();


public:
    SmokesDBFCAgent(CogServer& cs);
    virtual ~SmokesDBFCAgent();

    virtual const ClassInfo& classinfo() const
    {
        return info();
    }

    static const ClassInfo& info()
    {
        static const ClassInfo _ci("opencog::SmokesDBFCAgent");
        return _ci;
    }

    void run();

    bool is_surprising(const Handle& h);

    bool are_similar(const Handle& h1, const Handle& h2, bool strict_type_match);

};

#endif
//TODO summary
// - Make sure rules are loaded properly/ NOTE: max-iter loading is causing a problem. Thats the only issue. besides just tidying the mv command. then ready to run.
// - Add log messages in the code
// - Start running it and experimenting
