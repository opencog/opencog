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
#include <opencog/util/Logger.h>
#include <opencog/attention/experiment/tv-toolbox/TVToolBoxCInterface_stub.h>
#include <opencog/rule-engine/URECommons.h>
#include <opencog/attention/atom_types.h>
#include <algorithm>
#include <utility>

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
    HandleSeq fc_result={};
    AtomSpace& _atomspace;
    SchemeEval* _eval;
    Handle rule_base;
    string loggername = "smokeslog.log";
    Logger * smokes_logger;
    // A descending order sorted surprising result list.
    std::set<float, std::greater<int>> dist_surprisingness_friends;
    std::set<float, std::greater<int>> dist_surprisingness_smokes;

    const int K_PERCENTILE = 5;

    float friends_mean();
    float smokes_mean();
    float cancer_mean();

    Handle select_source(void)
    {
        HandleSeq out;
        _atomspace.get_handle_set_in_attentional_focus(std::back_inserter(out));
        std::map<Handle, float> atom_sti_map;
        for (Handle& h : out) {
            auto type = h->getType();
            if (type == ASYMMETRIC_HEBBIAN_LINK || type == HEBBIAN_LINK
                || type == SYMMETRIC_HEBBIAN_LINK
                || type == INVERSE_HEBBIAN_LINK
                || type == SYMMETRIC_INVERSE_HEBBIAN_LINK) {
                continue;
            }
            atom_sti_map[h] = h->getSTI();
        }

        if (atom_sti_map.size() == 0) {
            std::cout << "[SmokesDBFCAgent::select_source] Empty source candidate set exiting.\n";
            throw std::runtime_error("[SmokesDBFCAgent::select_source] Empty source candidate set.");
        }
        URECommons urec(_atomspace);

        return urec.tournament_select(atom_sti_map);
    }

    void adjust_af_boundary(int cap_size)
    {
        HandleSeq out;
        _atomspace.get_handle_set_in_attentional_focus(std::back_inserter(out));
        if(out.size() == 0)
            return;
        auto comparator =
                [](Handle& h1, Handle& h2) {return h1->getSTI() > h2->getSTI();};
        std::sort(out.begin(), out.end(), comparator);

        AttentionValue::sti_t afboundary;
        if (out.size() > (HandleSeq::size_type) cap_size) {
            afboundary = out[cap_size]->getSTI();
        } else {
            afboundary = out[out.size() - 1]->getSTI();
        }
        // Set the AF boundary
        _atomspace.set_attentional_focus_boundary(afboundary);

    }

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

    float surprisingness_value(const Handle& h);
};

#endif
//TODO summary
// - Make sure rules are loaded properly/ NOTE: max-iter loading is causing a problem. Thats the only issue. besides just tidying the mv command. then ready to run.
// - Add log messages in the code
// - Start running it and experimenting
