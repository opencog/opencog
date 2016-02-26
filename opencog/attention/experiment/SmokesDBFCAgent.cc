/*
 * SmokesDBFCAgnet.cc
 *
 *  Created on: 18 Feb 2016
 *      Author: misgana
 */

/**
 *
 *
 *
 *
 */

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atomspaceutils/AtomSpaceUtils.h>
#include <opencog/attention/atom_types.h>
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

#include "SmokesDBFCAgent.h"

using namespace opencog;

float SmokesDBFCAgent::friends_mean()
{
    Handle friends_predicate = _atomspace.add_node(PREDICATE_NODE, "friends");
    Handle var_1 = _atomspace.add_node(VARIABLE_NODE, "$A");
    Handle var_2 = _atomspace.add_node(VARIABLE_NODE, "$B");
    Handle friend_list = _atomspace.add_link(LIST_LINK, { var_1, var_2 });
    Handle eval_link = _atomspace.add_link(EVALUATION_LINK, { friends_predicate,
                                                              friend_list });
    Handle bind_link = _atomspace.add_link(BIND_LINK, { eval_link, eval_link });

    //BindLinkPtr bptr = BindLinkCast(bind_link);
    Handle friends = satisfying_set(&_atomspace, bind_link);

    strength_t tv_sum = 0.0f;
    int count = 0;
    for (const Handle& h : LinkCast(friends)->getOutgoingSet()) {
        tv_sum += (h->getTruthValue())->getMean();
        count++;
    }

    return (tv_sum / count);
}

float SmokesDBFCAgent::smokes_mean()
{
    Handle smokes_predicate = _atomspace.add_node(PREDICATE_NODE, "smokes");
    Handle var = _atomspace.add_node(VARIABLE_NODE, "$A");
    Handle smokes_list = _atomspace.add_link(LIST_LINK, HandleSeq { var });
    Handle eval_link = _atomspace.add_link(EVALUATION_LINK, { smokes_predicate,
                                                              smokes_list });
    Handle bind_link = _atomspace.add_link(BIND_LINK, { eval_link, eval_link });

    //BindLinkPtr bptr = BindLinkCast(bind_link);
    Handle friends = satisfying_set(&_atomspace, bind_link);

    remove_hypergraph(_atomspace, bind_link);

    strength_t tv_sum = 0.0f;
    int count = 0;
    for (const Handle& h : LinkCast(friends)->getOutgoingSet()) {
        tv_sum += (h->getTruthValue())->getMean();
        count++;
    }

    return (tv_sum / count);
}

SmokesDBFCAgent::SmokesDBFCAgent(CogServer& cs) :
        Agent(cs), _atomspace(cs.getAtomSpace())
{
    //Load core types
    config().set("SCM_PRELOAD", "/usr/local/share/opencog/scm/core_types.scm, "
                 "/usr/local/share/opencog/scm/utilities.scm, "
                 "/usr/local/share/opencog/scm/av-tv.scm, "
                 "opencog/attention/experiment/data/smokes/smokes_db.scm, "
                 "opencog/attention/experiment/data/smokes/rule_base.scm");

    _eval = new SchemeEval(&_atomspace);
    load_scm_files_from_config(_atomspace);
    rule_base = _atomspace.get_node(CONCEPT_NODE, "SMOKES_RB");
    std::cout << "RULE_bASE:\n";
    std::cout << rule_base->toShortString() << "\n";
}

SmokesDBFCAgent::~SmokesDBFCAgent()
{

}

void SmokesDBFCAgent::run()
{
    static bool first_run = true;
    HandleSeq af_set;
    Handle source = Handle::UNDEFINED;
    HandleSeq targets = { _atomspace.add_node(PREDICATE_NODE, "friends"),
                          _atomspace.add_node(PREDICATE_NODE, "smokes"),
                          _atomspace.add_node(PREDICATE_NODE, "cancer") };

    std::cout << "CYCLE:" << cogserver().getCycleCount() << std::endl;

    if (first_run) {
        //Pull some atoms to the AF set
        // Select a random source from the atomspace to start with FC.
        HandleSeq hs;
        _atomspace.get_handles_by_type(hs, ATOM);

        if (hs.empty()) {
            std::cout << "EMPTY ATOMSPACE\n";
            return;
        }
        // Choose a random source  and focus set from the AS with the ff type constraints.
        // We are looking for atoms containing smokes,friends,cancer atoms so filter em out.
        for (Handle& h : hs) {
            //Choose associated atoms as focu_set
            for (const Handle& t : targets) {
                if (exists_in(h, t)
                    and not opencog::contains_atomtype( h, VARIABLE_NODE)
                    and not classserver().isA(h->getType(),HEBBIAN_LINK)){
                          af_set.push_back(h);
                }
            }
        }

        if (af_set.empty()) {
            std::cout << "COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.\n";
            return;
        }

        //Stimulate source and focus set
        std::cout << "STIMULATING SOURCE FOR PULLING IT IN TO AF\n";
        for (Handle& h : af_set) {
            stimulateAtom(h,
                          _atomspace.get_attentional_focus_boundary() + 10);
        }

        source = rand_element(af_set);

        first_run = false;
    }

    else {
        af_set.clear();
        _atomspace.get_handle_set_in_attentional_focus(std::back_inserter(af_set));
        // Remove Hebbian links from focus set.
        af_set.erase(std::remove_if(af_set.begin(), af_set.end(), [](Handle& h) {
            return classserver().isA(h->getType(),HEBBIAN_LINK);
        }),
                   af_set.begin());
        if (af_set.empty()) {
            std::cout << "COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.\n";
            return;
        }
        source = rand_element(af_set);
    }

    // Do one step forward chaining.
    ForwardChainer fc(_atomspace, rule_base, source, { af_set });
    fc.do_step();
    std::cout << "FORWARD CHAINER STEPPED\n\t";


    // Stimulate surprising unique results.
    HandleSeq unique;
    HandleSeq fc_result = fc.get_chaining_result();
    std::cout << "Found " << fc_result.size() << " results.\n";
    for (const Handle& h : fc_result)
        std::cout << "\t" << h->toShortString() << "\n";
    std::sort(fc_result.begin(), fc_result.end());
    //Inference_result doesn't need to be sorted since set is ordered.
    std::set_difference(inference_result.begin(), inference_result.end(),
                        fc_result.begin(), fc_result.end(),
                        std::back_inserter(unique));
    std::cout << "\tFound " << unique.size() << " unique inferences.\n\t";

    for (Handle& h : unique) {
        if (is_surprising(h)) {
            //xxx not sure what amount of stimulus should be provided.
            std::cout << h->toShortString() << "\nWAS A SURPRISING RESULT\n";
            stimulateAtom(h, _atomspace.get_attentional_focus_boundary() + 10);
        }
        inference_result.insert(h);
    }

}

bool SmokesDBFCAgent::is_surprising(const Handle& h)
{

    strength_t mean_tv = 0.0f;
    if (is_friendship_reln(h)) {
        mean_tv = friends_mean();
    } else if (is_smokes_reln(h)) {
        mean_tv = smokes_mean();
    }
    // Calculate the Jensen Shanon distance bn mean_tv and h's tv
    float mi = sqrtJsdC_hs(10, mean_tv, 100, 10,
                           (h->getTruthValue())->getMean(), 100, 100);
    auto it = dist_surprisingness.begin();
    int top_k = (K_PERCENTILE / 100) * dist_surprisingness.size();

    // Consider the first top_k values as surprising. After we have enough
    // data only consider those who have higher value of the lbound of the
    // top_k as surprising.
    std::cout << "Surprising result list:\n";
    for (const auto& i : dist_surprisingness)
        std::cout << i << ", ";
    std::cout << "\n" << h->toShortString() << "\n\t JSD_VAL=" << mi << "\n";

    bool val;
    if (top_k > dist_surprisingness.size()) {
        dist_surprisingness.insert(mi);
        val = true;
    } else if (mi >= *std::next(it, top_k)) {
        dist_surprisingness.insert(mi);
        val = true;
    } else {
        dist_surprisingness.insert(mi);
        val = false;
    }

    std::cout << "Found to be " << (val ? "surprising" : "not surprising")
              << "\n";
    return val;
}

bool SmokesDBFCAgent::are_similar(const Handle& h1, const Handle& h2,
bool strict_type_match)
{
    if (h1 == h2)
        return true;

    if (NodeCast(h1) and NodeCast(h2))
        return !strict_type_match or h1->getType() == h2->getType();

    LinkPtr lh1(LinkCast(h1));
    LinkPtr lh2(LinkCast(h2));

    if (lh1 and lh2) {
        if (strict_type_match and (lh1->getType() != lh2->getType()))
            return false;

        HandleSeq hseqh1 = lh1->getOutgoingSet();
        HandleSeq hseqh2 = lh2->getOutgoingSet();

        if (hseqh1.size() != hseqh2.size())
            return false;

        // Unordered links should be treated in a special way
        if (classserver().isA(lh1->getType(), UNORDERED_LINK) or classserver().isA(
                lh2->getType(), UNORDERED_LINK)) {

            for (const auto& h1 : hseqh1) {
                for (auto it = hseqh2.begin(); it != hseqh2.end(); ++it) {
                    if (are_similar(h1, h2, strict_type_match)) {
                        hseqh2.erase(it);
                        break;
                    }
                }
            }

            // Empty means all has been mapped. Success.
            return hseqh2.empty() or false;
        }

        for (HandleSeq::size_type i = 0; i < hseqh1.size(); i++) {
            if (not are_similar(hseqh1[i], hseqh2[i], strict_type_match))
                return false;
        }

        return true;
    }

    return false;
}
bool SmokesDBFCAgent::exists_in(const Handle& hlink, const Handle& h) const
{
    if (hlink == h) {
        return true;
    } else {
        LinkPtr lp(LinkCast(hlink));
        if (nullptr == lp)
            return false;

        auto outg = lp->getOutgoingSet();
        if (std::find(outg.begin(), outg.end(), h) != outg.end())
            return true;
        else {
            for (const Handle& hi : outg) {
                if (LinkCast(hi) and exists_in(hi, h))
                    return true;
            }
        }
        return false;
    }
}

bool SmokesDBFCAgent::is_friendship_reln(const Handle& h)
{
    AtomSpace as;
    as.add_atom(h);
    Handle friends_predicate = as.add_node(PREDICATE_NODE, "friends");
    Handle var_1 = as.add_node(CONCEPT_NODE, "$0343O45FFEWW");
    Handle var_2 = as.add_node(CONCEPT_NODE, "$0343045FFEWY");
    Handle friend_list = as.add_link(LIST_LINK, HandleSeq { var_1, var_2 });
    Handle eval_link = as.add_link(EVALUATION_LINK, { friends_predicate,
                                                      friend_list });
    if (are_similar(h, eval_link, true))
        return true;
    else
        return false;

}

bool SmokesDBFCAgent::is_smokes_reln(const Handle& h)
{
    AtomSpace as;
    as.add_atom(h);
    Handle var_1 = as.add_node(CONCEPT_NODE, "$0343O45FFEWW");
    Handle smokes_predicate = as.add_node(PREDICATE_NODE, "smokes");
    Handle smokes_list = as.add_link(LIST_LINK, HandleSeq { var_1 });
    Handle eval_link = as.add_link(EVALUATION_LINK, { smokes_predicate,
                                                      smokes_list });
    if (are_similar(h, eval_link, true))
        return true;
    else
        return false;
}
