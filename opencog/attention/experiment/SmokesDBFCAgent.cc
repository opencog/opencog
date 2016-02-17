/**
 *
 *
 *
 *
 */
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

    float friends_mean()
    {
        Handle friends_predicate = _atomspace.add_node(PREDICATE_NODE,
                                                       "friends");
        Handle var_1 = _atomspace.add_node(VARIABLE_NODE, "$A");
        Handle var_2 = _atomspace.add_node(VARIABLE_NODE, "$B");
        Handle friend_list = _atomspace.add_link(LIST_LINK, { var_1, var_2 });
        Handle eval_link = _atomspace.add_link(EVALUATION_LINK, {
                friends_predicate, friend_list });
        Handle bind_link = _atomspace.add_link(BIND_LINK,
                                               { eval_link, eval_link });

        //BindLinkPtr bptr = BindLinkCast(bind_link);
        Handle friends = satisfying_set(&_atomspace, bind_link);

        strength_t tv_sum = 0.0f;
        int count = 0;
        for (const Handle& h : LinkCast(friends)->getOutgoingSet()) {
            tv_sum +=  (h->getTruthValue())->getMean();
            count++;
        }

        return (tv_sum / count);
    }

    float smokes_mean()
    {
        Handle smokes_predicate = _atomspace.add_node(PREDICATE_NODE, "smokes");
        Handle var = _atomspace.add_node(VARIABLE_NODE, "$A");
        Handle smokes_list = _atomspace.add_link(LIST_LINK, HandleSeq{ var });
        Handle eval_link = _atomspace.add_link(EVALUATION_LINK, {
                smokes_predicate, smokes_list });
        Handle bind_link = _atomspace.add_link(BIND_LINK,
                                               { eval_link, eval_link });

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

public:
    SmokesDBFCAgent(CogServer& cs) :
            Agent(cs), _atomspace(cs.getAtomSpace())
    {
        //Load core types
        config().set("SCM_PRELOAD",
                     "/usr/local/share/opencog/scm/core_types.scm, "
                     "/usr/local/share/opencog/scm/utilities.scm, "
                     "/usr/local/share/opencog/scm/av-tv.scm, "
                     "opencog/dynamics/experiment/data/smokes/smokes_db.scm, "
                     "opencog/dynamics/experiment/data/smokes/rule_base.scm");
        _eval = new SchemeEval(&_atomspace);
        load_scm_files_from_config(_atomspace);
        rule_base = _atomspace.get_node(CONCEPT_NODE, "SMOKES_RB");

    }

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

    void run()
    {
        HandleSeq af_set;
        _atomspace.get_handle_set_in_attentional_focus(
                std::back_inserter(af_set));

        Handle source;
        HandleSeq fc_result;

        if (not af_set.empty()) {
            // Select a random source from the AF to start with FC.
            source = rand_element(af_set);

            ForwardChainer fc(_atomspace, rule_base, source, af_set);
            fc.do_step();
            fc_result = fc.get_chaining_result();
        } else {
            // Select a random source from the atomspace to start with FC.
            HandleSeq hs;
            _atomspace.get_handles_by_type(hs, CONCEPT_NODE);
            source = rand_element(hs);

            ForwardChainer fc(_atomspace, rule_base, source, { });
            fc.do_step();
            fc_result = fc.get_chaining_result();
        }

        HandleSeq unique;
        std::set_difference(inference_result.begin(), inference_result.end(),
                            fc_result.begin(), fc_result.end(),
                            std::inserter(unique, unique.end()));

        for (Handle& h : unique) {
            if (is_surprising(h)) {
                //xxx not sure what amount of stimulus should be provided.
                stimulateAtom(h, 20);
            }
            inference_result.insert(h);
        }

    }

    bool is_surprising(const Handle& h)
    {
        Handle friends_predicate = _atomspace.add_node(PREDICATE_NODE,
                                                       "friends");
        Handle smokes_predicate = _atomspace.add_node(PREDICATE_NODE, "smokes");
        Handle var_1 = _atomspace.add_node(CONCEPT_NODE, "$0343O45FFEWW");
        Handle var_2 = _atomspace.add_node(CONCEPT_NODE, "$0343045FFEWY");
        Handle friend_list = _atomspace.add_link(LIST_LINK, HandleSeq{ var_1, var_2 });
        Handle smokes_list = _atomspace.add_link(LIST_LINK, HandleSeq{ var_1 });
        Handle eval_link_1 = _atomspace.add_link(EVALUATION_LINK, {
                friends_predicate, friend_list });
        Handle eval_link_2 = _atomspace.add_link(EVALUATION_LINK, {
                smokes_predicate, smokes_list });

        strength_t mean_tv;
        if (are_similar(h, eval_link_1, true)) {
            mean_tv = friends_mean();
        } else if (are_similar(h, eval_link_2, true)) {
            mean_tv = smokes_mean();
        }

        float mi; //TODO get the mi
        auto it = dist_surprisingness.begin();
        int top_k_percent = (K_PERCENTILE / 100) * dist_surprisingness.size();
        if (mi >= * std::next(it,top_k_percent)) {
            dist_surprisingness.insert(mi);
            return true;
        } else {
            dist_surprisingness.insert(mi);
            return false;
        }
    }

    bool are_similar(const Handle& h1, const Handle& h2, bool strict_type_match)
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
};

//TODO summary
// - Integrate the haskell C binding here i.e statical linking in the make file
// - Make sure rules are loaded properly
// - Add log messages in the code
// - Start running it and experimenting
