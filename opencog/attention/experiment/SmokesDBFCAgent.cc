/*
 * SmokesDBFCAgnet.cc
 *
 *  Created on: 18 Feb 2016
 *      Author: misgana
 */

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atomspaceutils/AtomSpaceUtils.h>
#include <opencog/attention/atom_types.h>
#include <opencog/guile/load-file.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Logger.h>
#include <opencog/rule-engine/forwardchainer/ForwardChainer.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/util/random.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/attention/experiment/tv-toolbox/TVToolBoxCInterface_stub.h>

#include <algorithm>
#include <fstream>
#include <cmath>

#include "Globals.h"
#include "SmokesDBFCAgent.h"

using namespace opencog;
using namespace opencog::ECANExperiment;

float SmokesDBFCAgent::friends_mean()
{
    Handle friends_predicate = _atomspace.add_node(PREDICATE_NODE, "friends");
    Handle var_1 = _atomspace.add_node(VARIABLE_NODE, "$A");
    Handle var_2 = _atomspace.add_node(VARIABLE_NODE, "$B");

    Handle friend_list = _atomspace.add_link(LIST_LINK, { var_1, var_2 });
    Handle eval_link = _atomspace.add_link(EVALUATION_LINK, { friends_predicate,
                                                              friend_list });
    Handle bind_link = _atomspace.add_link(BIND_LINK, { eval_link, eval_link });

    //Handle friends = satisfying_set(&_atomspace, bind_link);
    Handle friends = bindlink(&_atomspace, bind_link);
    strength_t tv_sum = 0.0f;
    float count = 0.0f;
    for (const Handle& h : LinkCast(friends)->getOutgoingSet()) {
        if (not opencog::contains_atomtype(h, VARIABLE_NODE)) {
            tv_sum += (h->getTruthValue())->getMean();
            count++;
        }
    }

    return (tv_sum / count);
}

float SmokesDBFCAgent::smokes_mean()
{
    Handle smokes_predicate = _atomspace.add_node(PREDICATE_NODE, "smokes");
    Handle var = _atomspace.add_node(VARIABLE_NODE, "$A");
    /*Handle varlist =
     _eval->eval_h(
     R"(VariableList
     (TypedVariableLink
     (VariableNode "$A")
     (TypeNode "ConceptNode")) )");*/

    Handle smokes_list = _atomspace.add_link(LIST_LINK, HandleSeq { var });
    Handle eval_link = _atomspace.add_link(EVALUATION_LINK, { smokes_predicate,
                                                              smokes_list });
    Handle bind_link = _atomspace.add_link(BIND_LINK, { eval_link, eval_link });

    //Handle smokers = satisfying_set(&_atomspace, bind_link);
    Handle smokers = bindlink(&_atomspace, bind_link);
    remove_hypergraph(_atomspace, bind_link);

    strength_t tv_sum = 0.0f;
    int count = 0.0f;

    for (const Handle& h : LinkCast(smokers)->getOutgoingSet()) {
        if (not opencog::contains_atomtype(h, VARIABLE_NODE)) {
            tv_sum += (h->getTruthValue())->getMean();
            count++;
        }
    }

    return (tv_sum / count);
}

float SmokesDBFCAgent::cancer_mean()
{
    Handle cancer_predicate = _atomspace.add_node(PREDICATE_NODE, "cancer");
    Handle var = _atomspace.add_node(VARIABLE_NODE, "$A");

    Handle list = _atomspace.add_link(LIST_LINK, HandleSeq { var });
    Handle eval_link = _atomspace.add_link(EVALUATION_LINK, { cancer_predicate,
                                                              list });
    Handle bind_link = _atomspace.add_link(BIND_LINK, { eval_link, eval_link });

    //Handle smokers = satisfying_set(&_atomspace, bind_link);
    Handle cancerous = bindlink(&_atomspace, bind_link);
    remove_hypergraph(_atomspace, bind_link);

    strength_t tv_sum = 0.0f;
    int count = 0.0f;

    for (const Handle& h : LinkCast(cancerous)->getOutgoingSet()) {
        if (not opencog::contains_atomtype(h, VARIABLE_NODE)) {
            tv_sum += (h->getTruthValue())->getMean();
            count++;
        }
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
                 "opencog/attention/experiment/data/noise.scm, "
                 "opencog/attention/experiment/data/smokes/rule_base.scm");

    _eval = new SchemeEval(&_atomspace);
    load_scm_files_from_config(_atomspace);
    smokes_logger = new Logger(loggername);
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

    std::cerr << "CYCLE:" << cogserver().getCycleCount() << std::endl;

    if (first_run) {
        // Pull some atoms to the AF set
        // and select a random source from the pulled set for starting FC with.
        HandleSeq hs;
        _atomspace.get_handles_by_type(std::back_inserter(hs), ATOM, true);

        if (hs.empty()) {
            std::cout << "EMPTY ATOMSPACE\n";
            return;
        }
        // Choose a random source  and focus set from the AS with the ff type constraints.
        // We are looking for atoms containing smokes,friends,cancer atoms so filter em out.
        // std::cerr << "---------Pushed the following atoms to the AF set-----------\n";
        for (Handle& h : hs) {
            //Choose associated atoms as focu_set
            for (const Handle& t : targets) {
                if (exists_in(h, t) and not opencog::contains_atomtype(
                        h, VARIABLE_NODE)
                    and not classserver().isA(h->getType(), HEBBIAN_LINK)) {
                    // std::cerr << h->toShortString() << "\n";
                    af_set.push_back(h);
                }
            }
        }

        if (af_set.empty()) {
            std::cerr
                    << "******COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.********\n";
            save("smokes-fc-resulut.data",
                 HandleSeq { },
                 "\n******COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.********\n");
            return;
        }

        //Stimulate source and focus set
        int size = af_set.size();
        //std::cerr << "STIMULATING SOURCE FOR PULLING IT IN TO AF with amount " << 10/log10(size) << "\n";
        save("smokes-fc-resulut.data", HandleSeq { },
             "INTERESTING_ATOMS_SIZE=" + std::to_string(size));

        for (Handle& h : af_set) {
            auto scaled_stim = 0.8*pow(10, surprisingness_value(h));

            save("smokes-fc-resulut.data",
                 HandleSeq { },
                 "stimulating atom " + h->toShortString() + " with amount "
                 + std::to_string(scaled_stim));
            stimulateAtom(h,scaled_stim);
        }

        source = rand_element(af_set);

        first_run = false;

        return; // We have to let the stimulus get converted to STI by the next running ECAN agent.
    }

    else {
        af_set.clear();
        _atomspace.get_handle_set_in_attentional_focus(
                std::back_inserter(af_set));
        // Remove Hebbian links from focus set.
        af_set.erase(
                std::remove_if(af_set.begin(), af_set.end(), [](Handle& h) {
                    return classserver().isA(h->getType(),HEBBIAN_LINK);
                }),
                af_set.end());
        if (af_set.empty()) {
            std::cerr
                    << "***********COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.**************\n";
            save("smokes-fc-resulut.data",
                 HandleSeq { },
                 "\n******COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.********\n");
            return;
        }
        //Cap the attentional focus set
        af_set = capped_af_set(af_set, af_set.size() < 50 ? af_set.size() : 50 ); //ceil(0.3*af_set.size()));

        source = rand_element(af_set);
    }

    std::cerr << "--------------MODIFIED AF CONTENT AT CYCLE: "
              << cogserver().getCycleCount() << "---------AF_SIZE: "
              << af_set.size() << "-------\n";

    for (const Handle& h : af_set)
        std::cerr << h->toString() << "\n";

    // Do one step forward chaining.
    std::cerr << "------------FCing-------------" << std::endl;
    std::cerr << "RULE_BASE: " << rule_base->toShortString() << "\n";
    std::cerr << "SOURCE: " << source->toShortString() << "\n";

    ForwardChainer fc(_atomspace, rule_base, source, HandleSeq { af_set },
                      opencog::source_selection_mode::STI);
    fc.do_step();

    std::cerr << "FORWARD CHAINER STEPPED\n\t";

    UnorderedHandleSet fc_result = fc.get_chaining_result();

    // Stimulate surprising unique results.
    HandleSeq unique;
    std::cerr << "Found " << fc_result.size() << " results.\n";
    for (const Handle& h : fc_result) {
        std::cerr << "\t" << h->toShortString() << "\n";
        if (std::find(inference_result.begin(), inference_result.end(), h) == inference_result.end())
        {
            unique.push_back(h);
            inference_result.push_back(h);
        }
    }

    std::cerr << "\tFound " << unique.size() << " unique inferences.\n\t";
    save("smokes-fc-resulut.data",
         HandleSeq { },
         "\n***FORWARD CHAINING RESULT***\ncycle=" + std::to_string(
                 cogserver().getCycleCount())
         + " smokers mean_tv=" + std::to_string(smokes_mean())
         + " friendship mean_tv=" + std::to_string(friends_mean()) + "\nFound "
         + std::to_string(unique.size()) + "unique inferences.\n");

    for (Handle& h : unique) {
        auto scaled_stim = 4 * pow(10, surprisingness_value(h));
        stimulateAtom(h, scaled_stim);

        std::cerr << "provided stimulus \n to " << h->getUUID()
        << " of amount " << std::to_string(scaled_stim) + "\n";
        save("smokes-fc-resulut.data", HandleSeq { h },
             "provided stimulus amount " + std::to_string(scaled_stim) + "\n");
    }

}

float SmokesDBFCAgent::surprisingness_value(const Handle& hx)
{
    Handle h;
    if (hx->getType() == IMPLICATION_LINK) {
        h = LinkCast(hx)->getOutgoingSet()[1];
    } else
        h = hx;

    strength_t mean_tv = 0.0f;
    bool val = false;
    float mi = 0.0f;
    save("smokes-fc-resulut.data", HandleSeq { }, "\n");
    if (is_friendship_reln(h)) {
        mean_tv = friends_mean();
        mi = sqrtJsdC_hs(10, mean_tv, 100, 10, (h->getTruthValue())->getMean(),
                         100, 100);
        save("smokes-fc-resulut.data",
             HandleSeq { },
             "JSD_VAL(10, " + std::to_string(mean_tv) + "100, 10, "
             + std::to_string((h->getTruthValue())->getMean())
             + ", 100, 100) = " + std::to_string(mi));

    } else if (is_smokes_reln(h)) {
        mean_tv = smokes_mean();
        mi = sqrtJsdC_hs(10, mean_tv, 100, 10, (h->getTruthValue())->getMean(),
                         100, 100);
        save("smokes-fc-resulut.data",
             HandleSeq { },
             "JSD_VAL(10," + std::to_string(mean_tv) + "100,10,"
             + std::to_string((h->getTruthValue())->getMean()) + ",100,100) = "
             + std::to_string(mi));

    }

    // If it contains has_cancer predicate, let it be surprising.
    else if (is_cancer_reln(h)) {
        mean_tv = cancer_mean();
        mi = sqrtJsdC_hs(10, mean_tv, 100, 10, (h->getTruthValue())->getMean(),
                         100, 100);
        save("smokes-fc-resulut.data",
             HandleSeq { },
             "JSD_VAL(10," + std::to_string(mean_tv) + "100,10,"
             + std::to_string((h->getTruthValue())->getMean()) + ",100,100) = "
             + std::to_string(mi));

    }

    return mi;
}
