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
        for (Handle& h : hs) {
            //Choose associated atoms as focu_set
            for (const Handle& t : targets) {
                if (exists_in(h, t) and not opencog::contains_atomtype(
                        h, VARIABLE_NODE)
                    and not classserver().isA(h->getType(), HEBBIAN_LINK)) {
                    af_set.push_back(h);
                }
            }
        }

        if (af_set.empty()) {
            std::cerr
                    << "******COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.********\n";
            save("smokes-fc-result.data",
                 HandleSeq { },
                 "\n******COULDNT FIND A SMOKES OR FRIENDS SOURCE.RETURNING.********\n");
            return;
        }

        //Stimulate source and focus set
        int size = af_set.size();
        save("smokes-fc-result.data", HandleSeq { },
             "INTERESTING_ATOMS_SIZE=" + std::to_string(size));

        for (Handle& h : af_set) {
            auto scaled_stim = 4 * pow(10, surprisingness_value(h)) * surprisingness_value(h);

            save("smokes-fc-result.data",
                 HandleSeq { },
                 "stimulating atom " + h->toShortString() + " with amount "
                 + std::to_string(scaled_stim));
            stimulateAtom(h, scaled_stim);
        }

        source = select_source(); // tournament selection by STI

        first_run = false;

        return; // We have to let the stimulus get converted to STI by the next running ECAN agent.
    }

    auto prev_afb = _atomspace.get_attentional_focus_boundary();
    HandleSeq temphseq;
     _atomspace.get_handle_set_in_attentional_focus(
            std::back_inserter(temphseq));
     auto prev_afsize = temphseq.size();

    // Dynamically set the AF boundary with lbound of the top K STI values
    // K=50
    adjust_af_boundary(30);

     af_set.clear();
    _atomspace.get_handle_set_in_attentional_focus(std::back_inserter(af_set));


    std::cerr << "--ADJUSTED AF CONTENT AT CYCLE: "
    << cogserver().getCycleCount() << "--AF_SIZE: " << af_set.size()
    << "--PRE_AF_SIZE: " << prev_afsize <<
    "--INITAIL_AF_BOUNDARY: " << prev_afb << "--ADJUSTED_AF_BOUNDARY: "
    << _atomspace.get_attentional_focus_boundary() << "--\n";

    for (const Handle& h : af_set)
        std::cerr << h->toString() << "\n";


    if(af_set.size() == 0 ){
        std::cout << "ATTENTIONAL FOCUS IS OUT OF ATOMS. TERMINATING EXPERIMENT\n";
        throw std::runtime_error( "ATTENTIONAL FOCUS IS OUT OF ATOMS. TERMINATING EXPERIMENT");
    }

    source = select_source(); // tournament selection by STI

    // Do one step forward chaining.
    std::cerr << "------------FCing-------------" << std::endl;
    std::cerr << "RULE_BASE: " << rule_base->toShortString() << "\n";
    std::cerr << "SOURCE: " << source->toShortString() << "\n";

    ForwardChainer fc(_atomspace, rule_base, source, HandleSeq { af_set },
                      opencog::source_selection_mode::STI);

    // Just to compensate for randomness in FC.
    for(int i=0; i < 5; i++)
       fc.do_step();

    std::cerr << "FORWARD CHAINER STEPPED\n\t";

    UnorderedHandleSet temp = fc.get_chaining_result();
    HandleSeq cur_fcresult;
    std::copy(temp.begin(), temp.end(), std::back_inserter(cur_fcresult));

    // Stimulate surprising unique results.
    HandleSeq unique_set;
    std::cerr << "Found " << cur_fcresult.size() << " results.\n";
    for (const Handle& h : cur_fcresult) {
        auto hinserted = _atomspace.add_atom(h);
        std::cerr << "\t" << hinserted->toShortString() << "\n";
        bool unique = true;
        for (const Handle& hi : fc_result) {
            if (hinserted == hi) {
                unique = false;
                std::cerr << "Not a new inference! \n";
                break;
            }
        }
        if (unique) {
            unique_set.push_back(hinserted);
            fc_result.push_back(hinserted);
        }
    }

    std::cerr << "\tFound " << unique_set.size() << " unique inferences.\n\t";
    save("smokes-fc-result.data",
         HandleSeq { },
         "\n***FORWARD CHAINING RESULT***\ncycle=" + std::to_string(
                 cogserver().getCycleCount())
         + " smokers mean_tv=" + std::to_string(smokes_mean())
         + " friendship mean_tv=" + std::to_string(friends_mean()) + "\nFound "
         + std::to_string(unique_set.size()) + "unique inferences.\n");

    // Stimulate surprising unique results.
    for (Handle& h : unique_set) {
        auto scaled_stim = 8 * pow(10, surprisingness_value(h)) * surprisingness_value(h);
        stimulateAtom(h, scaled_stim);

        std::cerr << "provided stimulus \n to " << h->getUUID() << " of amount "
        << std::to_string(scaled_stim) + "\n";
        save("smokes-fc-result.data", HandleSeq { h },
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
    save("smokes-fc-result.data", HandleSeq { }, "\n");
    if (is_friendship_reln(h)) {
        mean_tv = friends_mean();
        mi = sqrtJsdC_hs(10, mean_tv, 100, 10, (h->getTruthValue())->getMean(),
                         100, 100);
        save("smokes-fc-result.data",
             HandleSeq { },
             "JSD_VAL(10, " + std::to_string(mean_tv) + "100, 10, "
             + std::to_string((h->getTruthValue())->getMean())
             + ", 100, 100) = " + std::to_string(mi));

    } else if (is_smokes_reln(h)) {
        mean_tv = smokes_mean();
        mi = sqrtJsdC_hs(10, mean_tv, 100, 10, (h->getTruthValue())->getMean(),
                         100, 100);
        save("smokes-fc-result.data",
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
        save("smokes-fc-result.data",
             HandleSeq { },
             "JSD_VAL(10," + std::to_string(mean_tv) + "100,10,"
             + std::to_string((h->getTruthValue())->getMean()) + ",100,100) = "
             + std::to_string(mi));

    }

    return mi;
}
