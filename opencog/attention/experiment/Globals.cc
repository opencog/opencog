/*
 * Globals.cc
 *
 *  Created on: 2 Mar 2016
 *      Author: misgana
 */

#include <fstream>

#include "Globals.h"

using namespace opencog;
using namespace opencog::ECANExperiment;

std::vector<std::string> opencog::ECANExperiment::generated_sentences;
std::vector<HandleSeq> opencog::ECANExperiment::sent_wordnodes;
std::vector<HandleSeq> opencog::ECANExperiment::wordinstancenodes;

UnorderedHandleSet opencog::ECANExperiment::hspecial_word_nodes;

std::vector<std::string> opencog::ECANExperiment::special_words;
std::vector<std::string> opencog::ECANExperiment::nspecial_words;
int opencog::ECANExperiment::sent_size;
int opencog::ECANExperiment::special_word_occurence_period = 2;

bool opencog::ECANExperiment::are_similar(const Handle& h1, const Handle& h2, bool strict_type_match)
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

bool opencog::ECANExperiment::exists_in(const Handle& hlink, const Handle& h)
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

bool opencog::ECANExperiment::is_friendship_reln(const Handle& h)
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

bool opencog::ECANExperiment::is_smokes_reln(const Handle& h)
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

bool opencog::ECANExperiment::is_cancer_reln(const Handle& h)
{
    AtomSpace as;
    as.add_atom(h);
    Handle var_1 = as.add_node(CONCEPT_NODE, "$0343O45FFEWZ");
    Handle cancer_predicate = as.add_node(PREDICATE_NODE, "cancer");
    Handle list = as.add_link(LIST_LINK, HandleSeq { var_1 });
    Handle eval_link = as.add_link(EVALUATION_LINK, { cancer_predicate,
                                                      list });
    if (are_similar(h, eval_link, true))
        return true;
    else
        return false;
}

void opencog::ECANExperiment::save(const std::string& filename, const HandleSeq& seq, const std::string& header)
{
    std::stringstream sstream;
    sstream << header << std::endl;
    for (const Handle& h : seq)
        sstream << h->toShortString() << std::endl;

    std::ofstream outf(filename, std::ofstream::out | std::ofstream::app);
    outf << sstream.str();
    outf.flush();
    outf.close();

    return;
}




