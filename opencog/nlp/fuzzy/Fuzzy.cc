/*
 * Fuzzy.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * Author: Leung Man Hin <https://github.com/leungmanhin>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/atoms/base/Node.h>
#include <opencog/atomutils/AtomUtils.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atomutils/Neighbors.h>
#include <opencog/nlp/types/atom_types.h>

#include "Fuzzy.h"

using namespace opencog::nlp;
using namespace opencog;

/**
 * The constructor.
 *
 * @param as  The AtomSpace that we are using
 * @param tt  The type of atoms we are looking for
 * @param ll  A list of atoms that we don't want them to exist in the results
 */
Fuzzy::Fuzzy(AtomSpace* a, Type tt, const HandleSeq& ll) :
    as(a),
    rtn_type(tt),
    excl_list(ll)
{
}

Fuzzy::~Fuzzy()
{
}

static void get_all_words(const Handle& h, HandleSeq& words, HandleSeq& winsts)
{
    if (h->isNode())
    {
        for (const Handle& wi : get_target_neighbors(h, REFERENCE_LINK))
        {
            if (wi->isNode() and h->getName() == wi->getName() and
                    wi->getType() == WORD_INSTANCE_NODE)
            {
                for (const Handle& w : get_target_neighbors(wi, LEMMA_LINK))
                {
                    if (w->getType() == WORD_NODE and
                        std::find(winsts.begin(), winsts.end(), wi) == winsts.end())
                    {
                        winsts.emplace_back(wi);
                        words.emplace_back(w);
                    }
                }
            }
        }

        return;
    }

    for (const Handle& o : h->getOutgoingSet())
        get_all_words(o, words, winsts);
}

void Fuzzy::calculate_tfidf(const HandleSeq& words)
{
    double min = 0;
    double max = 0;

    int num_of_words = words.size();
    size_t num_of_sents = (size_t) as->get_num_atoms_of_type(SENTENCE_NODE);

    for (const Handle& w : words)
    {
        if (tfidf_words.count(w.value())) continue;

        int word_cnt = std::count(words.begin(), words.end(), w);

        OrderedHandleSet hs;
        for (const Handle& l : get_source_neighbors(w, LEMMA_LINK))
        {
            for (const Handle& p : get_target_neighbors(l, WORD_INSTANCE_LINK))
            {
                const HandleSeq& sent_nodes = get_target_neighbors(p, PARSE_LINK);
                hs.insert(sent_nodes.begin(), sent_nodes.end());
            }
        }

        size_t num_sents_contains_it = hs.size();

        double tf = (double) word_cnt / num_of_words;
        double idf = log2((double) num_of_sents / num_sents_contains_it);
        double tfidf = tf * idf;

        tfidf_words[w.value()] = tfidf;

        if (tfidf < min) min = tfidf;
        if (tfidf > max) max = tfidf;
    }

    // Normalize the values
    if (min != max)
        for (auto i = tfidf_words.begin(); i != tfidf_words.end(); i++)
            i->second = (i->second - min) / (max - min);
}

void Fuzzy::start_search(const Handle& trg)
{
    target = trg;
    get_all_words(target, target_words, target_winsts);
    calculate_tfidf(target_words);
}

/**
 * Determine whether or not to accept a node as a starter node for fuzzy
 * pattern matching. By default it doesn't allow variables or instances to
 * be starters, and they should also be either ConceptNodes or PredicateNodes
 * as these type of atoms are likely to be some actual words in sentences.
 *
 * @param np  A NodePtr pointing to a node in the pattern
 * @return    True if the node is accepted, false otherwise
 */
bool Fuzzy::accept_starter(const Handle& hp)
{
    if (hp->isLink()) return false;

    return (hp->getType() == CONCEPT_NODE or hp->getType() == PREDICATE_NODE)
            and (hp->getName().find("@") == std::string::npos);
}

/**
 * Determine whether or not to accept a potential solution found by the fuzzy
 * pattern matcher.
 *
 * The potential solution has to be of the same type as the rtn_type, and
 * does not contain any unwanted atoms listed in the excl_list. To calculate
 * a similarity score, a list of common nodes will be obtained. Different weights
 * will be assigned to those with certain linguistic relations in a sentence.
 * The accepted solutions will be stored in the solns vector.
 *
 * @param pat   The pattern
 * @param soln  The potential solution
 */
bool Fuzzy::try_match(const Handle& soln)
{
    if (target == soln) return false;

    // Keep exploring if this is not the type of atom that we want,
    // until it reaches its root
    if (soln->getType() != rtn_type)
        return true;

    // Check if we have seen the exact same one before
    if (std::find(solns_seen.begin(), solns_seen.end(), soln) != solns_seen.end())
        return false;

    solns_seen.insert(soln);

    // Reject it if it contains any unwanted atoms
    // TODO: any_atom_in_tree?
    for (const Handle& excl : excl_list)
        if (is_atom_in_tree(soln, excl))
            return false;

    HandleSeq soln_words;
    HandleSeq soln_winsts;
    get_all_words(soln, soln_words, soln_winsts);

    double score = 0;  // Initial value

    if (target_words.size() > soln_words.size())
        compare(soln_words, target_words, 0, score, false);
    else
        compare(target_words, soln_words, 0, score, true);

    score /= std::max(target_words.size(), soln_words.size());

    // Accept and store the solution
    if (score > 0)
        solns.push_back(std::make_pair(soln, score));

    return true;
}

void Fuzzy::compare(HandleSeq& hs1, HandleSeq& hs2,
                    double score, double& max_score, bool taf)
{
    if (hs1.size() == 0)
    {
        // Only record the highest score it found so far
        if (score > max_score)
            max_score = score;

        return;
    }

    Handle& h1 = hs1[0];

    for (Handle& h2 : hs2)
    {
        double s = get_score(h1, h2, taf);
        score += s;

        HandleSeq hs1_cp = hs1;
        HandleSeq hs2_cp = hs2;
        hs1_cp.erase(hs1_cp.begin());
        hs2_cp.erase(std::find(hs2_cp.begin(), hs2_cp.end(), h2));

        compare(hs1_cp, hs2_cp, score, max_score, taf);

        score -= s;
    }
}

double Fuzzy::get_score(const Handle& h1, const Handle& h2, bool target_at_first)
{
    std::pair<UUID, UUID> p(h1.value(), h2.value());

    if (scores.find(p) != scores.end())
        return scores.at(p);

    double score = 0;
    bool proceed = false;

    if (h1 == h2)
    {
        score += NODE_WEIGHT;
        proceed = true;
    }

    // If they are different, see if they are connected by a SimilarityLink
    else
    {
        HandleSeq iset;

        // Loop the smaller set...
        if (h1->getIncomingSetSize() > h2->getIncomingSetSize())
            h2->getIncomingSet(back_inserter(iset));
        else h1->getIncomingSet(back_inserter(iset));

        for (const Handle& h : iset)
        {
            if (h->getType() != SIMILARITY_LINK) continue;

            if (is_atom_in_tree(h, h1) and is_atom_in_tree(h, h2))
            {
                score = h->getTruthValue()->getMean() * NODE_WEIGHT;
                proceed = true;
                break;
            }
        }
    }

    if (proceed)
    {
        // See which one is from target
        // This only matters if two nodes are different
        // but they are connected by a SimilarityLink
        const Handle& tn = (target_at_first)? h1 : h2;

        score += tfidf_words[tn.value()] * RARENESS_WEIGHT;

        // TODO: May do the same checking for the soln_winst as well to see
        // if it matches with the target_winst
        size_t idx = std::find(target_words.begin(), target_words.end(), tn) - target_words.begin();
        Handle& winst = target_winsts[idx];
        HandleSeq evals = get_predicates(winst, DEFINED_LINGUISTIC_RELATIONSHIP_NODE);

        for (const Handle& el : evals)
        {
            // Extract the relation
            std::string ling_rel = (el->getOutgoingSet()[0])->getName();

            // Assign weights accordingly, subject to change
            if (ling_rel.compare("_subj") == 0 or
                ling_rel.compare("_obj") == 0 or
                ling_rel.compare("_predadj") == 0)
            {
                score += LINGUISTIC_RELATION_WEIGHT;
                break;
            }
        }
    }

    scores[p] = score;
    return score;
}

/**
 * Get method for getting the solutions sorted in descending order
 * of similarity.
 *
 * @return  A vector of solutions
 */
RankedHandleSeq Fuzzy::finished_search(void)
{
    // Sort the solutions by their similarity scores
    std::sort(solns.begin(), solns.end(),
        [] (std::pair<Handle, double> s1, std::pair<Handle, double> s2) {
            return s1.second > s2.second;
    });

    return solns;
}
