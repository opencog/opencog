/*
 * Fuzzy.cc
 *
 * Copyright (C) 2015, 2016 OpenCog Foundation
 * All Rights Reserved
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
 * @param a   The AtomSpace that we are using
 * @param tt  The type of atoms we are looking for
 * @param ll  A list of atoms that we don't want them to exist in the results
 */
Fuzzy::Fuzzy(AtomSpace* a, Type tt, const HandleSeq& ll, bool af_only) :
    as(a),
    rtn_type(tt),
    _af_only(af_only),
    excl_list(ll)
{
}

Fuzzy::Fuzzy(AtomSpace* a) : as(a)
{
}

Fuzzy::~Fuzzy()
{
}

/**
 * Examine a tree and get all the words from it.
 *
 * Words are the ones that are connected to their WordInstanceNodes by
 * ReferenceLinks. Those WordInstanceNodes should also connect to
 * their lemmas by LemmaLinks.
 *
 * @param h       The tree being examined
 * @param words   A list of WordNodes
 * @param winsts  A list of WordInstanceNodes
 */
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

/**
 * Compare two trees and return a similarity score.
 *
 * @param h1, h2  The two trees that will be compared
 * @return        A similarity score between the two trees
 */
double Fuzzy::fuzzy_compare(const Handle& h1, const Handle& h2)
{
    start_search(h1);

    HandleSeq h2_words;
    HandleSeq h2_winsts;
    get_all_words(h2, h2_words, h2_winsts);

    double score = 0;

    if (target_words.size() > h2_words.size())
        compare(h2_words, target_words, 0, score, false);
    else
        compare(target_words, h2_words, 0, score, true);

    score /= std::max(target_words.size(), h2_words.size());

    return score;
}

/**
 * A function intends to reflect how important a word is to a
 * document in a collection or corpus, which will be used in
 * the similarity estimation.
 *
 * @param words  A list of words (WordNode) of a sentence
 */
void Fuzzy::calculate_tfidf(const HandleSeq& words)
{
    double min = 0;
    double max = 0;

    // No. of words in the sentence
    int num_of_words = words.size();

    // Total no. of sentences in the AtomSpace
    size_t num_of_sents = (size_t) as->get_num_atoms_of_type(SENTENCE_NODE);

    for (const Handle& w : words)
    {
        if (tfidf_words.count(w.value())) continue;

        // No. of times this word exists in the sentence
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

        // No. of sentences that contain this word
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

/**
 * Override the start_search() method, to get words instead of nodes.
 *
 * @param np  A NodePtr pointing to a node in the pattern
 */
void Fuzzy::start_search(const Handle& trg)
{
    target = trg;
    get_all_words(target, target_words, target_winsts);
    std::sort(target_words.begin(), target_words.end());
    calculate_tfidf(target_words);
}

/**
 * Determine whether or not to accept a node as a starter node for fuzzy
 * matching. It accepts nodes that are not instances, as they are unlikely
 * to lead us to solutions that we want. They should also be either
 * ConceptNodes or PredicateNodes, as these type of atoms are likely to be
 * some actual words in sentences.
 *
 * @param hp  The target (input)
 * @return    True if an atom is accepted, false otherwise
 */
bool Fuzzy::accept_starter(const Handle& hp)
{
    if (hp->isLink()) return false;

    return (hp->getType() == CONCEPT_NODE or hp->getType() == PREDICATE_NODE)
            and (hp->getName().find("@") == std::string::npos);
}

/**
 * Determine whether or not to accept a potential solution found by the fuzzy
 * matcher. The potential solution has to be of the same type as the rtn_type,
 * and does not contain any unwanted atoms listed in the excl_list. The score
 * currently depends on the number of common words they both share (words
 * connects together by a SimilarityLink will also be considered as "common"
 * to a certain extent), the "rareness" of the words to all others existing
 * in the AtomSpace, and the linguistic relations of the words. The accepted
 * solutions will be stored in the solns vector.
 *
 * @param soln  The potential solution found
 * @return      True if the potential solution is accepted, false otherwise
 */
bool Fuzzy::try_match(const Handle& soln)
{
    AttentionValue::sti_t afboundary = as->get_attentional_focus_boundary();
    
    if(_af_only and  (soln->getSTI() < afboundary))  return false;
    
    if (target == soln) return false;

    // Keep exploring if this is not the type of atom that we want,
    // until it reaches its root
    if (soln->getType() != rtn_type)
        return true;

    // Reject if we have seen the exact same one before
    if (std::find(solns_seen.begin(), solns_seen.end(), soln) != solns_seen.end())
        return false;

    solns_seen.insert(soln);

    // Reject if it contains any unwanted atoms
    for (const Handle& excl : excl_list)
        if (is_atom_in_tree(soln, excl))
            return false;

    HandleSeq soln_words;
    HandleSeq soln_winsts;
    get_all_words(soln, soln_words, soln_winsts);
    std::sort(soln_words.begin(), soln_words.end());

    // Reject if it's identical to the input
    if (soln_words == target_words)
        return false;

    HandleSeq target_diff;
    HandleSeq soln_diff;
    HandleSeq common_nodes;
    auto target_iterator = target_words.begin();
    auto soln_iterator = soln_words.begin();

    // See which nodes are common/uncommon
    while (target_iterator != target_words.end() or
           soln_iterator != soln_words.end())
    {
        if (target_iterator == target_words.end())
        {
            soln_diff.insert(soln_diff.end(), soln_iterator, soln_words.end());
            break;
        }

        if (soln_iterator == soln_words.end())
        {
            target_diff.insert(target_diff.end(), target_iterator, target_words.end());
            break;
        }

        if (*target_iterator < *soln_iterator)
        {
            target_diff.push_back(*target_iterator);
            target_iterator++;
        }

        else if (*soln_iterator < *target_iterator)
        {
            soln_diff.push_back(*soln_iterator);
            soln_iterator++;
        }

        else
        {
            common_nodes.push_back(*target_iterator);
            target_iterator++;
            soln_iterator++;
        }
    }

    // Initial value
    double score = 0;

    for (const Handle& c : common_nodes)
        score += get_score(c, c, true);

    if (target_diff.size() > soln_diff.size())
        compare(soln_diff, target_diff, score, score, false);
    else
        compare(target_diff, soln_diff, score, score, true);

    score /= std::max(target_words.size(), soln_words.size());

    // Accept and store the solution
    if (score > 0)
        solns.push_back(std::make_pair(soln, score));

    return true;
}

/**
 * A recursive method for finding the highest similarity score between
 * two trees.
 *
 * @param hs1, hs2   The two trees being compared
 * @param score      The score of the current combination
 * @param h_score  The highest score among the explored combinations
 * @param taf        A flag indicating which tree is from the target
 */
void Fuzzy::compare(HandleSeq& hs1, HandleSeq& hs2,
                    double score, double& h_score, bool taf)
{
    if (hs1.size() == 0)
    {
        // Only record the highest score it found so far
        if (score > h_score)
            h_score = score;

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

        compare(hs1_cp, hs2_cp, score, h_score, taf);

        score -= s;
    }
}

/**
 * A method for estimating how similar two nodes (words) are.
 *
 * @param h1, h2           The two WordNodes being compared
 * @param target_at_first  A flag indicates whether the h1 is from the target
 * @return                 A similarity score between the two nodes
 */
double Fuzzy::get_score(const Handle& h1, const Handle& h2, bool target_at_first)
{
    std::pair<UUID, UUID> p(h1.value(), h2.value());

    if (scores.find(p) != scores.end())
        return scores.at(p);

    double score = 0;
    bool proceed = false;

    // Consider they are the same if two nodes are identical
    // TODO: Should check the semantic as well
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
