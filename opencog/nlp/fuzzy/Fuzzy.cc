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
 * Examine a tree and get all the word instance nodes associate with its nodes.
 *
 * @param h           The tree being examined
 * @param word_insts  A list of WordInstanceNodes
 */
static void get_all_word_insts(const Handle& h, HandleSeq& wi)
{
    if (h->isNode())
    {
        for (const Handle& w : get_target_neighbors(h, REFERENCE_LINK))
        {
            if (w->getType() == WORD_INSTANCE_NODE and
                h->getName() == w->getName() and
                std::find(wi.begin(), wi.end(), w) == wi.end())
            {
                wi.emplace_back(w);
            }
        }

        return;
    }

    for (const Handle& o : h->getOutgoingSet())
        get_all_word_insts(o, wi);
}

/**
 * Get the WordNode associates with the WordInstanceNode via a LemmaLink,
 * assuming that it's in this form:
 *
 * LemmaLink
 *   WordInstanceNode "like@123"
 *   WordNode "like"
 *
 * @param h  The WordInstanceNode
 */
static Handle get_word(const Handle& h)
{
    return get_target_neighbors(h, LEMMA_LINK)[0];
}

/**
 * Compare the WordNode associates with the WordInstanceNode
 *
 * @param h1, h2  Handles of the nodes being compared
 */
static bool compare_word(const Handle& h1, const Handle& h2)
{
    return get_word(h1) < get_word(h2);
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

    HandleSeq h2_word_insts;
    get_all_word_insts(h2, h2_word_insts);

    // TODO
    // std::sort(h2_word_insts.begin(), h2_word_insts.end());

    double score = 0;

    /* TODO
    if (target_words.size() > h2_words.size())
        compare(h2_words, target_words, 0, score, false);
    else
        compare(target_words, h2_words, 0, score, true);

    score /= std::max(target_words.size(), h2_words.size());
    */

    return score;
}

/**
 * A function intends to reflect how important a word is to a
 * document in a collection or corpus, which will be used in
 * the similarity estimation.
 *
 * @param words  A list of words (WordNode) of a sentence
 */
void Fuzzy::calculate_tfidf(const HandleSeq& word_insts)
{
    double min = 0;
    double max = 0;

    // No. of words in the sentence
    int num_of_words = word_insts.size();

    // Total no. of sentences in the AtomSpace
    size_t num_of_sents = (size_t) as->get_num_atoms_of_type(SENTENCE_NODE);

    for (const Handle& wi : word_insts)
    {
        if (tfidf_weights.count(wi)) continue;

        auto word_match = [&](const Handle& h)
        {
            return get_word(wi) == get_word(h);
        };

        // No. of times this word exists in the sentence
        int word_cnt = std::count_if(word_insts.begin(), word_insts.end(), word_match);

        OrderedHandleSet hs;
        for (const Handle& p : get_target_neighbors(wi, WORD_INSTANCE_LINK))
        {
            const HandleSeq& sent_nodes = get_target_neighbors(p, PARSE_LINK);
            hs.insert(sent_nodes.begin(), sent_nodes.end());
        }

        // No. of sentences that contain this word
        size_t num_sents_contains_it = hs.size();

        double tf = (double) word_cnt / num_of_words;
        double idf = log2((double) num_of_sents / num_sents_contains_it);
        double tfidf = tf * idf;

        tfidf_weights[wi] = tfidf;

        if (tfidf < min) min = tfidf;
        if (tfidf > max) max = tfidf;
    }

    // Normalize the values
    if (min != max)
        for (auto i = tfidf_weights.begin(); i != tfidf_weights.end(); i++)
            i->second = (i->second - min) / (max - min);
}

void Fuzzy::get_ling_rel(const HandleSeq& hs)
{
    for (const Handle& h : hs)
    {
        HandleSeq evals = get_predicates(h, DEFINED_LINGUISTIC_RELATIONSHIP_NODE);

        for (const Handle& el : evals)
        {
            // Extract the relation
            std::string ling_rel = (el->getOutgoingSet()[0])->getName();

            // Assign weights accordingly, subject to change
            if (ling_rel.compare("_subj") == 0 or
                ling_rel.compare("_obj") == 0 or
                ling_rel.compare("_predadj") == 0)
            {
                ling_rel_weights[h] = LINGUISTIC_RELATION_WEIGHT;
                break;
            }
        }
    }
}

/**
 * Override the start_search() method, to get words instead of nodes.
 *
 * @param np  A NodePtr pointing to a node in the pattern
 */
void Fuzzy::start_search(const Handle& trg)
{
    target = trg;
    get_all_word_insts(target, target_word_insts);
    calculate_tfidf(target_word_insts);
    get_ling_rel(target_word_insts);
    std::sort(target_word_insts.begin(), target_word_insts.end(), compare_word);
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
    if (hp->isLink())
        return false;

    Type t = hp->getType();

    return (t == CONCEPT_NODE or t == PREDICATE_NODE) and
           hp->getName().find("@") == std::string::npos;
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
    if (_af_only and soln->getSTI() < as->get_attentional_focus_boundary())
        return false;

    if (target == soln)
        return false;

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

    HandleSeq soln_word_insts;
    get_all_word_insts(soln, soln_word_insts);
    std::sort(soln_word_insts.begin(), soln_word_insts.end(), compare_word);

    HandleSeq common_words;
    std::set_intersection(target_word_insts.begin(), target_word_insts.end(),
                          soln_word_insts.begin(), soln_word_insts.end(),
                          std::back_inserter(common_words), compare_word);

    // Reject if it's identical to the input
    if (common_words.size() == target_word_insts.size() and
        common_words.size() == soln_word_insts.size())
        return false;

    // Initial value
    double score = 0;

    for (const Handle& c : common_words)
        score += get_score(c);

    score /= std::max(target_word_insts.size(), soln_word_insts.size());

    // Accept and store the solution
    if (score > 0)
        solns.push_back(std::make_pair(soln, score));

    return true;
}

/**
 * Get the score of a node that exists in both the target and potential solution
 *
 * @param h  The node for getting the score, should be a WordInstanceNode
 * @return   The score of the node
 */
double Fuzzy::get_score(const Handle& h)
{
    if (scores.find(h) != scores.end())
        return scores.at(h);

    // The default value for a node
    double score = NODE_WEIGHT;
    score += tfidf_weights[h] * RARENESS_WEIGHT;
    score += ling_rel_weights[h];

    scores[h] = score;

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
