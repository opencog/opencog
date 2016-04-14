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
    FuzzyMatchBasic::start_search(trg);
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

static void get_all_nodes(const Handle& h, HandleSeq& node_list)
{
   LinkPtr lll(LinkCast(h));
   if (nullptr == lll)
   {
      node_list.emplace_back(h);
      return;
   }

   for (const Handle& o : lll->getOutgoingSet())
      get_all_nodes(o, node_list);
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

    HandleSeq soln_nodes;
    get_all_nodes(soln, soln_nodes);

    HandleSeq common_nodes;
    std::sort(soln_nodes.begin(), soln_nodes.end());

    // Find out how many nodes it has in common with the pattern
    std::set_intersection(target_nodes.begin(), target_nodes.end(),
                          soln_nodes.begin(), soln_nodes.end(),
                          std::back_inserter(common_nodes));

    std::sort(common_nodes.begin(), common_nodes.end());

    // Remove duplicate common_nodes
// XXX  FIXME .. how can there possibly be duplicates ???
// All target nodes are unique ... !?
    common_nodes.erase(std::unique(common_nodes.begin(), common_nodes.end()),
                       common_nodes.end());

    double similarity = 0;

    for (const Handle& common_node : common_nodes) {
        // If both the pattern and the potential solution share some "instance"
        // node, that probably means the potential solution is a sub-pattern
        // of the input pattern, or is some related atoms that is generated
        // with the pattern, which is pretty likely not what we want here
        if (common_node->getName().find("@") != std::string::npos)
            return false;

        double weight = 0.25;

        // Helper function for getting the instance of a ConceptNode or a
        // PredicateNode from the input pattern.
        // e.g. Getting (PredicateNode "is@123") from (PredicateNode "be")
        // or (ConceptNode "cats@123") from (ConceptNode "cat") etc
        auto find_instance = [&] (Handle n) {
            if (n->getName().find("@") == std::string::npos)
                return false;

            for (const Handle& ll : target->getOutgoingSet()) {
                if (is_atom_in_tree(ll, common_node) and is_atom_in_tree(ll, n)) {
                    if ((common_node->getType() == PREDICATE_NODE and
                         ll->getType() == IMPLICATION_LINK) or
                        (common_node->getType() == CONCEPT_NODE and
                         ll->getType() == INHERITANCE_LINK))
                        return true;
                }
            }

            return false;
        };

        auto inst = std::find_if(target_nodes.begin(), target_nodes.end(), find_instance);

        if (inst != target_nodes.end()) {
            // Get the WordInstanceNode from the ReferenceLink that connects both
            // WordInstanceNode and the instance ConceptNode / PredicateNode
            HandleSeq word_inst = get_target_neighbors(*inst, REFERENCE_LINK);

            if (word_inst.size() > 0) {
                // Get the DefinedLinguisticRelationshipNode from the EvaluationLink
                HandleSeq eval_links =
                    get_predicates(word_inst[0], DEFINED_LINGUISTIC_RELATIONSHIP_NODE);

                for (const Handle& el : eval_links) {
                    // Extract the relation
                    std::string ling_rel = (el->getOutgoingSet()[0])->getName();

                    // Assign weights accordingly, subject to change if needed
                    if (ling_rel.compare("_subj") == 0)
                        weight += 1.0;
                    else if (ling_rel.compare("_obj") == 0)
                        weight += 1.0;
                    else if (ling_rel.compare("_predadj") == 0)
                        weight += 1.0;
                    else
                        weight += 0.5;
                }
            }
        }

        similarity += weight;
        // similarity += (weight / common_node->getIncomingSetSize());
    }

    size_t max_size = std::max(target_nodes.size(), soln_nodes.size());

    // Also take the size into account
    similarity /= max_size;

    // Accept and store the solution
    solns.push_back(std::make_pair(soln, similarity));

    return true;
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
