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

#include <opencog/nlp/types/atom_types.h>
#include <opencog/atomutils/AtomUtils.h>
#include <opencog/atomutils/FindUtils.h>

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
Fuzzy::Fuzzy(AtomSpace* as, Type tt, const HandleSeq& ll) :
    FuzzyPatternMatch(as),
    rtn_type(tt),
    excl_list(ll)
{
}

Fuzzy::~Fuzzy()
{
}

/**
 * Override the set_pattern method.
 *
 * Set the pattern, extract all the nodes and sort them. This will be useful
 * in later stages.
 *
 * @param vars  The variables in the pattern
 * @param pat   The pattern
 */
void Fuzzy::set_pattern(const Variables& vars, const Pattern& pat)
{
    FuzzyPatternMatch::set_pattern(vars, pat);

    pattern = pat.mandatory[0];
    pat_nodes = get_all_nodes(pattern);
    std::sort(pat_nodes.begin(), pat_nodes.end());
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
bool Fuzzy::accept_starter(const NodePtr np)
{
    return FuzzyPatternMatch::accept_starter(np) and
           (np->getType() == CONCEPT_NODE or np->getType() == PREDICATE_NODE);
}

/**
 * Determine whether or not to accept a potential solution found by the fuzzy
 * pattern matcher.
 *
 * The potential solution has to be of the same type as the rtn_type, and
 * does not contain any unwanted atoms listed in the excl_list. To calculate
 * a similarity score, a list of common nodes will be obtained. Different weights
 * will be assigned to those with certain linguistic relations in a sentence.
 * Nodes with no linguistic relation will have a weight of zero at the moment,
 * along with those that are not in common with the pattern. The accepted
 * solutions will be stored in the solns vector.
 *
 * @param pat   The pattern
 * @param soln  The potential solution
 */
void Fuzzy::accept_solution(const Handle& pat, const Handle& soln)
{
    // Check if this is the type of atom that we want
    if (soln->getType() != rtn_type)
        return;

    HandleSeq soln_nodes = get_all_nodes(soln);

    // Check if it contains any unwanted atoms
    if (std::any_of(soln_nodes.begin(), soln_nodes.end(),
        [&](Handle& h) {
            return std::find(excl_list.begin(), excl_list.end(), h)
                             != excl_list.end(); }))
        return;

    HandleSeq common_nodes;
    std::sort(soln_nodes.begin(), soln_nodes.end());

    // Find out how many nodes it has in common with the pattern
    std::set_intersection(pat_nodes.begin(), pat_nodes.end(),
                          soln_nodes.begin(), soln_nodes.end(),
                          std::back_inserter(common_nodes));

    std::sort(common_nodes.begin(), common_nodes.end());

    // Remove duplicate common_nodes
    common_nodes.erase(std::unique(common_nodes.begin(), common_nodes.end()),
                       common_nodes.end());

    double similarity = 0;

    for (const Handle& common_node : common_nodes) {
        // If both the pattern and the potential solution share some "instance"
        // node, that probably means the potential solution is a sub-pattern
        // of the input pattern, or is some related atoms that is generated
        // with the pattern, which is pretty likely not what we want here
        if (NodeCast(common_node)->getName().find("@") != std::string::npos)
            return;

        double weight = 0;

        // Helper function for getting the instance of a ConceptNode or a
        // PredicateNode from the input pattern.
        // e.g. Getting (PredicateNode "is@123") from (PredicateNode "be")
        // or (ConceptNode "cats@123") from (ConceptNode "cat") etc
        auto find_instance = [&] (Handle n) {
            if (NodeCast(n)->getName().find("@") == std::string::npos)
                return false;

            for (const Handle& ll : LinkCast(pattern)->getOutgoingSet()) {
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

        auto inst = std::find_if(pat_nodes.begin(), pat_nodes.end(), find_instance);

        if (inst != pat_nodes.end()) {
            // Get the WordInstanceNode from the ReferenceLink that connects both
            // WordInstanceNode and the instance ConceptNode / PredicateNode
            HandleSeq word_inst = get_neighbors(*inst, false, true, REFERENCE_LINK, false);

            if (word_inst.size() > 0) {
                // Get the DefinedLinguisticRelationshipNode from the EvaluationLink
                HandleSeq eval_links =
                    get_predicates(word_inst[0], DEFINED_LINGUISTIC_RELATIONSHIP_NODE);

                for (const Handle& el : eval_links) {
                    // Extract the relation
                    std::string ling_rel =
                        NodeCast(LinkCast(el)->getOutgoingSet()[0])->getName();

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

    size_t max_size = std::max(pat_nodes.size(), soln_nodes.size());

    // Also take the size into account
    similarity /= max_size;

    // Check the content of the soln to see if we've accepted something similar before
    // soln_nodes.erase(std::remove_if(soln_nodes.begin(), soln_nodes.end(),
    //                  [](Handle& h) {
    //                      return NodeCast(h)->getName().find("@") != std::string::npos; }),
    //                  soln_nodes.end());

    // if (std::find(dup_check.begin(), dup_check.end(), soln_nodes) != dup_check.end())
    //     return;

    // dup_check.push_back(soln_nodes);

    // Accept and store the solution
    solns.push_back(std::make_pair(soln, similarity));
}

/**
 * Get method for getting the solutions sorted in descending order of similarity.
 *
 * @return  A vector of solutions
 */
std::vector<std::pair<Handle, double>> Fuzzy::get_solns()
{
    // Sort the solutions by their similarity scores
    std::sort(solns.begin(), solns.end(),
        [] (std::pair<Handle, double> s1, std::pair<Handle, double> s2) {
            return s1.second > s2.second;
    });

    return solns;
}

