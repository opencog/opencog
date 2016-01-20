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
 * @param dd  A flag indicating if duplicate solns should be returned
 */
Fuzzy::Fuzzy(Type tt, const HandleSeq& ll, bool dd) :
    rtn_type(tt),
    excl_list(ll),
    dup_check(dd)
{
}

Fuzzy::~Fuzzy()
{
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
    NodePtr np(NodeCast(hp));
    if (nullptr == np) return false;

    return
       (np->getType() == CONCEPT_NODE or np->getType() == PREDICATE_NODE)
       and (np->getName().find("@") == std::string::npos);
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

    // Check if this is the type of atom that we want
    if (soln->getType() != rtn_type)
        return false;

    HandleSeq soln_nodes;
    get_all_nodes(soln, soln_nodes);

    // Check if it contains any unwanted atoms
    if (std::any_of(soln_nodes.begin(), soln_nodes.end(),
        [&](Handle& h) {
            return std::find(excl_list.begin(), excl_list.end(), h)
                             != excl_list.end(); }))
        return false;

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
        if (NodeCast(common_node)->getName().find("@") != std::string::npos)
            return false;

        double weight = 0.25;

        // Helper function for getting the instance of a ConceptNode or a
        // PredicateNode from the input pattern.
        // e.g. Getting (PredicateNode "is@123") from (PredicateNode "be")
        // or (ConceptNode "cats@123") from (ConceptNode "cat") etc
        auto find_instance = [&] (Handle n) {
            if (NodeCast(n)->getName().find("@") == std::string::npos)
                return false;

            for (const Handle& ll : LinkCast(target)->getOutgoingSet()) {
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

    size_t max_size = std::max(target_nodes.size(), soln_nodes.size());

    // Also take the size into account
    similarity /= max_size;

    if (dup_check) {
        // Check the content of the soln to see if we've accepted something similar before
        soln_nodes.erase(std::remove_if(soln_nodes.begin(), soln_nodes.end(),
                        [](Handle& h) {
                            return NodeCast(h)->getName().find("@") != std::string::npos; }),
                        soln_nodes.end());

        if (std::find(solns_contents.begin(), solns_contents.end(), soln_nodes) != solns_contents.end())
            return false;

        solns_contents.push_back(soln_nodes);
    }

    // Accept and store the solution
    solns.push_back(std::make_pair(soln, similarity));

    return false;
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
