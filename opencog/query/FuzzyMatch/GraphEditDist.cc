/*
 * GraphEditDist.cc
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

#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/ProbabilisticTruthValue.h>
#include "GraphEditDist.h"

using namespace opencog;

//#define DEBUG

GraphEditDist::GraphEditDist()
{

}


/**
 * A dynamic programming approach for computing the edit distance between two
 * hypergraphs, i.e., how many operations do we need to transform one
 * hypergraph (hg1) into another one (hg2), which implies how similar the two
 * hypergraphs are. The available operations are:
 *
 *   Insertion    Inserting an atom into a hypergraph
 *   Deletion     Deleting an atom from a hypergraph
 *   Replacement  Replacing an atom with another one
 *
 * Each operation has a cost. The resulting edit distance is the min. cost of
 * the transformation. The smaller the edit distance, the more similar the two
 * hypergraphs are.
 *
 * Note: This function only computes the edit distance, no actual
 *       "hypergraph transformation" is taken place in the atomspace.
 *
 * @param hg1  A query hypergraph
 * @param hg2  Another hypergraph that's being compared with
 * @return     The min. cost (edit distance) of transforming hg1 into hg2
 */
double GraphEditDist::compute(const Handle& hg1, const Handle& hg2)
{
#ifdef DEBUG
    printf("Computing the edit distance between these two hypergraphs:\n\n%s\n%s",
            hg1->toShortString().c_str(), hg2->toShortString().c_str());
#endif

    // Examine the two hypergraphs and store their atoms into the
    // corresponding HandleSeq
    HandleSeq hg1_seq;
    HandleSeq hg2_seq;
    examine_graph(hg1, hg1_seq);
    examine_graph(hg2, hg2_seq);

    // Initialize the cost matrix
    size_t row_size = hg1_seq.size() + 1;
    size_t col_size = hg2_seq.size() + 1;
    double cost_matrix[row_size][col_size];

    // Compute the costs and store them in the cost matrix
    for (size_t i = 0; i < row_size; i++)
    {
        for (size_t j = 0; j < col_size; j++)
        {
            if (i == 0 and j == 0) cost_matrix[i][j] = 0;

            // Base case. It can be viewed as transforming a hypergraph from
            // one that contains no atoms, so the operations are solely insertions
            else if (i == 0)
            {
                cost_matrix[i][j] = j * INSERT_COST;
            }

            // Base case. It can be viewed as transforming a hypergraph into
            // one that contains no atoms, so the operations are solely deletions
            else if (j == 0)
            {
                cost_matrix[i][j] = i * DELETE_COST;
            }

            // At any point the cost[i][j] is the minimum of (c1, c2, c3)
            else
            {
                double c1 = cost_matrix[i-1][j] + DELETE_COST;
                double c2 = cost_matrix[i][j-1] + INSERT_COST;
                double c3 = cost_matrix[i-1][j-1]
                              + get_replacement_cost(hg1_seq[i-1], hg2_seq[j-1]);

                cost_matrix[i][j] = std::min(c1, std::min(c2, c3));
            }
        }
    }

#ifdef DEBUG
    // Print out the cost matrix
    printf("\nCost Matrix:\n");
    for (size_t i = 0; i < row_size; i++)
    {
        for (size_t j = 0; j < col_size; j++)
            printf("%.1f, ", cost_matrix[i][j]);
        printf("\n");
    }
    printf("\nEdit Distance = %.3f\n", cost_matrix[row_size-1][col_size-1]);
#endif

    return cost_matrix[row_size-1][col_size-1];
}


/**
 * Traverse the hypergraph and insert the atoms into a HandleSeq, which is then
 * used for the edit distance computation.
 *
 * @param hg      The hypergraph that's being traversed
 * @param hg_seq  HandleSeq that stores the atoms
 */
void GraphEditDist::examine_graph(const Handle& hg, HandleSeq& hg_seq)
{
    if (Handle::UNDEFINED == hg) return;

    hg_seq.push_back(hg);

    // Traverse its outgoing set if it is a link
    LinkPtr lp(LinkCast(hg));
    if (NULL != lp)
    {
        const HandleSeq& vh = lp->getOutgoingSet();
        size_t sz = vh.size();
        for (size_t i = 0; i < sz; i++)
            examine_graph(vh[i], hg_seq);
    }
}


/**
 * Estimate the cost of replacing an atom with another one.
 *
 * The replacement cost should be directly related to the similarity between
 * the two atoms. The more similar they are, the lower the replacement cost
 * would be.
 *
 * The current atom-similarity estimation:
 *   For links, check if they are having the same type.
 *   For nodes, check if they are having the same type and same name.
 *
 * Currently this similarity estimation is a bit crude. In the near future,
 * it will be replaced by some other heuristic, for instance truth values of
 * the atoms, in order to get more accurate results.
 *
 * @param h1  Handle of an atom from a hypergraph
 * @param h2  Handle of an atom from another hypergraph
 * @return    The cost of replacing h1 with h2
 */
double GraphEditDist::get_replacement_cost(const Handle& h1, const Handle& h2)
{
    // For links
    LinkPtr lp1(LinkCast(h1));
    LinkPtr lp2(LinkCast(h2));
    if (lp1 and lp2)
    {
        // Check if both of them are of the same type
        return (lp1->getType() != lp2->getType()) * REPLACE_COST;
    }

    // For nodes
    NodePtr np1(NodeCast(h1));
    NodePtr np2(NodeCast(h2));
    if (np1 and np2)
    {
        Type t1 = np1->getType();
        Type t2 = np2->getType();

        // Let's make an exception for VariableNode
        if (t1 == VARIABLE_NODE and t2 == VARIABLE_NODE) return 0;

        // If both types and names are different, cost = REPLACE_COST
        // If only one of them is different, cost = (REPLACE_COST / 2)
        // Else, they are considered to be identical, cost = 0
        return ((t1 != t2) + !is_same_name(np1, np2)) * (REPLACE_COST / 2);
    }

    // Else, they probably are a node and a link
    else return REPLACE_COST;
}


/**
 * Check if the names of the two nodes, ignoring their UUIDs, are identical.
 *
 * For instance,
 *
 *  (ConceptNode "she@13781c75-caf4-42c6-bcad-bc01e48ac657")
 *  (ConceptNode "she@21c4eb76-60f1-4c8d-acf9-26e88173222c")
 *
 * would be considered identical, i.e., returns true.
 *
 * A reason of doing this is to avoid matching similar hypergraphs with
 * "identical content", if any, which normally are not what we're expecting.
 *
 * @param node1  A query node
 * @param node2  Another node that's being compared with
 * @return       True if their names are identical, false if not
 */
bool GraphEditDist::is_same_name(const NodePtr& node1, const NodePtr& node2)
{
    size_t pos1 = node1->getName().find_first_of('@');
    size_t pos2 = node2->getName().find_first_of('@');

    return (node1->getName().substr(0, pos1) == node2->getName().substr(0, pos2));
}
