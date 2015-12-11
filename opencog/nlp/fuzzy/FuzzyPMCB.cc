/*
 * FuzzyPMCB.cc
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

#include <opencog/atomutils/AtomUtils.h>

#include "FuzzyPMCB.h"

using namespace opencog::nlp;
using namespace opencog;

FuzzyPMCB::FuzzyPMCB(AtomSpace* as, Type t, const HandleSeq& excl) :
    FuzzyPatternMatch(as, t, excl)
{
}

FuzzyPMCB::~FuzzyPMCB()
{
}

void FuzzyPMCB::similarity_match(const Handle& pat, const Handle& soln, HandleSeq& solns)
{
    // Weights for various part of speech ...
    // Pass from Scheme??

    // Find out how many nodes it has in common with the pattern
    HandleSeq common_nodes;
    HandleSeq pat_nodes = get_all_nodes(pat);
    HandleSeq soln_nodes = get_all_nodes(soln);

    std::sort(pat_nodes.begin(), pat_nodes.end());
    std::sort(soln_nodes.begin(), soln_nodes.end());

    std::set_intersection(pat_nodes.begin(), pat_nodes.end(),
                          soln_nodes.begin(), soln_nodes.end(),
                          std::back_inserter(common_nodes));

    // The size different between the pattern and the potential solution
    size_t diff = std::abs((int)(pat_nodes.size() - soln_nodes.size()));

    double similarity = 0;

    for (const Handle& common_node : common_nodes) {
        // InheritanceLink -> ConceptNode instance (inside some R2L SetLink?)
        // ConceptNode instance -> ReferenceLink
        // ReferenceLink -> WordInstanceNode
        // WordInstanceNode -> PartOfSpeechLink
        // PartOfSpeechLink -> DefinedLinguisticConceptNode
    }

    // Rank the results

    // Sort the results
}

