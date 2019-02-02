/*
 * FuzzyMatchBasic.h
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

#ifndef FUZZY_MATCH_BASIC_H
#define FUZZY_MATCH_BASIC_H

#include <opencog/nlp/fuzzy/FuzzyMatch.h>

namespace opencog
{
/**
 * The fuzzy matcher searches for trees which are similar but
 * not identical to the input target pattern. This is done by examining
 * all possible trees that have at least one leaf node in common with
 * the target pattern.  A similarity score is assigned to each such
 * tree, and the ones with the highest scores are returned.
 *
 * This class implements certain specific similarity score.
 */
class FuzzyMatchBasic : public FuzzyMatch
{
protected:
    virtual void start_search(const Handle&);
    virtual bool accept_starter(const Handle&);
    virtual bool try_match(const Handle&);
    virtual RankedHandleSeq finished_search(void);

    // What we are matching
    Handle target;
    HandleSeq target_nodes;
    HandleSeq target_atoms;

private:
    // The solutions that were found.
    RankedHandleSeq solns;

    // The minimum difference between the pattern and all
    // the known solutions
    size_t min_size_diff = SIZE_MAX;

    // The maximum similarity of all the potential solutions we found
    double max_similarity = -std::numeric_limits<double>::max();
};

} // namespace opencog
#endif  // FUZZY_MATCH_BASIC_H
