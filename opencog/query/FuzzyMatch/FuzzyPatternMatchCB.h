/*
 * FuzzyPatternMatchCB.h
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

#ifndef FUZZYPATTERNMATCHCB_H
#define FUZZYPATTERNMATCHCB_H

#include <opencog/query/DefaultPatternMatchCB.h>
#include "GraphEditDist.h"

namespace opencog
{
    class FuzzyPatternMatchCB : public DefaultPatternMatchCB
    {
        public:
            // Storing the hypergraphs that are similar to the input one
            HandleSeq solns;

            FuzzyPatternMatchCB(AtomSpace* as);

            virtual void initiate_search(PatternMatchEngine* pme,
                                        const std::set<Handle>& vars,
                                        const std::vector<Handle>& clauses);
            virtual bool link_match(const LinkPtr& pLink, const LinkPtr& gLink);
            virtual bool node_match(const Handle& pNode, const Handle& gNode);
            virtual bool grounding(const std::map<Handle, Handle>& var_soln,
                                   const std::map<Handle, Handle>& pred_soln);

        private:
            // Min. edit distance of the query hypergraph and the candidate
            double cand_edit_dist = 0;

            // Min. edit distance among all the candidates
            double min_edit_dist = SIZE_MAX;

            GraphEditDist ged;
            bool check_if_accept(const Handle& ph, const Handle& gh);
    };
}

#endif // FUZZYPATTERNMATCHCB_H
