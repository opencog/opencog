/*
 * GraphEditDist.h
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

#ifndef GRAPHEDITDIST_H
#define GRAPHEDITDIST_H

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/Node.h>

namespace opencog
{
    /**
     * Estimate how similar two hypergraphs are by computing the edit distance
     * between them.
     */
    class GraphEditDist
    {
        public:
            GraphEditDist();
            double compute(const Handle& hg1, const Handle& hg2);

        private:
            /**
             * The (max.) costs of performaing different operations for the
             * edit distance computation.
             *
             * The costs are in the range of 0 to 1.
             *
             * More can be added if necessary.
             */
            double INSERT_COST = 1;
            double DELETE_COST = 1;
            double REPLACE_COST = 1;

            void examine_graph(const Handle& hg, HandleSeq& hg_seq);
            double get_replacement_cost(const Handle& h1, const Handle& h2);
            bool is_same_name(const NodePtr& node1, const NodePtr& node2);
    };
}

#endif // GRAPHEDITDIST_H
