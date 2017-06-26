/*
 * CartesianProductGenerator.h
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#ifndef _OPENCOG_CARTESIANPRODUCTGENERATOR_H
#define _OPENCOG_CARTESIANPRODUCTGENERATOR_H

#include "vector"
#include <stdexcept>

namespace opencog
{

/*
 * Implements a way to iterate thru all possible points of a cartesian product.
 *
 * For example, given N sets M elements each.
 *
 *   CartesianProductGenerator cart(N, M);
 *   while (! cart.depleted()) {
 *       cart.printForDebug("", "\n");
 *       cart.generateNext();
 *   }
 *
 * this piece of code would give us:
 *
 * (0 0 ... 0 0)
 * (0 0 ... 0 1)
 * (0 0 ... 0 2)
 * ...
 * (0 0 ... 0 M)
 * (0 0 ... 1 0)
 * (0 0 ... 1 1)
 * ...
 * ...
 * (M M ... M M)
 *
 * At each point one may call at(pos) to get the selected element at a given
 * position. In the above example, in the second iteration we would have:
 *
 * at(0) ==> 0
 * at(1) ==> 0
 * ...
 * at(N - 2) ==> 0
 * at(N - 1) ==> 1
 *
 */
class CartesianProductGenerator 
{

public:

    /*
     * The first constructor is the one from the example above, all the sets
     * have the same cardinality. The second one is more generic as one can 
     * pass a vector with the cardinality of each set.
     *
     * avoidEqual = true makes at(i) != at(j) for all i != j (any iteration)
     * triangularFlag = true makes at(i) < at(j) for all i < j (any iteration)
     */
    CartesianProductGenerator(unsigned int n, unsigned int m, bool avoidEqual = false, bool triangularFlag = false);
    CartesianProductGenerator(const std::vector<unsigned int> &v, bool avoidEqual = false, bool triangularFlag = false);

    ~CartesianProductGenerator();
    bool depleted() const;
    unsigned int at(unsigned int pos) const;
    void generateNext();
    void drop(unsigned int pos);
    void printForDebug(std::string prefix, std::string suffix) const;

private:

    std::vector<unsigned int> cursorVector;
    std::vector<unsigned int> base;
    bool depletedFlag;
    bool avoidEqualFlag;
    bool triangularFlag;

    void init(const std::vector<unsigned int> &v);
    void checkForRepetition();
    void printBaseForDebug(std::string prefix, std::string suffix) const;

};

}

#endif // _OPENCOG_CARTESIANPRODUCTGENERATOR_H
