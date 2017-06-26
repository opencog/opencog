/*
 * CombinationGenerator.h
 *
 * Copyright (C) 2016 OpenCog Foundation
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

#ifndef _OPENCOG_COMBINATIONGENERATOR_H
#define _OPENCOG_COMBINATIONGENERATOR_H

#include <vector>
#include <stdexcept>

namespace opencog
{

/**
 * Implements an iterator to all the combinations of N elements of a set taken K
 * by K.
 *
 * For example:
 *
 *   CombinationGenerator comb(3);
 *   while (! comb.depleted()) {
 *       comb.printForDebug("", "\n");
 *       comb.generateNext();
 *   }
 *
 * Whould give us:
 *
 *   (0 0 0)
 *   (1 0 0)
 *   (0 1 0)
 *   (1 1 0)
 *   (0 0 1)
 *   (1 0 1)
 *   (0 1 1)
 *   (1 1 1)
 *
 *   CombinationGenerator comb(5, (unsigned int) 3);
 *   while (! comb.depleted()) {
 *       comb.printForDebug("", "\n");
 *       comb.generateNext();
 *   }
 *
 * Whould give us:
 *
 *   (1 1 1 0 0)
 *   (1 1 0 1 0)
 *   (1 0 1 1 0)
 *   (0 1 1 1 0)
 *   (1 1 0 0 1)
 *   ...
 *   ...
 *   (0 0 1 1 1)
 *
 * At any iteration on may use at(pos) to check whether a given element is selected
 * for that iteration or not.
 *
 */
class CombinationGenerator 
{

public:

    /*
     * avoidAllZero = true excludes the element (0 0 ... 0) 
     * avoidAllOne = true excludes the element (1 1 ... 1) 
     */
    CombinationGenerator(unsigned int n, bool avoidAllZero = false, bool avoidAllOne = false);
    CombinationGenerator(unsigned int n, unsigned int k);

    ~CombinationGenerator();

    bool at(unsigned int pos) const;
    bool depleted() const;
    void generateNext();

    void printForDebug(std::string prefix, std::string suffix) const;

private:

    enum Algorithms { ALL_COMB, K_COMB };

    enum Algorithms combinationAlgorithm;
    bool depletedFlag;
    bool avoidAllZero;
    bool avoidAllOne;
    std::vector<bool> elements;
    unsigned int countOnes;

    void generateNextAllComb();
    void generateNextKComb();

};

}

#endif // _OPENCOG_COMBINATIONGENERATOR_H
