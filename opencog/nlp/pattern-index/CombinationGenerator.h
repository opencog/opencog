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
 *
 */
class CombinationGenerator 
{

public:

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
