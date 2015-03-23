/*
 * opencog/embodiment/Learning/FitnessEstimator/SizePenalty.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _SIZEPENALTY_H
#define _SIZEPENALTY_H

#include <opencog/comboreduct/combo/vertex.h>

namespace FitnessEstimator
{

class SizePenalty
{

private:
    double a;
    double b;
    double c;

    const std::set<opencog::combo::definite_object>& _dos;

    void setc(int indefinite_object_count, int operator_count,
              int condition_count, int action_count);

public:

    SizePenalty(const std::set<opencog::combo::definite_object>& dos,
                int indefinite_object_count = 0, int operator_count = 0,
                int predicate_count = 0, int action_count = 0);
    ~SizePenalty();

    //Occam's razor factor
    //tends to 0 when the size of the combo tends to infinity
    //tends to 1 when the size of the combo tends to 1
    double computeSizePenalty(const opencog::combo::combo_tree& tr) const;

    void update(int definite_object_count, int operator_count,
                int condition_count, int action_count);

};

}//~namespace FitnessEstimator

#endif
