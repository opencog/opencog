/*
 * opencog/embodiment/Learning/FitnessEstimator/SizePenalty.cc
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


#include "SizePenalty.h"
#include <opencog/util/exceptions.h>
#include "DistortedComboSize.h"
#include <opencog/util/Logger.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

namespace FitnessEstimator
{

SizePenalty::SizePenalty(const std::set<opencog::combo::definite_object>& dos,
                         int indefinite_object_count, int operator_count,
                         int predicate_count, int action_count)
        : _dos(dos)
{
    a = opencog::config().get_double("SIZE_PENALTY_COEF_A");
    b = opencog::config().get_double("SIZE_PENALTY_COEF_B");
    setc(indefinite_object_count, operator_count,
         predicate_count, action_count);
}

SizePenalty::~SizePenalty() { }

//Occam's razor factor
//tends to 0 when the size of the combo tends to infinity
//tends to 1 when the size of the combo tends to 1
double SizePenalty::computeSizePenalty(const opencog::combo::combo_tree& tr) const
{
    OC_ASSERT(!tr.empty(),
                     "SizePenalty - combo_tree should not be empty.");
    int s = DistortedComboSize::size(tr, _dos);

    //debug log for SPCTools
    opencog::logger().debug("SizePenalty - SPCTools - Combo size : %d", s);
    //~debug log for SPCTools

    return std::exp(-a*std::log(b*c + std::exp(1))*(double)s);
}

void SizePenalty::update(int indefinite_object_count, int operator_count,
                         int condition_count, int action_count)
{
    setc(indefinite_object_count, operator_count,
         condition_count, action_count);
}

void SizePenalty::setc(int indefinite_object_count, int operator_count,
                       int condition_count, int action_count)
{
    int sum = (_dos.size() + indefinite_object_count
               + operator_count + condition_count + action_count);
    OC_ASSERT(sum > 0,
                     "SizePenalty - sum should be greater than 0.");
    c = (double)(sum);
}

}//~namespace FitnessEstimator
