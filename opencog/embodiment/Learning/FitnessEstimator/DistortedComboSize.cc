/*
 * opencog/embodiment/Learning/FitnessEstimator/DistortedComboSize.cc
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

#include "DistortedComboSize.h"
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>
#include <opencog/util/Config.h>

//the distortion of the size of the random instruction is number of
//definite objects time below
#define RANDOM_DISTOR_FACTOR 2

//there are currently 4 steps for random_steps
#define RANDOM_STEP_NUMBER 4

namespace FitnessEstimator
{

using namespace AvatarCombo;

bool DistortedComboSizeOrder::operator()(const combo_tree& tr1,
        const combo_tree& tr2) const
{
    int s1 = DistortedComboSize::size(tr1, _dos);
    int s2 = DistortedComboSize::size(tr2, _dos);
    if (s1 == s2) {
        return (cmp(tr1.begin(), tr2.begin()) > 0);
    } else return (s1 < s2);
}

//TODO : it might be important to calibrate this using some meta learning
int DistortedComboSize::vertex_size(const vertex& v,
                                    const std::set<definite_object>& dos)
{

    static const int while_operator_size =
        opencog::config().get_int("WHILE_OPERATOR_SIZE");
    static const int conditional_size = 
        opencog::config().get_int("CONDITIONAL_SIZE");
    static const int contin_size =
        opencog::config().get_int("CONTIN_SIZE");

    //random operators
    if (v == get_instance(id::random_object))
        return (int)(std::log((double)(dos.size() + 1))
                     * RANDOM_DISTOR_FACTOR);
    else if (v == get_instance(id::random_step))
        return (int)(std::log((double)RANDOM_STEP_NUMBER)
                     * RANDOM_DISTOR_FACTOR);
    //while operators
    else if (v == id::boolean_while || v == id::action_while)
        return while_operator_size;
    //conditionals
    else if (v == id::action_boolean_if || v == id::action_action_if)
        return conditional_size;
    //contin
    else if (is_contin(v))
        return contin_size;
    else return 1;
}

int DistortedComboSize::size(const combo_tree& tr,
                             const std::set<definite_object>& dos)
{
    int i = 0;
    for (pre_it it = tr.begin(); it != tr.end(); ++it)
        i += vertex_size(*it, dos);
    return i;
}

}
