/*
 * opencog/embodiment/Learning/PetaverseHC/HCPetaverseVocabularyProvider.cc
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

#include "HCPetaverseVocabularyProvider.h"
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

namespace opencog { namespace hillclimbing {

using namespace AvatarCombo;

HCPetaverseVocabularyProvider::HCPetaverseVocabularyProvider()
{
    //elementary operators
    if (eo.empty()) {
        for (unsigned int i = 0; i < _elementary_operators_size; i++)
            eo.insert(_elementary_operators[i]);
    }
    //elementary builtin actions
    if (ea.empty()) {
        for (unsigned int i = 0; i < _elementary_actions_size; i++)
            ea.insert(get_instance(_elementary_actions[i]));
    }
    //elementary perceptions
    if (ep.empty()) {
        for (unsigned int i = 0; i < _elementary_perceptions_size; i++)
            ep.insert(get_instance(_elementary_perceptions[i]));
    }
    //indefinite objects
    if (is.empty()) {
        for (unsigned int i = 0; i < _indefinite_objects_size; i++)
            is.insert(get_instance(_indefinite_objects[i]));
    }
}

//return a reference of the set of operators
const PetaverseVocabularyProviderBase::operator_set&
HCPetaverseVocabularyProvider::get_elementary_operators() const
{
    return eo;
}

//return a reference of the set of actions
const builtin_action_set& HCPetaverseVocabularyProvider::get_elementary_actions() const
{
    return ea;
}

//return a reference of the set of perceptions
const perception_set& HCPetaverseVocabularyProvider::get_elementary_perceptions() const
{
    return ep;
}

//return a reference of the set of indefinite objects
const PetaverseVocabularyProviderBase::indefinite_object_set&
HCPetaverseVocabularyProvider::get_indefinite_objects() const
{
    return is;
}

} // ~namespace opencog
} // ~namespace hillclimbing
