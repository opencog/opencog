/*
 * opencog/embodiment/Learning/PetaverseHC/HCPetaverseVocabularyProvider.h
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

#ifndef _HC_PETAVERSE_VOCABULARY_PROVIDER_H
#define _HC_PETAVERSE_VOCABULARY_PROVIDER_H

#include <opencog/embodiment/Learning/ImitationLearningAPI/PetaverseVocabularyProvider.h>

namespace opencog { namespace hillclimbing {

using namespace combo;

//------------------------------------------------------------------------
// Definitions of the operators, actions and perceptions used in petaverse
// for imitation learning
//------------------------------------------------------------------------

/**
 * operators used for petaverse hillclimber
 */

static const vertex _elementary_operators[] = {
    id::sequential_and,
    id::action_boolean_if,
    // id::action_action_if,
    // id::action_while,
    id::boolean_while,
    // id::action_not,
    id::logical_not
};

static const unsigned int _elementary_operators_size =
    sizeof(_elementary_operators) / sizeof(vertex);


/**
 * action and perception set for petaverse hillclimber
 */

static const avatar_builtin_action_enum _elementary_actions[] = {
    id::goto_obj,
    id::step_forward,
    id::step_backward,
    id::random_step,
    id::rotate_left,
    id::rotate_right,
    id::jump_up,
    id::grab,
    id::drop,
    id::sit,
    id::kick_left,
    id::kick_right
};

static const unsigned int _elementary_actions_size =
    sizeof(_elementary_actions) / sizeof(avatar_builtin_action_enum);

static const avatar_perception_enum _elementary_perceptions[] = {
    id::exists_edible,
    id::is_avatar,
    // id::is_moving,
    id::near,
    // id::above,
    // id::below,
    // id::inside
    id::has_said,
    id::is_last_agent_action
};

static const unsigned int _elementary_perceptions_size =
    sizeof(_elementary_perceptions) / sizeof(avatar_perception_enum);

static const avatar_indefinite_object_enum _indefinite_objects[] = {
    // id::random_object,
    // id::nearest_object
};

static const unsigned int _indefinite_objects_size =
    sizeof(_indefinite_objects) / sizeof(avatar_indefinite_object_enum);

class HCPetaverseVocabularyProvider : public PetaverseVocabularyProviderBase
{

public:

    // ctor, dtor

    HCPetaverseVocabularyProvider();
    ~HCPetaverseVocabularyProvider() {}

    // access methods

    // return a reference of the set of operators
    const operator_set& get_elementary_operators() const;

    // return a reference of the set of actions
    const builtin_action_set& get_elementary_actions() const;

    // return a reference of the set of perceptions
    const perception_set& get_elementary_perceptions() const;

    // return a reference of the set of indefinite objects
    const indefinite_object_set& get_indefinite_objects() const;

private:
    operator_set eo; // elementary operators
    builtin_action_set ea; // elementary builtin actions
    perception_set ep; // elementary perceptions
    indefinite_object_set is; // indefinite objects
};

} // ~namespace opencog
} // ~namespace hillclimbing

#endif

