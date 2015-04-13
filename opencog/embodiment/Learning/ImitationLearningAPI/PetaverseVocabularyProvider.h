/*
 * opencog/embodiment/Learning/ImitationLearningAPI/PetaverseVocabularyProvider.h
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


/**
 * Abstract class to be inherited to use a learning algo in the context
 * of petaverse imitation learning, the inherited class will be used by
 * ImitationLearningTask and provides the methods needed by the EntropyFilter
 * and ActionFilter to determine the set of atomic actions and perceptions
 * involved in imitation learning. The operator set is still requiered to
 * compute sizePenalty function in the fitness estimator. It should contain
 * only non action, non perception, non indefinite_object operators.
 */
#ifndef _PETAVERSE_VOCABULARY_PROVIDER_BASE_H
#define _PETAVERSE_VOCABULARY_PROVIDER_BASE_H

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

class PetaverseVocabularyProviderBase
{

public:

    typedef std::set<opencog::combo::vertex> operator_set;
    typedef std::set<opencog::combo::indefinite_object> indefinite_object_set;

    //ctor, dtor

    PetaverseVocabularyProviderBase() {}
    virtual ~PetaverseVocabularyProviderBase() {}

    //access methods

    //return a reference of the set of operators
    virtual const operator_set& get_elementary_operators() const = 0;

    //return a reference of the set of actions
    virtual const opencog::combo::builtin_action_set& get_elementary_actions() const = 0;

    //return a reference of the set of perceptions
    virtual const opencog::combo::perception_set& get_elementary_perceptions() const = 0;

    //return a reference of the set of indefinite objects
    virtual const indefinite_object_set& get_indefinite_objects() const = 0;
};

#endif
