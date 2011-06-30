/*
 * opencog/embodiment/WorldWrapper/WorldWrapperUtilMock.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#ifndef _WORLD_WRAPPER_UTIL_MOCK_H
#define _WORLD_WRAPPER_UTIL_MOCK_H

#include <opencog/util/exceptions.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/variable_unifier.h>

#include <opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldState.h>
#include <opencog/embodiment/PetComboVocabulary/PetComboVocabulary.h>

namespace opencog { namespace world {

/**
 *
 */
class WorldWrapperUtilMock
{


public:

    // pet perception maps, iterator and pair
    typedef std::map < combo::pet_perception_enum,  float (VirtualWorldData::VirtualAgent::*)() const >
    PetPerception;
    typedef std::map < combo::pet_perception_enum,  float (VirtualWorldData::VirtualAgent::*)() const >::const_iterator
    PetPerceptionIt;
    typedef std::pair < combo::pet_perception_enum, float (VirtualWorldData::VirtualAgent::*)() const >
    PetPerceptionPair;

    // pet perception maps, iterator and pair
    typedef std::map < combo::pet_perception_enum,  bool (VirtualWorldData::VirtualAgent::*)() const >
    BoolPetPerception;
    typedef std::map < combo::pet_perception_enum,  bool (VirtualWorldData::VirtualAgent::*)() const >::const_iterator
    BoolPetPerceptionIt;
    typedef std::pair < combo::pet_perception_enum, bool (VirtualWorldData::VirtualAgent::*)() const >
    BoolPetPerceptionPair;


    // entity perception map, iterator and pair
    typedef std::map < combo::pet_perception_enum,  bool (VirtualWorldData::VirtualEntity::*)() const >
    EntityPerception;
    typedef std::map < combo::pet_perception_enum,  bool (VirtualWorldData::VirtualEntity::*)() const >::const_iterator
    EntityPerceptionIt;
    typedef std::pair < combo::pet_perception_enum, bool (VirtualWorldData::VirtualEntity::*)() const >
    EntityPerceptionPair;

    // entity perception map, iterator and pair
    typedef std::map < combo::pet_perception_enum,  bool (VirtualWorldData::VirtualWorldState::*)(const std::string&, const std::string&) const > WorldPerception;
    typedef std::map < combo::pet_perception_enum,  bool (VirtualWorldData::VirtualWorldState::*)(const std::string&, const std::string&) const >::const_iterator WorldPerceptionIt;
    typedef std::pair < combo::pet_perception_enum, bool (VirtualWorldData::VirtualWorldState::*)(const std::string&, const std::string&) const > WorldPerceptionPair;

    /**
     * evalIndefiniteObject
     * eval indefinite object with a given spaceMap and a given AtomSpace
     * WARNING :It does not return the special definite_object self and owner
     * instead it returns their corresponding atom name
     *
     * @param vu      variable_unifier object used to resolve wild_card symbol _*_
     */
    static combo::vertex evalIndefiniteObject(combo::indefinite_object io,
            VirtualWorldData::VirtualWorldState & vw,
            combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    /**
     * evalIndefiniteObject
     * eval indefinite object with a given spaceMap and a given AtomSpace
     * WARNING :It does not return the special definite_object self and owner
     * instead it returns their corresponding atom name
     *
     * @param vu      variable_unifier object used to resolve wild_card symbol _*_
     */
    static combo::vertex evalIndefiniteObject(combo::pet_indefinite_object_enum ioe,
            VirtualWorldData::VirtualWorldState & vw,
            combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    /**
     * evalPerception
     * eval perception with a given spaceMap and a given AtomSpace
     * @param vu      variable_unifier object used to resolve wild_card symbol _*_
     */
    static combo::vertex evalPerception(const combo::combo_tree::iterator it,
                                        VirtualWorldData::VirtualWorldState & virtualWorld,
                                        combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

private:

    static bool initialized;

    static void initializeMaps();

    // map for VirtualAgent pointers to member functions
    static PetPerception petPerception;

    // map for VirtualAgent pointers to member functions
    static BoolPetPerception boolPetPerception;

    // map for VirtualEntity pointers to member functions
    static EntityPerception entityPerception;

    // map for VirtualWorldState pointers to member functions
    static WorldPerception worldPerception;

    // this vertex would be a definite_object, an indefinite_object or a
    // wild_card (_*_). Definite_object and indefinite_objects will produce
    // a one-element vector while wild_card will result in a multi-valued
    // vector.
    static std::vector<combo::definite_object> getDefiniteObjects(combo::vertex& v,
            VirtualWorldData::VirtualWorldState & virtualWorld,
            combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

};

} } // namespace opencog::world

#endif
