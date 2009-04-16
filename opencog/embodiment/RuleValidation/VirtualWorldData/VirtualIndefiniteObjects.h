/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualIndefiniteObjects.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef VIRTUAL_INDEFINITE_OBJ_H
#define VIRTUAL_INDEFINITE_OBJ_H

#include <string>

namespace VirtualWorldData
{

struct IndefiniteObjects {

    std::string nearest_object;
    std::string nearest_edible;
    std::string nearest_movable;
    std::string nearest_pickupable;
    std::string nearest_drinkable;
    std::string nearest_avatar;
    std::string nearest_pet;
    std::string nearest_small;
    std::string nearest_moving;
    std::string nearest_friendly;
    std::string nearest_poo_place;
    std::string nearest_pee_place;
    std::string nearest_noisy;

    std::string random_object;
    std::string random_edible;
    std::string random_movable;
    std::string random_pickupable;
    std::string random_drinkable;
    std::string random_avatar;
    std::string random_pet;
    std::string random_small;
    std::string random_moving;
    std::string random_friendly;
    std::string random_poo_place;
    std::string random_pee_place;
    std::string random_noisy;

    std::string food_bowl;
    std::string water_bowl;
    std::string pet_home;
    std::string pet_bowl;
    std::string last_food_place;
    std::string exemplar_avatar;

}; // struct
}  // namespace

#endif
