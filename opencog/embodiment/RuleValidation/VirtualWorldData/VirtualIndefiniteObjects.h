#ifndef VIRTUAL_INDEFINITE_OBJ_H
#define VIRTUAL_INDEFINITE_OBJ_H

#include <string>

namespace VirtualWorldData {

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
