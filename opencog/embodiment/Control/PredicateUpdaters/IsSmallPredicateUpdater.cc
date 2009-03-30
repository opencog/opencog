/*
 * opencog/embodiment/Control/PredicateUpdaters/IsSmallPredicateUpdater.cc
 *
 * Copyright (C) 2007-2008 Carlos Lopes
 * All Rights Reserved
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
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include "AtomSpaceUtil.h"
#include "IsSmallPredicateUpdater.h"

using namespace OperationalPetController;
using namespace opencog;
 
IsSmallPredicateUpdater::IsSmallPredicateUpdater(AtomSpace & atomSpace) :
                         BasicPredicateUpdater(atomSpace){
}

IsSmallPredicateUpdater::~IsSmallPredicateUpdater(){
}

/**
 * Size predicate template
 * 
 * EvalLink
 *   PredicateNode:"size"
 *   ListLink
 *      SLObjectNode:"<obj_id">
 *      NumberNode:"<length>"
 *      NumberNode:"<width>"
 *      NumberNode:"<height>"
 */
double IsSmallPredicateUpdater::getSize(Handle object){
    double length = 0.0;
    double width  = 0.0;
    double height  = 0.0;

    bool result = AtomSpaceUtil::getSizeInfo(atomSpace, object, length, width, height);
    if(!result){
        return 0.0;
    }

    return (length * width * height);
}


void IsSmallPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp ){
    
    // an is_small predicate is already assigned for this object, just
    // return. This function is used to keep the predicates consistent
    // over time
    if(isUpdated(object, "is_small")){
        return;
    }

    logger().log(opencog::Logger::FINE, "IsSmall - Updating is_small for obj %s.", 
                    atomSpace.getName(object).c_str());        

    // truth value - mean equals 0.0 --> not smaller than pet
    //                 mean equals 1.0 --> is  smaller than pet 
    SimpleTruthValue tv(0.0, 1.0);
    
    // while there are no size information some assumptions 
    // will guide the is_small predicate

    // 1. all avatars are considered bigger than any pet
    if(atomSpace.getType(object) == SL_AVATAR_NODE){
        tv.setMean(0.0);
        AtomSpaceUtil::setPredicateValue(atomSpace, "is_small", tv, object);
        return;
    }
    
    // 2. all accessories are considered smaller than the pet
    if(atomSpace.getType(object) == SL_ACCESSORY_NODE){
        tv.setMean(1.0);
        AtomSpaceUtil::setPredicateValue(atomSpace, "is_small", tv, object);
        return;        
    }
    
    // 3. compare the size of the pet and the object and compare 
    if(getSize(pet) > getSize(object)){
        tv.setMean(1.0);        
    } else {
        tv.setMean(0.0);
    }
    AtomSpaceUtil::setPredicateValue(atomSpace, "is_small", tv, object);    
}
