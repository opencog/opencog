/** 
 * IsSmallPredicateUpdater.cc
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
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
