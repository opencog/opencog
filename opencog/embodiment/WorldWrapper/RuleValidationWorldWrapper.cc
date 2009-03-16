#include "RuleValidationWorldWrapper.h"
#include "WorldWrapperUtilMock.h"

using namespace WorldWrapper;

RuleValidationWorldWrapper::RuleValidationWorldWrapper(VirtualWorldData::VirtualWorldState& _vw) : 
                            virtualWorld(_vw) {}

RuleValidationWorldWrapper::~RuleValidationWorldWrapper(){
}

bool RuleValidationWorldWrapper::isPlanFinished() const{
    // always finished - remember, this is a MOCK
    return true;
}

bool RuleValidationWorldWrapper::isPlanFailed() const {
    // never failed - remember, this is a MOCK
    return false;
}

bool RuleValidationWorldWrapper::sendSequential_and(sib_it from, sib_it to){
    // no sequential_and is sent, but the answer is always true - remember, this is a MOCK
    return true;
}

/**
 * @override
 */
combo::vertex RuleValidationWorldWrapper::evalPerception(pre_it it, combo::variable_unifier& vu){
    return WorldWrapperUtilMock::evalPerception(it, virtualWorld, vu);
}

/**
 * @override
 */
combo::vertex RuleValidationWorldWrapper::evalIndefiniteObject(combo::indefinite_object io, combo::variable_unifier& vu){
    return WorldWrapperUtilMock::evalIndefiniteObject(io, virtualWorld, vu);
}


