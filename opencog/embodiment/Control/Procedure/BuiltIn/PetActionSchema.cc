/*
 * opencog/embodiment/Control/Procedure/BuiltIn/PetActionSchema.cc
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

#include <opencog/util/Logger.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/comboreduct/combo/iostream_combo.h>
#include <opencog/spacetime/atom_types.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PVPXmlConstants.h>
#include "PetActionSchema.h"

namespace opencog { namespace Procedure {

using namespace pai;

PetActionSchema::PetActionSchema(PAI& _pai, const ActionType& _actionType) : pai(_pai), actionType(_actionType)
{
    name =  StringManipulator::toUpper(actionType.getName());

    minArity = actionType.getMandatoryParamSize();
    optionalArity = actionType.getOptionalParamSize();
}

PetActionSchema::~PetActionSchema() {}

const std::string& PetActionSchema::getName() const
{
    return name;
}

bool PetActionSchema::isPetAction() const
{
    return true;
}

/**
 * This is the map from the actions (for similarity purposes) at
 * https://extranet.vettalabs.com:8443/bin/view/Petaverse/PetPredicateSimilarityMatrix
 * (no idea what the equivalent link on the opencog wiki should be...)
 * to the action schemata defined at
 * http://wiki.opencog.org/w/SchemaVocabulary_%28Embodiment%29
 *
 * 
 *  goto_obj(object)
 *  grab(object)        => Status grab(EntityID id, float RangeRadius)
 *  step_forward
 *  step_backward
 *  rotate_left
 *  rotate_right
 *  jump                => void jump(float theta, Vector velocity)
 *  drop                => void drop()
 *  grab                => Status grab(EntityID id, float RangeRadius)
 *  sniff               => void sniff([EntityID id, short bodyPart])
 *  bark                => void bark([EntityID id])
 *  look_up_turn_head   => void look_up_turn_head()
 *  bare_teeth          => void bareTeeth([EntityID id])
 *  back_flip
 *  wag                 => void wag([float duration])
 *  stretch             => void stretch()
 *  sit                 => void sit([short duration])
 *  beg                 => void beg()
 *  heel                => void heel()
 *  random_step
 */
combo::vertex PetActionSchema::execute(const std::vector<combo::vertex>& arguments) const throw (RuntimeException, InvalidParamException, std::bad_exception)
{
    PetAction action(actionType);

    const ActionType::ParamTypes& mandatoryParamTypes = actionType.getMandatoryParamTypes();
    if (arguments.size() < minArity) {
        throw InvalidParamException(TRACE_INFO,
             "PetActionSchema - Schema %s got insuficient no. "
             "of parameters: %u (expected at least %u).",
             actionType.getName().c_str(), arguments.size(), minArity);
    }

    // TODO: Optional parameters are not being added, since procedure arity is constant 
    // (so, only for mandatory parameters)
    //       Review this when/if procedures may have variable number of parameters...
    //const ActionType::ParamTypes& optionalParamTypes = actionType.getOptionalParamTypes();
    unsigned int maximalArity = minArity + optionalArity;
    if (arguments.size() > maximalArity) {
        throw InvalidParamException(TRACE_INFO,
             "PetActionSchema - Schema %s exceeded no. of parameters: %u (expected at most %u).",
             actionType.getName().c_str(), arguments.size(), maximalArity);
    }

    unsigned int argIndex = 0;
    // std::vector<combo::vertex>::const_iterator argItr = arguments.begin();

    foreach(const ActionParamType& paramType, mandatoryParamTypes) {
        bool validArg = false;

        switch (paramType.getCode()) {
        case BOOLEAN_CODE: {
            // The value of the builtin must be checked because there are many things
            // in addition to the logical values in the definition of builtin type
            const combo::id::builtin* arg = boost::get<combo::id::builtin>(&(arguments[argIndex]));
            if (arg && (*arg == combo::id::logical_true || *arg == combo::id::logical_false)) {
                bool value = (*arg == combo::id::logical_true);
                ActionParameter actionParam(actionType.getParamNames()[argIndex], paramType, toString(value));
                action.addParameter(actionParam);
                validArg = true;
            }
            break;
        }
        case INT_CODE: {
            // TODO: Check this: it seems all numbers are double in combo (typedef double contin_t;)
            const combo::contin_t* arg = boost::get<combo::contin_t>(&(arguments[argIndex]));
            if (arg) {
                ActionParameter actionParam(actionType.getParamNames()[argIndex], paramType, toString((int)*arg));
                action.addParameter(actionParam);
                validArg = true;
            }
        }
        case FLOAT_CODE: {
            // TODO: Check this: it seems all numbers are double in combo (typedef double contin_t;)
            const combo::contin_t* arg = boost::get<combo::contin_t>(&(arguments[argIndex]));
            if (arg) {
                ActionParameter actionParam(actionType.getParamNames()[argIndex], paramType, toString(*arg));
                action.addParameter(actionParam);
                validArg = true;
            }
            break;
        }
        case STRING_CODE: {
            const combo::definite_object* arg = boost::get<combo::definite_object>(&(arguments[argIndex]));
            if (arg) {
                ActionParameter actionParam(actionType.getParamNames()[argIndex], paramType, *arg);
                action.addParameter(actionParam);
                validArg = true;
            } else {
                // bodyParts are defined as combo::id::action_symbol in combo, which is an enum
                // So, if any body part code is given, convert it to string
                const combo::action_symbol* arg = boost::get<combo::action_symbol>(&(arguments[argIndex]));
                if (arg) {
                    std::string bodyPart = lexical_cast<std::string>(*arg);
                    ActionParameter actionParam(actionType.getParamNames()[argIndex], paramType, bodyPart);
                    action.addParameter(actionParam);
                    validArg = true;
                }
            }
            break;
        }
        case VECTOR_CODE: {
            // TODO:  Don't know how combo::vertex value would represent this. Using tree contin_t values in sequence, perhaps...
            break;
        }
        case ROTATION_CODE: {
            // TODO:  Don't know how combo::vertex value would represent this. Using tree contin_t values in sequence, perhaps...
            break;
        }
        case ENTITY_CODE: {
            const combo::definite_object* arg = boost::get<combo::definite_object>(&(arguments[argIndex]));
            if (arg) {
                std::string objectId = *arg;
                // Builds the Entity param
                AtomSpace& atomSpace = pai.getAtomSpace();
                HandleSeq hs;
                atomSpace.getHandleSet(back_inserter(hs), OBJECT_NODE, objectId, true);
                if (!hs.empty()) {
                    if (hs.size() > 1) {
                        std::string atomTypes;
                        foreach(Handle h, hs) {
                            atomTypes += classserver().getTypeName(atomSpace.getType(h));
                            atomTypes += " ";
                        }
                        logger().warn("WARNING: Got multiple ObjectNode with a same name: %s (atom types: %s)\n", objectId.c_str(), atomTypes.c_str());
                    }
                    Type atomType = atomSpace.getType(hs[0]);
                    // TODO: What about the other types of SL object? (accessory and structure)
                    Entity entity(objectId, (atomType == AVATAR_NODE) ? AVATAR_OBJECT_TYPE : (atomType == PET_NODE) ? PET_OBJECT_TYPE : (atomType == HUMANOID_NODE) ? HUMANOID_OBJECT_TYPE : UNKNOWN_OBJECT_TYPE);
                    ActionParameter actionParam(actionType.getParamNames()[argIndex], paramType, entity);
                    action.addParameter(actionParam);
                    validArg = true;
                }
            }
            break;
        }
        default:
            break;
        }

        if (!validArg) {
            throw InvalidParamException(TRACE_INFO,
                "PetActionSchema - Schema %s got invalid argument at %u: %s (expected an %s).",
                actionType.getName().c_str(), argIndex,
                toString(arguments[0]).c_str(), paramType.getName().c_str());
        }

    }
    ActionPlanID planId = pai.createActionPlan();
    pai.addAction(planId, action);

    // This function can throw a RuntimeException
	if(!config().get_bool("EXTRACTED_ACTION_MODE")) {
		pai.sendActionPlan(planId);
	} else {
		pai.sendExtractedActionFromPlan(planId);
	}

    return planId;
}

}} // ~namespace opencog::Procedure
