/*
 * Strips.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * Written by Shujing KE
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

#include "Strips.h"
#include <opencog/util/oc_assert.h>
#include <opencog/util/StringManipulator.h>
#include <math.h>


using namespace opencog::oac;

const char* opencog::oac::EFFECT_OPERATOR_NAME[9] =
{
    "OP_REVERSE", // this is only for the bool variables
    "OP_ASSIGN",  // this operator can be used in any variable type =
    "OP_ASSIGN_NOT_EQUAL_TO", // this operator can be used in any variable type !=
    "OP_ASSIGN_GREATER_THAN", // only for numeric variables >
    "OP_ASSIGN_LESS_THAN", // only for numeric variables <
    "OP_ADD",     // only for numeric variables +=
    "OP_SUB",     // only for numeric variables -=
    "OP_MUL",     // only for numeric variables *=
    "OP_DIV"      // only for numeric variables /=
};

const char* opencog::oac::STATE_TYPE_NAME[5] =
{
    "=",     // EvaluationLink
    "!=",     // EvaluationLink
    "WITHIN", // EvaluationLink + PredicationNode "Fuzzy_within"
    ">", // GreaterThanLink
    "<"     // LessThanLink
};

State::State(string _stateName, ActionParamType _valuetype,StateType _stateType, ParamValue  _ParamValue,
             vector<ParamValue> _stateOwnerList, bool _need_inquery, InqueryStateFun _inqueryStateFun, bool _permanent)
    : stateOwnerList(_stateOwnerList),stateType(_stateType), need_inquery(_need_inquery),inqueryStateFun(_inqueryStateFun), permanent(_permanent)
{
    this->stateName = _stateName;
    stateVariable = new ActionParameter(_stateName,_valuetype,_ParamValue);
}

State::State(string _stateName, ActionParamType _valuetype ,StateType _stateType, ParamValue _ParamValue,
             bool _need_inquery, InqueryStateFun _inqueryStateFun, bool _permanent)
    : stateType(_stateType), need_inquery(_need_inquery),inqueryStateFun(_inqueryStateFun),permanent(_permanent)
{
    stateVariable = new ActionParameter(_stateName,_valuetype,_ParamValue);
    stateOwnerList.clear();
    this->stateName = _stateName;
}

State::~State()
{
    if (stateVariable)
        delete stateVariable;

    stateVariable = 0;
}

State* State::clone()
{
    State* cloneState = new State(this->stateName,this->getActionParamType(),this->stateType, this->stateVariable->getValue(),
                                  this->stateOwnerList,this->need_inquery, this->inqueryStateFun, this->permanent );
    return cloneState;
}

void State::assignValue(const ParamValue& newValue)
{
    stateVariable->assignValue(newValue);
}

ParamValue State::getParamValue()
{
    // if it's not an ungrouned variable , just return its value
    if (! Rule::isParameterUnGrounded(*(this->stateVariable)))
        return this->stateVariable->getValue();

    if (need_inquery)
    {
        if (inqueryStateFun != 0)
            return inqueryStateFun(stateOwnerList);
        else
            return UNDEFINED_VALUE;
    }
    else
    {
        bool is_true;
        ParamValue value = Inquery::getParamValueFromAtomspace(*this,  is_true);
        if ( (! is_true) &&  (stateType !=  STATE_NOT_EQUAL_TO))
            return UNDEFINED_VALUE;
        else if ((stateType ==  STATE_NOT_EQUAL_TO) && ( is_true))
            return UNDEFINED_VALUE;

        return value;
    }

}

// pls make sure the goal describes the same state content with this state first
bool State::isSatisfied( State &goal, float& satisfiedDegree, bool &unknown, State *original_state)
{
    unknown = false;

    if ((goal.stateType == stateType)&&(stateVariable->getValue() == goal.stateVariable->getValue()))
    {
       satisfiedDegree = 1.0f;
       return true;
    }


    // For non-numberic ParamValues,  the satisfiedDegree is always 0.0 or 1.0
    if (! ActionParamType::isNumbericValueType( goal.getActionParamType().getCode()))
    {
       if (stateType == STATE_EQUAL_TO)
       {
           if ( goal.stateType == STATE_EQUAL_TO)
           {
               satisfiedDegree = 0.0f;
               return false;
           }
           else if (goal.stateType == STATE_NOT_EQUAL_TO)
           {
               if (!(stateVariable->getValue() == goal.stateVariable->getValue()))
               {
                   satisfiedDegree = 1.0f;
                   return true;
               }
               else
               {
                   satisfiedDegree = 0.0f;
                   return false;
               }
           }
       }
       else if (stateType == STATE_NOT_EQUAL_TO)
       {
           if (goal.getActionParamType().getCode() == BOOLEAN_CODE)
           {
               if (!(stateVariable->getValue() == goal.stateVariable->getValue()))
               {
                   satisfiedDegree = 1.0f;
                   return true;
               }
               else
               {
                   satisfiedDegree = 0.0f;
                   return false;
               }
           }
           else
           {
               if (goal.stateType == STATE_NOT_EQUAL_TO)
               {
                   satisfiedDegree = 1.0f;
                   unknown = true;
                   return false;
               }
               else if ( (goal.stateType == STATE_EQUAL_TO))
               {
                   if (!(stateVariable->getValue() == goal.stateVariable->getValue()))
                   {
                       satisfiedDegree = 0.0f;
                       unknown = true;
                       return false;
                   }
                   else
                   {
                       satisfiedDegree = 0.0f;
                       return false;
                   }
               }
           }
       }
       else
       {
           satisfiedDegree = 0.0f;
           return false;
       }

    }

    // Deal with numberic ParamValues:

    int intVal, intGoalVal, intOriVal;
    float floatVal, floatGoalVal, floatOriVal;
    opencog::pai::FuzzyIntervalInt fuzzyInt, fuzzyGoalInt, fuzzyOriInt;
    opencog::pai::FuzzyIntervalFloat fuzzyFloat, fuzzyGoalFloat, fuzzyOriFloat;


    getNumbericValues(intVal,floatVal,fuzzyInt,fuzzyFloat);
    goal.getNumbericValues(intGoalVal,floatGoalVal,fuzzyGoalInt,fuzzyGoalFloat);
    if (original_state)
        original_state->getNumbericValues(intOriVal,floatOriVal,fuzzyOriInt,fuzzyOriFloat);


    // we allow the goal, current and origin states have different stateType (EQUAL_TO,GREAT_THAN,FUZZY_WITHIN and so on),
    // and also allow they have different value types (Int, float, fuzzy values and so on)
    // So their will be a big number of combinations, currently only finish some common combinations.
    // TODO: finish all the combinations

    float ori,cur=0.0f;

    switch (goal.stateType)
    {
    case STATE_EQUAL_TO:

        if (original_state != 0)
        {
            if (stateType ==  STATE_EQUAL_TO)
            {
                if (original_state->stateType == STATE_EQUAL_TO)
                    satisfiedDegree = State::calculateNumbericsatisfiedDegree(floatGoalVal,floatVal,floatOriVal);
                else
                    satisfiedDegree = 0.0f; //TODO
            }
            else
                satisfiedDegree = 0.0f; //TODO
        }

        return false;

    case STATE_NOT_EQUAL_TO:

        if (stateType  == STATE_EQUAL_TO)
        {
            if (!(stateVariable == goal.stateVariable))
            {
                satisfiedDegree = 1.0f; //TODO
                return true;
            }
            else
            {
                satisfiedDegree = 0.0f; //TODO
                return false;
            }
        }

        if (stateType == STATE_FUZZY_WITHIN)
        {
            if (getActionParamType().getCode()  == FUZZY_INTERVAL_INT_CODE)
            {
               if (! (fuzzyInt.isInsideMe(intGoalVal)))
               {
                   satisfiedDegree = 1.0f; //TODO
                   return true;
               }
               else
               {
                   satisfiedDegree = 0.0f; //TODO
                   return false;
               }
            }
            else if (getActionParamType().getCode()  == FUZZY_INTERVAL_FLOAT_CODE)
            {
                if (! (fuzzyFloat.isInsideMe(floatGoalVal)))
                {
                    satisfiedDegree = 1.0f; //TODO
                    return true;
                }
                else
                {
                    satisfiedDegree = 0.0f; //TODO
                    return false;
                }
            }
        }

        if (stateType  == STATE_GREATER_THAN)
        {
             if (floatVal > floatGoalVal)
             {
                 satisfiedDegree = 1.0f; //TODO
                 return true;
             }
             else
             {
                 satisfiedDegree = 0.0f; //TODO
                 return false;
             }
        }

        if (stateType  == STATE_LESS_THAN)
        {
             if (floatVal < floatGoalVal)
             {
                 satisfiedDegree = 1.0f; //TODO
                 return true;
             }
             else
             {
                 satisfiedDegree = 0.0f; //TODO
                 return false;
             }
        }

        satisfiedDegree = 0.0f; //TODO
        return false;

    case STATE_GREATER_THAN:

        if (stateType  == STATE_FUZZY_WITHIN)
            cur = fuzzyFloat.bound_low ;
        else if ((stateType  == STATE_EQUAL_TO) || (stateType == STATE_GREATER_THAN) || (stateType  == STATE_LESS_THAN))
            cur = floatVal;

        if (original_state)
        {
            if (cur > floatGoalVal)
                satisfiedDegree = 1.0f;
            else
            {
                switch (original_state->stateType)
                {
                case STATE_FUZZY_WITHIN:
                    ori = fuzzyOriFloat.bound_low ;
                    satisfiedDegree = State::calculateNumbericsatisfiedDegree(floatGoalVal,cur,ori);
                    break;
                case STATE_EQUAL_TO:
                case STATE_GREATER_THAN:
                    ori = floatOriVal;
                    satisfiedDegree = State::calculateNumbericsatisfiedDegree(floatGoalVal,cur,ori);
                    break;
                default:
                    satisfiedDegree = 0.0f; //TODO

                }
            }

        }

        return (cur > floatGoalVal);


    case STATE_LESS_THAN:

        if (stateType  == STATE_FUZZY_WITHIN)
            cur = fuzzyFloat.bound_high ;
        else if ((stateType  == STATE_EQUAL_TO) || (stateType == STATE_GREATER_THAN)||(stateType  == STATE_LESS_THAN) )
            cur = floatVal;

        if (original_state)
        {
            if (cur < floatGoalVal)
                satisfiedDegree = 1.0f;
            else
            {
                switch (original_state->stateType)
                {
                case STATE_FUZZY_WITHIN:
                    ori = fuzzyOriFloat.bound_high ;
                    satisfiedDegree = State::calculateNumbericsatisfiedDegree(floatGoalVal,cur,ori);
                    break;
                case STATE_EQUAL_TO:
                case STATE_GREATER_THAN:
                    ori = floatOriVal;
                    satisfiedDegree = State::calculateNumbericsatisfiedDegree(floatGoalVal,cur,ori);
                    break;
                default:
                    satisfiedDegree = 0.0f; //TODO

                }
            }

        }

        return (cur < floatGoalVal);

    case STATE_FUZZY_WITHIN:
        if (stateType  == STATE_EQUAL_TO)
        {
            if ((fuzzyGoalFloat.isInsideMe(floatVal)))
            {
                satisfiedDegree = 1.0f;
                return true;
            }
            else
            {
                if (original_state)
                {
                    if (original_state->stateType == STATE_EQUAL_TO)
                        satisfiedDegree = State::calculateNumbericsatisfiedDegree(fuzzyGoalFloat,floatVal,floatOriVal);
                    else if (original_state->stateType == STATE_FUZZY_WITHIN) //TODO:
                        satisfiedDegree = State::calculateNumbericsatisfiedDegree(floatGoalVal,floatVal,(fuzzyOriFloat.bound_high + fuzzyOriFloat.bound_low)/2.0f);
                    else
                        satisfiedDegree = 0.0f; // TODO
                }
                return false;
            }
        }

        else if (stateType  == STATE_FUZZY_WITHIN)
        {
            if (fuzzyGoalFloat.isInsideMe(fuzzyFloat))
            {
                satisfiedDegree = 1.0f;
                return true;
            }
            else
            {
                if (original_state)
                {
                    if (original_state->stateType == STATE_EQUAL_TO)  //TODO:
                        satisfiedDegree = State::calculateNumbericsatisfiedDegree(fuzzyGoalFloat,(fuzzyFloat.bound_high + fuzzyFloat.bound_low)/2.0f,floatOriVal);
                    else if (original_state->stateType == STATE_FUZZY_WITHIN)
                        satisfiedDegree = State::calculateNumbericsatisfiedDegree(fuzzyGoalFloat,fuzzyFloat,fuzzyOriFloat);
                    else
                        satisfiedDegree = 0.0f; // TODO
                }
                return false;

            }
        }
        else
        {
            satisfiedDegree = 0.0f; // TODO
            return false;
        }

    default:
        return false;

    }

}

bool State::getNumbericValues(int& intVal, float& floatVal,opencog::pai::FuzzyIntervalInt& fuzzyInt, opencog::pai::FuzzyIntervalFloat& fuzzyFloat)
{
    if (getActionParamType().getCode() == FUZZY_INTERVAL_INT_CODE)
    {
       fuzzyInt = boost::get<opencog::pai::FuzzyIntervalInt>(stateVariable->getValue());
       fuzzyFloat = FuzzyIntervalFloat((float)fuzzyInt.bound_low, (float)fuzzyInt.bound_high);
    }
    else if (getActionParamType().getCode() == FUZZY_INTERVAL_FLOAT_CODE)
        fuzzyFloat = boost::get<opencog::pai::FuzzyIntervalFloat>(stateVariable->getValue());
    else if (getActionParamType().getCode()  == INT_CODE)
    {
        string str = boost::get<string>(stateVariable->getValue());
        intVal = atoi(str.c_str());
        floatVal = (float)intVal;
    }
    else if (getActionParamType().getCode()  == FLOAT_CODE)
    {
        string str = boost::get<string>(stateVariable->getValue());
        floatVal = (float)atof(str.c_str());
    }
    else
        return false;

    return true;
}

float State::getFloatValueFromNumbericState()
{
    // please make sure this a numberic state before call this function
    if (getActionParamType().getCode() == FUZZY_INTERVAL_INT_CODE)
    {
       FuzzyIntervalInt fuzzyInt = boost::get<opencog::pai::FuzzyIntervalInt>(stateVariable->getValue());
       FuzzyIntervalFloat fuzzyFloat = FuzzyIntervalFloat((float)fuzzyInt.bound_low, (float)fuzzyInt.bound_high);
       return (fuzzyFloat.bound_low + fuzzyInt.bound_high)/2.0f;
    }
    else if (getActionParamType().getCode() == FUZZY_INTERVAL_FLOAT_CODE)
    {
        FuzzyIntervalFloat fuzzyFloat = boost::get<opencog::pai::FuzzyIntervalFloat>(stateVariable->getValue());
        return (fuzzyFloat.bound_low + fuzzyFloat.bound_high)/2.0f;
    }
    else if (getActionParamType().getCode()  == INT_CODE)
    {
        string str = boost::get<string>(stateVariable->getValue());
        int intVal = atoi(str.c_str());
        return (float)intVal;

    }
    else if (getActionParamType().getCode()  == FLOAT_CODE)
    {
        string str = boost::get<string>(stateVariable->getValue());
        float floatVal = (float)atof(str.c_str());
        return floatVal;
    }


    return 0.0f;
}

bool State::isNumbericState() const
{
    if (  (stateVariable->getType().getCode() == INT_CODE) ||
          (stateVariable->getType().getCode() == FLOAT_CODE) ||
          (stateVariable->getType().getCode() == FUZZY_INTERVAL_INT_CODE) ||
          (stateVariable->getType().getCode() == FUZZY_INTERVAL_FLOAT_CODE) )
        return true;
    else
        return false;

}

bool State::isStateOwnerTypeTheSameWithMe(const State& other) const
{
    if (stateOwnerList.size() != other.stateOwnerList.size())
        return false;

    vector<ParamValue>::const_iterator it1 = stateOwnerList.begin();
    vector<ParamValue>::const_iterator it2 = other.stateOwnerList.begin();
    for ( ; it1 != stateOwnerList.end(); ++ it1, ++ it2)
    {
        if ( ! ActionParameter::areFromSameType(*it1, *it2))
            return false;
    }

    return true;
}

float State::calculateNumbericsatisfiedDegree(float goal, float current, float origin)
{
    float disCurToGoal = fabs(goal - current);
    float disOriToGoal = fabs(goal - origin);
    return (disOriToGoal - disCurToGoal)/disOriToGoal;
}

float State::calculateNumbericsatisfiedDegree(const FuzzyIntervalFloat& goal, float current, float origin)
{
    float disCurToGoal=0.0f,disOriToGoal=0.0f;

    // Make sure that both current and origin value are not inside the goal boundaries
    OC_ASSERT((!goal.isInsideMe(current)) && (!goal.isInsideMe(origin)),
              "State::calculateNumbericsatisfiedDegree(FuzzyIntervalFloat goal, float current, float origin): the current state value %f or the original value %f has already satisfied the goal [%f,%f]\n",
              current,origin,goal.bound_low,goal.bound_high);

    if (current < goal.bound_low)
        disCurToGoal = goal.bound_low - current;
    else if (current > goal.bound_high)
        disCurToGoal = current - goal.bound_high;

    if (origin< goal.bound_low)
        disCurToGoal = goal.bound_low - origin;
    else if (origin > goal.bound_high)
        disCurToGoal = origin - goal.bound_high;

    return (disOriToGoal - disCurToGoal)/disOriToGoal;
}

float State::calculateNumbericsatisfiedDegree(const FuzzyIntervalFloat& goal, const FuzzyIntervalFloat& current, const FuzzyIntervalFloat& origin)
{
    float disCurToGoal,disOriToGoal;

    // Make sure that both current and origin value are not inside the goal boundaries
    OC_ASSERT((!goal.isInsideMe(current)) && (!goal.isInsideMe(origin)),
              "State::calculateNumbericsatisfiedDegree(FuzzyIntervalFloat goal, FuzzyIntervalFloat current, FuzzyIntervalFloat origin): the current state value [%f,%f] or the original value [%f,%f] has already satisfied the goal [%f,%f]\n",
              current.bound_low,current.bound_high,origin.bound_low,origin.bound_high,goal.bound_low,goal.bound_high);

    disCurToGoal = distanceBetween2FuzzyFloat(goal,current);
    disOriToGoal = distanceBetween2FuzzyFloat(goal,origin);

    return (disOriToGoal - disCurToGoal)/disOriToGoal;
}

float State::distanceBetween2FuzzyFloat(const FuzzyIntervalFloat& goal, const FuzzyIntervalFloat& other)
{
    if (other.bound_high < goal.bound_low)
        return (goal.bound_low - other.bound_high) + (goal.bound_low - other.bound_low);
    else if (other.bound_low > goal.bound_high)
        return (other.bound_high - goal.bound_high) + (other.bound_low - goal.bound_high);
    else if ( (other.bound_low < goal.bound_low) && (other.bound_high > goal.bound_high))
        return (other.bound_high - goal.bound_high) + (goal.bound_low - other.bound_low);
    else if (( other.bound_low < goal.bound_low) && (goal.isInsideMe( other.bound_high)))
        return goal.bound_low - other.bound_low;
    else if ((other.bound_high > goal.bound_high) && (goal.isInsideMe(other.bound_low)))
        return other.bound_high - goal.bound_high;
    else
        return 0.0f;

}

Effect::Effect(State* _state, EFFECT_OPERATOR_TYPE _op, ParamValue _OPValue, bool _ifCheckStateOwnerType)
{
//    OC_ASSERT(_AssertValueType(*_state,_op,_OPValue),
//              "Effect constructor: got invalid effect value type: s% for state value type: %s, operator: %s, in state: %s\n",
//              _OPValue.type().name(), _state->getActionParamType().getName().c_str(),EFFECT_OPERATOR_NAME[_op],_state->name().c_str());

    state = _state;
    effectOp = _op;
    opParamValue = _OPValue;
    ifCheckStateOwnerType = _ifCheckStateOwnerType;
}

bool Effect::isEffectOpOpposite(Effect* effect)
{
    if ( ((effect->effectOp == OP_ASSIGN) && (effect->state->stateType == STATE_NOT_EQUAL_TO)) ||
         ((effect->effectOp == OP_ASSIGN_NOT_EQUAL_TO) && (effect->state->stateType == STATE_EQUAL_TO)) ||
         ((effect->effectOp == OP_ASSIGN_GREATER_THAN) && (effect->state->stateType == STATE_LESS_THAN)) ||
         ((effect->effectOp == OP_ASSIGN_LESS_THAN) && (effect->state->stateType == STATE_GREATER_THAN)) )
        return true;
    else
        return false;
}


StateType Effect::getTargetStateType()
{
    if (effectOp == OP_ASSIGN)
        return STATE_EQUAL_TO;

    if (effectOp == OP_ASSIGN_NOT_EQUAL_TO)
        return STATE_NOT_EQUAL_TO;

    if (effectOp == OP_ASSIGN_GREATER_THAN)
        return STATE_GREATER_THAN;

    if (effectOp == OP_ASSIGN_LESS_THAN)
        return STATE_LESS_THAN;

   /* if ( (effectOp == OP_REVERSE)||
         (effectOp == OP_ADD)||
         (effectOp == OP_SUB)||
         (effectOp == OP_MUL)||
         (effectOp == OP_DIV))*/
    return state->stateType;

}

bool Effect::executeEffectOp(State* state, Effect* effect, ParamGroundedMapInARule &groundings)
{
    ParamValue opParamValue;

    if (effect->effectOp != OP_REVERSE) // OP_REVERSE doesn't need an opParamValue
    {
        if (Rule::isParamValueUnGrounded(effect->opParamValue))
        {
            // look up this value in groundings
            string varName = ActionParameter::ParamValueToString(effect->opParamValue);
            ParamGroundedMapInARule::iterator paramMapIt = groundings.find(varName);
            if (paramMapIt == groundings.end())
            {
                // if the effect operator is to make the operator opposite, e.g. change from STATE_EQUAL_TO to STATE_NOT_EQUAL_TO
                // then just need to change the state type, don't need to change the value
                if (Effect::isEffectOpOpposite(effect))
                {
                    state->changeStateType(effect->getTargetStateType());
                    return true;
                }

                return false;
            }
            else
                opParamValue = paramMapIt->second;
        }
        else
        {
            opParamValue = effect->opParamValue;
        }
    }

 /*   if (effect->effectOp != OP_ASSIGN_NOT_EQUAL_TO)
    {
        if (state->stateType != STATE_EQUAL_TO)
            state->changeStateType(STATE_EQUAL_TO);
    }
    else
    {
        if (state->stateType != STATE_NOT_EQUAL_TO)
            state->changeStateType(STATE_NOT_EQUAL_TO);
    }
    */

    if (effect->effectOp == OP_ASSIGN)
    {
        if (state->stateType != STATE_EQUAL_TO)
            state->changeStateType(STATE_EQUAL_TO);

        state->assignValue(opParamValue);

    }
    else if (effect->effectOp == OP_ASSIGN_NOT_EQUAL_TO)
    {
        if (state->stateType != STATE_NOT_EQUAL_TO)
            state->changeStateType(STATE_NOT_EQUAL_TO);

        state->assignValue(opParamValue);

    }
    else if (effect->effectOp == OP_REVERSE)
    {
        string oldStr = boost::get<string>(state->stateVariable->getValue());
        if (oldStr == "true")
            state->assignValue(ParamValue(string("false")));
        else if (oldStr == "false")
            state->assignValue(ParamValue(string("true")));
        else
            return false;
    }
    else if (effect->effectOp == OP_ASSIGN_GREATER_THAN)
    {
        state->assignValue(opParamValue);

        if (state->stateType != STATE_GREATER_THAN)
            state->changeStateType(STATE_GREATER_THAN);
    }
    else if(effect->effectOp == OP_ASSIGN_LESS_THAN)
    {
        state->assignValue(opParamValue);

        if (state->stateType != STATE_LESS_THAN)
            state->changeStateType(STATE_LESS_THAN);
    }
    else
    {
        string oldStr = boost::get<string>(state->stateVariable->getValue());
        string opvStr = boost::get<string>(opParamValue);

        double oldv = atof(oldStr.c_str());
        double opv = atof(opvStr.c_str());
        double newV;
        switch (effect->effectOp)
        {
        case OP_ADD:
            newV = oldv + opv;
            break;
        case OP_SUB:
            newV = oldv - opv;
            break;
        case OP_MUL:
            newV = oldv * opv;
            break;
        case OP_DIV:
            newV = oldv / opv;
            break;
        default:
            return false;
        }

        ParamValue sv;
        if (state->getActionParamType().getCode() == INT_CODE)
        {
            sv = toString<int>((int)newV);
            state->assignValue( sv );
        }
        else if (state->getActionParamType().getCode() == FLOAT_CODE)
        {
            sv = toString<float>((float)newV);
            state->assignValue( sv );
        }
        else
            return false;
    }


    return true;

}

// variant<bool, int, float, double, string, Rotation, Vector > ParamValue
bool Effect::_AssertValueType(State& _state, EFFECT_OPERATOR_TYPE _effectOp, ParamValue &_OPValue)
{
    // first,make sure the value type of the operater value is the same with the value type of the state

//    if (! ActionParameter::areFromSameType(_state.stateVariable->getValue(),_OPValue))
//        return false;

    // if the value type is string, Entity, Vector, Rotation or fuzzy values , it's only allow to use the OP_ASSIGN operator
    if (_state.getActionParamType().getCode() == STRING_CODE || _state.getActionParamType().getCode() == ROTATION_CODE ||
        _state.getActionParamType().getCode() == VECTOR_CODE || _state.getActionParamType().getCode() == ENTITY_CODE ||
        _state.getActionParamType().getCode() == FUZZY_INTERVAL_INT_CODE ||  _state.getActionParamType().getCode() == FUZZY_INTERVAL_FLOAT_CODE   )
    {
        return (_effectOp == OP_ASSIGN);
    }

    // and also this value type should be able to be the parameter for this operator
    switch(_effectOp)
    {
    case OP_REVERSE:
        return (_state.getActionParamType().getCode() == BOOLEAN_CODE);
    case OP_ASSIGN: // all value types can use OP_ASSIGN and OP_ASSIGN_NOT_EQUAL_TO
    case OP_ASSIGN_NOT_EQUAL_TO:
        return true;

    case OP_ADD:// only for the numeric value types
    case OP_SUB:// only for the numeric value types
    case OP_MUL:// only for the numeric value types
    case OP_DIV:// only for the numeric value types
    case OP_ASSIGN_GREATER_THAN:// only for the numeric value types
    case OP_ASSIGN_LESS_THAN:// only for the numeric value types
    {
       string* v =  boost::get<string>(&_OPValue);
       if ( v == 0)
           return false;

       if ( (_state.getActionParamType().getCode() != INT_CODE) && (_state.getActionParamType().getCode() != FLOAT_CODE))
           return false;

    }

    default:
        return false;

    }
}

float Rule::getBasicCost()
{
    return basic_cost;
}


float Rule::getCost(float basic_cost,vector<CostHeuristic>& CostHeuristics, ParamGroundedMapInARule& groudings)
{
    // the cost calculation is : basic_cost + cost_cal_state.value1 * cost_coefficient1 + cost_cal_state.value2 * cost_coefficient2 + ...
    // the cost_cal_state is the state related to the cost, e.g.: if an action is move from A to B, then the cost will depend on the state distanceOf(A,B)
    if (CostHeuristics.size() == 0)
        return basic_cost;
    else
    {
        float totalcost = basic_cost;

        vector<CostHeuristic>::iterator costIt;
        for (costIt = CostHeuristics.begin(); costIt != CostHeuristics.end(); ++ costIt)
        {
            State* cost_cal_state = ((CostHeuristic)(*costIt)).cost_cal_state;
            // get numberic value from this cost_cal_state
            if (! cost_cal_state->isNumbericState())
            {
                logger().error("Planner::Rule::getCost : The relatied state is not numberic state: " + cost_cal_state->name() );
                return -1.0f;
            }

            State* groundedState = groundAStateByRuleParamMap(cost_cal_state, groudings);
            if (groundedState == 0)
            {
                logger().error("Planner::Rule::getCost : This state cannot be grounded: " + cost_cal_state->name() );
                return -1.0f;
            }
            totalcost += groundedState->getFloatValueFromNumbericState() * ((CostHeuristic)(*costIt)).cost_coefficient;
        }

        return totalcost;
    }
}

void Rule::preProcessRule()
{
    _preProcessRuleParameterIndexes();
    IsRecursiveRule = _isRecursiveRule();
}

bool Rule::_isRecursiveRule()
{
    // if the all the preconditions and effects are of the same state, then it's a recursive rule
    // e.g. if can move from A to B & can move from B to C, then can move from A to C , is a recursive rule

    vector<EffectPair>::iterator iteffect;
    for (iteffect = effectList.begin(); iteffect != effectList.end(); ++ iteffect)
    {
        Effect* e = (Effect*)(((EffectPair)(*iteffect)).second);

        vector<State*>::iterator itpre;
        for (itpre = preconditionList.begin(); itpre != preconditionList.end(); ++ itpre)
        {
            State* ps = *itpre;
            if (ps->name() != e->state->name())
                return false;
        }
    }

    return true;
}

//bool Rule::isUnGroundedString( string& s)
//{
//    for (int i = 0; i < PARAMETER_NUM; ++i)
//    {
//        if ( (s == bool_var[i]) ||
//             (s == str_var[i]) ||
//             (s == int_var[i]) ||
//             (s == float_var[i]) )
//            return true;
//    }

//    return false;
//}

//bool Rule::isUnGroundedVector( Vector& v)
//{
//    for (int i = 0; i < PARAMETER_NUM; ++i)
//    {
//        if (v == vector_var[i])
//            return true;
//    }

//    return false;
//}

//bool Rule::isUnGroundedEntity( Entity& e)
//{
//    for (int i = 0; i < PARAMETER_NUM; ++i)
//    {
//        if (e == entity_var[i])
//            return true;
//    }

//    return false;
//}


bool Rule::isUnGroundedString( string& s)
{
    for (int i = 0; i < PARAMETER_NUM; ++i)
    {
        if ( (s == bool_var[i]) ||
             (s == str_var[i]) ||
             (s == int_var[i]) ||
             (s == float_var[i]) )
            return true;
    }

    return false;
}

bool Rule::isUnGroundedVector( Vector &v)
{
    for (int i = 0; i < PARAMETER_NUM; ++i)
    {
        if (v == vector_var[i])
            return true;
    }

    return false;
}

bool Rule::isUnGroundedEntity(Entity& e)
{
    for (int i = 0; i < PARAMETER_NUM; ++i)
    {
        if (e == entity_var[i])
            return true;
    }

    return false;
}

bool Rule::isParameterUnGrounded( ActionParameter& param)
{
    switch(param.getType().getCode())
    {
    case ENTITY_CODE:
        return isUnGroundedEntity(boost::get<Entity>(param.getValue()));

    case VECTOR_CODE:
        return isUnGroundedVector(boost::get<Vector>(param.getValue()));

    case STRING_CODE:
    case INT_CODE:
    case FLOAT_CODE:
    case BOOLEAN_CODE:  
        return isUnGroundedString(boost::get<string>(param.getValue()));

    default:
        return false;
    }
}

//bool Rule::isParameterUnGrounded( ActionParameter& param)
//{
//    switch(param.getType().getCode())
//    {
//    case ENTITY_CODE:
//        return isUnGroundedEntity(param.getValue());

//    case VECTOR_CODE:
//        return isUnGroundedVector(param.getValue());

//    case STRING_CODE:
//    case INT_CODE:
//    case FLOAT_CODE:
//    case BOOLEAN_CODE:
//        return isUnGroundedString(param.getValue());
//    default:
//        return false;
//    }
//}

bool Rule::isParamValueUnGrounded(ParamValue& paramVal)
{
    if(boost::get<Entity>(&paramVal))
        return isUnGroundedEntity(boost::get<Entity>(paramVal));

    if(boost::get<Vector>(&paramVal))
        return isUnGroundedVector(boost::get<Vector>(paramVal));

    if(boost::get<string>(&paramVal))
        return isUnGroundedString(boost::get<string>(paramVal));

    return false;

}

//bool Rule::isParamValueUnGrounded(ParamValue& paramVal)
//{
//    if(boost::get<Entity>(&paramVal))
//        return isUnGroundedEntity(paramVal);

//    if(boost::get<Vector>(&paramVal))
//        return isUnGroundedVector(paramVal);

//    if(boost::get<string>(&paramVal))
//        return isUnGroundedString(paramVal);

//    return false;

//}

// in some planning step, need to ground some state to calculate the cost or others
// return a new state which is the grounded version of s, by a parameter value map
State* Rule::groundAStateByRuleParamMap(State* s, ParamGroundedMapInARule& groundings, bool toGroundStateValue,bool ifRealTimeQueryStateValue,ParamValue knownStateVal,bool fullGroundStateOwners)
{
    State* groundedState = s->clone();

    vector<ParamValue>::iterator ownerIt;
    ParamGroundedMapInARule::iterator paramMapIt;

    // check if all the stateOwner parameters grounded
    for (ownerIt = groundedState->stateOwnerList.begin(); ownerIt != groundedState->stateOwnerList.end(); ++ ownerIt)
    {
        if (isParamValueUnGrounded(*ownerIt))
        {
            // look for the value of this variable in the parameter map
            string varName = ActionParameter::ParamValueToString((ParamValue)(*ownerIt));
            paramMapIt = groundings.find(varName);
            if (paramMapIt == groundings.end())
            {
                if (fullGroundStateOwners)
                {
                    delete groundedState;
                    return 0;
                }
            }
            else
                ((ParamValue&)(*ownerIt)) = paramMapIt->second;
        }
    }

    // try to ground the state value if it is ungrounded
    if (isParameterUnGrounded(*(s->stateVariable)))
    {
        // if the state value is assigned
        if ( !(knownStateVal == UNDEFINED_VALUE))
        {
            groundedState->stateVariable->assignValue(knownStateVal);
        }
        else
        {
            // look for the value of this variable in the parameter map
            paramMapIt = groundings.find(groundedState->stateVariable->stringRepresentation());
            if (paramMapIt != groundings.end())
                groundedState->stateVariable->assignValue(paramMapIt->second);
            else if (ifRealTimeQueryStateValue)
            {
                ParamValue value = groundedState->getParamValue();
                if ((value == UNDEFINED_VALUE) && toGroundStateValue)
                {
                    delete groundedState;
                    return 0;
                }

                groundedState->stateVariable->assignValue(value);
            }
            else if (toGroundStateValue)
            {
                delete groundedState;
                return 0;
            }
        }
    }

    return groundedState;

}

bool Rule::isRuleUnGrounded( Rule* rule)
{
    // Check if the actor grounded
    if (isParamValueUnGrounded(rule->actor))
        return true;

    // Check if all the action parameters grounded
    list<ActionParameter> parameters = rule->action->getParameters();
    list<ActionParameter>::iterator it;
    for(it = parameters.begin(); it != parameters.end(); ++it)
    {
        if (isParameterUnGrounded(*it))
            return true;
    }

    // check if all the preconditiion parameters grounded
    vector<State*>::iterator itpre;
    for (itpre = rule->preconditionList.begin(); itpre != rule->preconditionList.end(); ++ itpre)
    {
        State* s = *itpre;

        // check if all the stateOwner parameters grounded
        vector<ParamValue>::iterator ownerIt;
        for (ownerIt = s->stateOwnerList.begin(); ownerIt != s->stateOwnerList.end(); ++ ownerIt)
        {
            if (isParamValueUnGrounded(*ownerIt))
                return true;
        }

        // check the state value
        if (isParameterUnGrounded(*(s->stateVariable)))
                return true;
    }

    // Check if all the effect parameters grounded
    vector<EffectPair>::iterator effectIt;
    for(effectIt = rule->effectList.begin(); effectIt != rule->effectList.end(); ++effectIt)
    {
        Effect* e = effectIt->second;

        State* s = e->state;
        // check if all the stateOwner parameters grounded
        vector<ParamValue>::iterator ownerIt;
        for (ownerIt = s->stateOwnerList.begin(); ownerIt != s->stateOwnerList.end(); ++ ownerIt)
        {
            if (isParamValueUnGrounded(*ownerIt))
                return true;
        }

        // check the state value
        if (isParameterUnGrounded( *(s->stateVariable)))
                return true;

        // check the effect value
        if (isParamValueUnGrounded(e->opParamValue))
            return true;
    }

    return false;

}

void Rule::_addParameterIndex(State* s,ParamValue& paramVal)
{
    string paramToStr = ActionParameter::ParamValueToString(paramVal);
    map<string , vector<paramIndex> >::iterator it;
    it = paraIndexMap.find(paramToStr);

    if (it == paraIndexMap.end())
    {
        vector<paramIndex> addresses;
        addresses.push_back(paramIndex(s,&paramVal));
        paraIndexMap.insert(std::pair<string , vector<paramIndex> >(paramToStr,addresses));
    }
    else
    {
        vector<paramIndex>::iterator indexIt;
        for (indexIt = ((vector<paramIndex>&)(it->second)).begin(); indexIt != ((vector<paramIndex>&)(it->second)).end(); ++ indexIt)
        {
            if ( indexIt->first == s)
                return;
        }

        ((vector<paramIndex>&)(it->second)).push_back(paramIndex(s,&paramVal));
    }

}


void Rule::_preProcessRuleParameterIndexes()
{
    // map<string , vector<ParamValue*> >
    // the string is the string representation of an orginal ungrounded parameter,
    // such like: OCPlanner::vector_var[3].stringRepresentation(), see ActionParameter::stringRepresentation()
    // In vector<ParamValue*>, the ParamValue* is the address of one parameter,help easily to find all using places of this parameter in this rule
    // map<string , vector<ParamValue*> > paraIndexMap;

    // Go through all the parameters in this rule

    // Check if the actor grounded
    if (isParamValueUnGrounded(actor))
    {
        _addParameterIndex(0,actor);
    }

    // check if all the preconditiion parameters grounded
    vector<State*>::iterator itpre;
    for (itpre = preconditionList.begin(); itpre != preconditionList.end(); ++ itpre)
    {
        State* s = *itpre;

        // check if all the stateOwner parameters grounded
        vector<ParamValue>::iterator ownerIt;
        for (ownerIt = s->stateOwnerList.begin(); ownerIt != s->stateOwnerList.end(); ++ ownerIt)
        {
            if (isParamValueUnGrounded(*ownerIt))
                _addParameterIndex(s,*ownerIt);
        }

        // check the state value
        if (isParameterUnGrounded(*(s->stateVariable)))
                _addParameterIndex(s,s->stateVariable->getValue());
    }

    // Check if all the action parameters grounded
    list<ActionParameter> parameters = action->getParameters();
    list<ActionParameter>::iterator it;
    for(it = parameters.begin(); it != parameters.end(); ++it)
    {
        if (isParameterUnGrounded(*it))
            _addParameterIndex(0,((ActionParameter)(*it)).getValue());
    }

    // Check if all the effect parameters grounded
    vector<EffectPair>::iterator effectIt;
    for(effectIt = effectList.begin(); effectIt != effectList.end(); ++effectIt)
    {
        Effect* e = effectIt->second;

        State* s = e->state;
        // check if all the stateOwner parameters grounded
        vector<ParamValue>::iterator ownerIt;
        for (ownerIt = s->stateOwnerList.begin(); ownerIt != s->stateOwnerList.end(); ++ ownerIt)
        {
            if (isParamValueUnGrounded(*ownerIt))
                _addParameterIndex(s,*ownerIt);
        }


        // check the effect value
        if (isParamValueUnGrounded(e->opParamValue))
            _addParameterIndex(s,e->opParamValue);

        //  not need to add the old state value in index
//        // check the old state value
//        if (isParameterUnGrounded( *(s->stateVariable)))
//                _addParameterIndex(s,s->stateVariable->getValue());

    }

    // Check if all the cost calcuation states parameters grounded
    vector<CostHeuristic>::iterator costIt;
    for(costIt = CostHeuristics.begin(); costIt != CostHeuristics.end(); ++costIt)
    {
        State* s = ((CostHeuristic)(*costIt)).cost_cal_state;
        vector<ParamValue>::iterator ownerIt;
        for (ownerIt = s->stateOwnerList.begin(); ownerIt != s->stateOwnerList.end(); ++ ownerIt)
        {
            if (isParamValueUnGrounded(*ownerIt))
                _addParameterIndex(s,*ownerIt);
        }

        // check the state value
        if (isParameterUnGrounded( *(s->stateVariable)))
                _addParameterIndex(s,s->stateVariable->getValue());

    }

}

// todo: this is not complete. if other users have more requiment, need to implment his own cases.
bool  Rule::isRulePossibleToHelpToAchieveGoal(State* goal, bool &directHelp)
{
    vector<EffectPair>::iterator effectIt;
    for(effectIt = effectList.begin(); effectIt != effectList.end(); ++effectIt)
    {
        bool help = false;
        Effect* e = effectIt->second;
        State* s = e->state;
        if (s->name() == goal->name())
        {
            help = true;
            directHelp = false;
            switch (e->effectOp)
            {
            case OP_ASSIGN:
                if (goal->stateType == STATE_NOT_EQUAL_TO)
                {
                    help = false;

                    if (! isParamValueUnGrounded(e->opParamValue))
                    {
                        help = !(e->opParamValue == goal->getParamValue());
                    }

                }
                else if (goal->stateType == STATE_EQUAL_TO)
                {
                    if (! isParamValueUnGrounded(e->opParamValue))
                    {
                        directHelp = e->opParamValue == goal->getParamValue();
                        help = directHelp;
                    }
                }
                break;

            case OP_ASSIGN_NOT_EQUAL_TO:
                if (goal->stateType == STATE_EQUAL_TO)
                    help = false;
                break;

            case OP_ASSIGN_GREATER_THAN:
                if (goal->stateType != STATE_GREATER_THAN)
                    help = false;
                break;
            case OP_ASSIGN_LESS_THAN:
                if (goal->stateType != STATE_LESS_THAN)
                    help = false;
                break;
            case OP_REVERSE:
            case OP_ADD:
            case OP_SUB:
            case OP_MUL:
            case OP_DIV:
            case OP_NUM_OPS:
            default:
                help = true;
                break;

            }

        }

        if (help == true)
            return true;
    }

    return false;
}
