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

const char* opencog::oac::EFFECT_OPERATOR_NAME[7] =
{
    "OP_REVERSE", // this is only for the bool variables
    "OP_ASSIGN",  // this operator can be used in any variable type =
    "OP_ASSIGN_NOT_EQUAL_TO", // this operator can be used in any variable type !=
    "OP_ADD",     // only for numeric variables +=
    "OP_SUB",     // only for numeric variables -=
    "OP_MUL",     // only for numeric variables *=
    "OP_DIV"      // only for numeric variables /=
};

State::State(string _stateName, StateValuleType _valuetype,StateType _stateType, StateValue  _stateValue,
             vector<StateValue> _stateOwnerList, bool _need_inquery, InqueryFun _inqueryFun)
    : stateOwnerList(_stateOwnerList),need_inquery(_need_inquery),inqueryFun(_inqueryFun)
{
    State(_stateName, _valuetype,_stateType,  _stateValue);
}

State::State(string _stateName, StateValuleType _valuetype ,StateType _stateType, StateValue _stateValue,
             bool _need_inquery, InqueryFun _inqueryFun)
    : stateType(_stateType),need_inquery(_need_inquery),inqueryFun(_inqueryFun)
{
    stateVariable = new StateVariable(_stateName,_valuetype,_stateValue);
}

State::~State()
{
    delete stateVariable;
}

void State::assignValue(const StateValue& newValue)
{
    stateVariable->assignValue(newValue);
}

// I am the goal, I want to check if this @param value is satisfied me
bool State::isSatisfiedMe(const StateValue& value, float &satisfiedDegree, const State *original_state) const
{
    State other(this->name(),this->getStateValuleType(),this->stateType,value,this->stateOwnerList);
    return other.isSatisfied(*this,satisfiedDegree,original_state);
}

// pls make sure the goal describes the same state content with this state first
bool State::isSatisfied(const State &goal, float& satisfiedDegree, const State *original_state) const
{
    if ((goal.getStateType() == getStateType())&&(getStateValue() == goal.getStateValue()))
    {
       satisfiedDegree = 1.0f;
       return true;
    }


    // For non-numberic statevalues,  the satisfiedDegree is always 0.0 or 1.0
    if (! StateValuleType::isNumbericValueType( goal.getStateValuleType().getCode()))
    {
       if (goal.getStateType() == STATE_EQUAL_TO)
       {
           satisfiedDegree = 0.0f;
           return false;
       }
       else if (goal.getStateType() == STATE_NOT_EQUAL_TO)
       {
           if (!(getStateValue() == goal.getStateValue()))
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

    // Deal with numberic statevalues:

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
    float ori,cur;

    switch (goal.getStateType())
    {
    case STATE_EQUAL_TO:

        if (original_state != 0)
        {
            if (getStateType() ==  STATE_EQUAL_TO)
            {
                if (original_state->getStateType() == STATE_EQUAL_TO)
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
            if (!(getStateValue() == goal.getStateValue()))
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
            if (getStateValuleType().getCode()  == FUZZY_INTERVAL_INT_CODE)
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
            else if (getStateValuleType().getCode()  == FUZZY_INTERVAL_FLOAT_CODE)
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

        float ori,cur;

        if (stateType  == STATE_FUZZY_WITHIN)
            cur = fuzzyFloat.bound_low ;
        else if ((stateType  == STATE_EQUAL_TO) || (stateType == STATE_GREATER_THAN))
            cur = floatVal;

        if (original_state)
        {
            if (cur > floatGoalVal)
                satisfiedDegree = 1.0f;
            else
            {
                switch (original_state->getStateType())
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
        else if ((stateType  == STATE_EQUAL_TO) || (stateType == STATE_GREATER_THAN))
            cur = floatVal;

        if (original_state)
        {
            if (cur < floatGoalVal)
                satisfiedDegree = 1.0f;
            else
            {
                switch (original_state->getStateType())
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
                    if (original_state->getStateType() == STATE_EQUAL_TO)
                        satisfiedDegree = State::calculateNumbericsatisfiedDegree(fuzzyGoalFloat,floatVal,floatOriVal);
                    else if (original_state->getStateType() == STATE_FUZZY_WITHIN) //TODO:
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
                    if (original_state->getStateType() == STATE_EQUAL_TO)  //TODO:
                        satisfiedDegree = State::calculateNumbericsatisfiedDegree(fuzzyGoalFloat,(fuzzyFloat.bound_high + fuzzyFloat.bound_low)/2.0f,floatOriVal);
                    else if (original_state->getStateType() == STATE_FUZZY_WITHIN)
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

bool State::getNumbericValues(int& intVal, float& floatVal,opencog::pai::FuzzyIntervalInt& fuzzyInt, opencog::pai::FuzzyIntervalFloat& fuzzyFloat) const
{
    if (getStateValuleType().getCode() == FUZZY_INTERVAL_INT_CODE)
    {
       fuzzyInt = boost::get<opencog::pai::FuzzyIntervalInt>(getStateValue());
       fuzzyFloat = FuzzyIntervalFloat((float)fuzzyInt.bound_low, (float)fuzzyInt.bound_high);
    }
    else if (getStateValuleType().getCode() == FUZZY_INTERVAL_FLOAT_CODE)
        fuzzyFloat = boost::get<opencog::pai::FuzzyIntervalFloat>(getStateValue());
    else if (getStateValuleType().getCode()  == INT_CODE)
    {
        intVal = boost::get<int>(getStateValue());
        floatVal = (float)intVal;
    }
    else if (getStateValuleType().getCode()  == FLOAT_CODE)
        floatVal = boost::get<float>(getStateValue());
    else
        return false;

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
    float disCurToGoal,disOriToGoal;

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

Effect::Effect(State* _state, EFFECT_OPERATOR_TYPE _op, StateValue _OPValue)
{
    OC_ASSERT(_AssertValueType(*_state,_op,_OPValue),
              "Effect constructor: got invalid effect value type: s% for state value type: %s, operator: %s, in state: %s\n",
              _OPValue.type().name(), _state->getStateValuleType().getName().c_str(),EFFECT_OPERATOR_NAME[_op],_state->name().c_str());

    state = _state;
    effectOp = _op;
    opStateValue = _OPValue;
}

bool Effect::executeEffectOp()
{
    if (effectOp != OP_ASSIGN_NOT_EQUAL_TO)
    {
        if (state->getStateType() != STATE_EQUAL_TO)
            state->changeStateType(STATE_EQUAL_TO);
    }
    else
    {
        if (state->getStateType() != STATE_NOT_EQUAL_TO)
            state->changeStateType(STATE_NOT_EQUAL_TO);
    }

    if (effectOp == OP_ASSIGN)
    {
        state->assignValue(opStateValue);

    }
    else if (effectOp == OP_ASSIGN_NOT_EQUAL_TO)
    {
        state->assignValue(opStateValue);

    }
    else if (effectOp == OP_REVERSE)
    {
        string oldStr = boost::get<string>(state->getStateValue());
        if (oldStr == "true")
            state->assignValue(StateValue(string("false")));
        else if (oldStr == "false")
            state->assignValue(StateValue(string("true")));
        else
            return false;
    }
    else
    {
        string oldStr = boost::get<string>(state->getStateValue());
        string opvStr = boost::get<string>(opStateValue);

        double oldv = atof(oldStr.c_str());
        double opv = atof(opvStr.c_str());
        double newV;
        switch (effectOp)
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

        StateValue sv;
        if (state->getStateValuleType().getCode() == INT_CODE)
        {
            sv = toString<int>((int)newV);
            state->assignValue( sv );
        }
        else if (state->getStateValuleType().getCode() == FLOAT_CODE)
        {
            sv = toString<float>((float)newV);
            state->assignValue( sv );
        }
        else
            return false;
    }

    return true;

}

// variant<bool, int, float, double, string, Rotation, Vector > StateValue
bool Effect::_AssertValueType(State& _state, EFFECT_OPERATOR_TYPE _effectOp, StateValue &_OPValue)
{
    // first,make sure the value type of the operater value is the same with the value type of the state

    if (! StateVariable::areFromSameType(_state.getStateValue(),_OPValue))
        return false;

    // if the value type is string, Entity, Vector, Rotation or fuzzy values , it's only allow to use the OP_ASSIGN operator
    if (_state.getStateValuleType().getCode() == STRING_CODE || _state.getStateValuleType().getCode() == ROTATION_CODE ||
        _state.getStateValuleType().getCode() == VECTOR_CODE || _state.getStateValuleType().getCode() == ENTITY_CODE ||
        _state.getStateValuleType().getCode() == FUZZY_INTERVAL_INT_CODE ||  _state.getStateValuleType().getCode() == FUZZY_INTERVAL_FLOAT_CODE   )
    {
        return (_effectOp == OP_ASSIGN);
    }

    // and also this value type should be able to be the parameter for this operator
    switch(_effectOp)
    {
    case OP_REVERSE:
        return (_state.getStateValuleType().getCode() == BOOLEAN_CODE);
    case OP_ASSIGN: // all value types can use OP_ASSIGN and OP_ASSIGN_NOT_EQUAL_TO
    case OP_ASSIGN_NOT_EQUAL_TO:
        return true;

    case OP_ADD:// only for the numeric value types
    case OP_SUB:// only for the numeric value types
    case OP_MUL:// only for the numeric value types
    case OP_DIV:// only for the numeric value types
    {
       string* v =  boost::get<string>(&_OPValue);
       if ( v == 0)
           return false;

       if ( (_state.getStateValuleType().getCode() != INT_CODE) && (_state.getStateValuleType().getCode() != FLOAT_CODE))
           return false;

    }

    default:
        return false;

    }
}
