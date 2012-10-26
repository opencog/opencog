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
bool State::isSatisfiedMe(const StateValue& value)
{
    State other(this->name(),this->getStateValuleType(),this->stateType,value,this->stateOwnerList);
    return other.isSatisfied(*this);
}

// pls make sure the goal describes the same state content with this state first
bool State::isSatisfied(const State &goal)
{
    if ((goal.getStateType() == getStateType())&&(getStateValue() == goal.getStateValue()))
       return true;

   // opencog::pai::FuzzyIntervalInt* fuzzyIntP, fuzzyGoalIntP;
    opencog::pai::FuzzyIntervalInt fuzzyInt, fuzzyGoalInt;
   // opencog::pai::FuzzyIntervalFloat* fuzzyFloatP, fuzzyGoalFloatP;
    opencog::pai::FuzzyIntervalFloat fuzzyFloat, fuzzyGoalFloat;
    int intVal, intGoalVal;
    float floatVal, floatGoalVal;

    // if the value type is int, we'll also assign a value to the float variables for convenience of calculation
    if (getStateValuleType().getCode() == FUZZY_INTERVAL_INT_CODE)
    {
       fuzzyInt = boost::get<opencog::pai::FuzzyIntervalInt>(getStateValue());
       fuzzyFloat = FuzzyIntervalFloat((float)fuzzyInt.bound_low, (float)fuzzyInt.bound_high);
    }
    else if (getStateValuleType().getCode() == FUZZY_INTERVAL_FLOAT_CODE)
        fuzzyFloat = boost::get<opencog::pai::FuzzyIntervalFloat>(getStateValue());
    else if (getStateValuleType().getCode()  == INT_CODE)
    {
        intVal = boost::get<int>(goal.getStateValue());
        floatVal = (float)intVal;
    }
    else if (getStateValuleType().getCode()  == FLOAT_CODE)
        floatVal = boost::get<float>(goal.getStateValue());

    if (goal.getStateValuleType().getCode() == FUZZY_INTERVAL_INT_CODE)
    {
       fuzzyGoalInt = boost::get<opencog::pai::FuzzyIntervalInt>(goal.getStateValue());
       fuzzyGoalFloat = FuzzyIntervalFloat((float)fuzzyGoalInt.bound_low, (float)fuzzyGoalInt.bound_high);
    }
    else if (goal.getStateValuleType().getCode() == FUZZY_INTERVAL_FLOAT_CODE)
       fuzzyGoalFloat = boost::get<opencog::pai::FuzzyIntervalFloat>(goal.getStateValue());
    else if (goal.getStateValuleType().getCode() == INT_CODE)
    {
       intGoalVal = boost::get<int>(goal.getStateValue());
       floatGoalVal = intGoalVal;
    }
    else if (goal.getStateValuleType().getCode() == FLOAT_CODE)
       floatGoalVal = boost::get<float>(goal.getStateValue());

    switch (goal.getStateType())
    {
    case STATE_EQUAL_TO:
         return false;

    case STATE_NOT_EQUAL_TO:

        if (stateType  == STATE_EQUAL_TO)
        {
            if (!(getStateValue() == goal.getStateValue()))
                return true;
            else
                return false;
        }

        if (stateType == STATE_FUZZY_WITHIN)
        {
            if (getStateValuleType().getCode()  == FUZZY_INTERVAL_INT_CODE)
                return ! (fuzzyInt.isInsideMe(intGoalVal));
            else if (getStateValuleType().getCode()  == FUZZY_INTERVAL_FLOAT_CODE)
                return ! (fuzzyFloat.isInsideMe(floatGoalVal));
        }

        if (stateType  == STATE_GREATER_THAN)
            return (floatVal > floatGoalVal);

        if (stateType  == STATE_LESS_THAN)
            return (floatVal < floatGoalVal);

        return false;

    case STATE_GREATER_THAN:

        if (stateType  == STATE_EQUAL_TO)
            return (floatVal > floatGoalVal);

        if (stateType  == STATE_FUZZY_WITHIN)
            return (fuzzyFloat.bound_low > floatGoalVal);

        return false;

    case STATE_LESS_THAN:

        if (stateType  == STATE_EQUAL_TO)
            return (floatVal < floatGoalVal);

        if (stateType == STATE_FUZZY_WITHIN)
            return (fuzzyFloat.bound_high < floatGoalVal);

        return false;

    case STATE_FUZZY_WITHIN:
        if (stateType  == STATE_EQUAL_TO)
            return (fuzzyGoalFloat.isInsideMe(floatVal));

        if (stateType  == STATE_FUZZY_WITHIN)
            return (fuzzyGoalFloat.isInsideMe(fuzzyFloat));

        return false;

    default:
        return false;

    }
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
