/*
 * Strips.h
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

#ifndef _OCPANNER_STRIPS_H
#define _OCPANNER_STRIPS_H


#include <vector>
#include <boost/variant.hpp>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParamType.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionType.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PetAction.h>
#include "PlanningHeaderFiles.h"
#include "Inquery.h"

using namespace std;
using namespace opencog::pai;
using namespace boost;

// The Actions described in STRIPS format
// Including the preconditions,effects,parameters of the action
using namespace opencog::pai;

namespace opencog { namespace oac {



    enum EFFECT_OPERATOR_TYPE
    {
        OP_REVERSE, // this is only for the bool variables
        OP_ASSIGN,  // this operator can be used in any variable type =
        OP_ASSIGN_NOT_EQUAL_TO, // this operator can be used in any variable type !=
        OP_ASSIGN_GREATER_THAN, // this operator can be used in any variable type >
        OP_ASSIGN_LESS_THAN, // this operator can be used in any variable type <
        OP_ADD,     // only for numeric variables +=
        OP_SUB,     // only for numeric variables -=
        OP_MUL,     // only for numeric variables *=
        OP_DIV,     // only for numeric variables /=
        OP_NUM_OPS  // must always be the last one in this list.
    };

    extern const char* EFFECT_OPERATOR_NAME[9];


    // There are 3 kinds of state types:
    // 1. equal, which can be used in non-numeric and numeric state values: e.g.: the price of Book1 is 15 dollors
    // 2. fuzzy value, e.g.: the price of Book1 is between 10~20 dollors
    // 3. comparator: the price of Book1 is greater than 10, less than 20
    enum StateType
    {
        STATE_EQUAL_TO,     // EvaluationLink
        STATE_NOT_EQUAL_TO,     // EvaluationLink
        STATE_FUZZY_WITHIN, // EvaluationLink + PredicationNode "Fuzzy_within"
        STATE_GREATER_THAN, // GreaterThanLink
        STATE_LESS_THAN     // LessThanLink
    };

    // some kind of state values cannot directly get from the Atomspace.see inquery.h
    // so each of the state value need a coresponding funciton to inquery the state value in real time.
    // the vector<string> is the stateOwnerList
    typedef ParamValue (*InqueryStateFun)(const vector<ParamValue>&);

    // return a vector of all the possible values for grounding a variable in a rule
    // if cannot find proper value, return a empty vector
    typedef vector<ParamValue> (*InqueryVariableFun)(const vector<ParamValue>&);

    // A state is an environment variable representing some feature of the system in a certain time point
    // like the foodState of the egg(id_5904) is now RAW
    // it would be represented as an predicate link in the Atomspace
    // e.g.
    /*(AtTimeLink (stv 1 1) (av -3 0 0)
       (TimeNode "12491327572" (av -3 0 0))
       (EvaluationLink (stv 1 0.0012484394) (av -3 0 0)
          (PredicateNode "foodState" (av -3 1 0))
          (ListLink (av -3 0 0)
             (ObjectNode "id_5904" (av -3 0 0))
             (ConceptNode "RAW" (av -3 0 0))
          )
       )
    )*/
    class State
    {
    public:
        ActionParameter* stateVariable;

        // whose feature this state describes. e.g. the robot's energy
        // sometimes it's more than one ower, e.g. the relationship between RobotA and RobotB
        // typedef variant<string, Rotation, Vector, Entity > ParamValue
        // the owner can be any type
        vector<ParamValue> stateOwnerList;

        string stateName;

        string name() const {return stateName;}

        // see the enum StateType
        StateType stateType;

        // some kinds of state is not apparent, need real-time inquery from the system
        // e.g.: Distance(a,b) - the distance between a and b, it's usually not apparent, need to call corresponding funciton to calcuate
        // e.g.: the height of Ben, this is a essential attribute, which is usuall apparent and not changed, so this is not need a real-time inquery
        bool need_inquery;

        // if the need_inquery is true, then it needs a inquery funciton to get the value of the state in real-time
        InqueryStateFun inqueryStateFun;

        void changeStateType(StateType newType){this->stateType = newType;}

        // If this state doesn't need real time inquery, this fuction will return the direct value of the variable.
        // If it need real time inquery, it will do inquery and return the value.
        ParamValue getParamValue();

        ActionParamType getActionParamType()  {return stateVariable->getType();}

        //map<string , vector<ParamValue*> > paraIndexMap; // this is only used when this state is inside a rule, the rule class will assign the values to this map

        State(string _stateName, ActionParamType _valuetype,StateType _stateType, ParamValue  _ParamValue,
              vector<ParamValue> _stateOwnerList, bool _need_inquery = false, InqueryStateFun _inqueryStateFun = 0);

        State(string _stateName, ActionParamType _valuetype ,StateType _stateType, ParamValue _ParamValue,
               bool _need_inquery = false, InqueryStateFun _inqueryStateFun = 0);

        State(){}

        // clone a same state
        State* clone();
        ~State();

        void assignValue(const ParamValue& newValue);

        void addOwner(ParamValue& _owner)
        {
            stateOwnerList.push_back(_owner);
        }

        inline bool isSameState(const State& other) const
        {
            if (name() == other.name())
            {
                if (std::equal(stateOwnerList.begin(),stateOwnerList.end(),other.stateOwnerList.begin()))
                    return true;
            }

            return false;

            // return ((name() == other.name())&&(stateOwnerList == other.stateOwnerList));
        }

        // @ satisfiedDegree is a return value between (-infinity,1.0], which shows how many percentage has this goal been achieved,
        //   if the value is getting father away from goal, it will be negative
        //   satisfiedDegree = |original - current|/|original - goal|
        //   when it's a boolean goal, only can be 0.0 or 1.0
        // @ original_state is the corresponding begin state of this goal state, so that we can compare the current state to both fo the goal and origninal states
        //                  to calculate its satisfiedDegree value.
        // when original_state is not given (defaultly 0), then no satisfiedDegree is going to be calculated
        bool isSatisfiedMe( ParamValue& value, float& satisfiedDegree,  State *original_state = 0);
        bool isSatisfied( State& goal, float &satisfiedDegree,  State *original_state = 0) ;

        // To get int,float value or fuzzy int or float value from a state
        // For convenience, we will also consider int value as float value
        // if the value type is not numberic, return false and the floatVal and fuzzyFloat will not be assigned value
        bool getNumbericValues(int& intVal, float& floatVal,opencog::pai::FuzzyIntervalInt& fuzzyInt, opencog::pai::FuzzyIntervalFloat& fuzzyFloat);

        // please make sure this a numberic state before call this function
        float getFloatValueFromNumbericState();

        bool isNumbericState() const;

        // About the calculation of Satisfie Degree
        // compare 3 ParamValue for a same state: the original ParamValue, the current ParamValue and the goal ParamValue
        // to see how many persentage the current value has achieved the goal, compared to the original value
        // it can be negative, when it's getting farther away from the goal than the original value
        // For these 3, the state should be the same state, but the state operator type can be different, e.g.:
        // the goal is to eat more than 10 apples, and at the beginning 2 apple has been eaten, and current has finished eaten 3 apples and begin to eat the 6th one, so:
        // original: Num_eaten(apple) = 3
        // current:  5 < Num_eaten(apple) < 6
        // goal:     Num_eaten(apple) > 10
        // so the distance between original and goal is 10 - 3 = 7
        // the distance between current and goal is ((10 - 5) +(10 - 6)) / 2 = 5.5
        // the SatifiedDegree = (7 - 5.5)/7 = 0.2143
        float static calculateNumbericsatisfiedDegree(float goal, float current, float origin);
        float static calculateNumbericsatisfiedDegree(const FuzzyIntervalFloat& goal, float current, float origin);
        float static calculateNumbericsatisfiedDegree(const FuzzyIntervalFloat& goal, const FuzzyIntervalFloat& current, const FuzzyIntervalFloat& origin);
        float static distanceBetween2FuzzyFloat(const FuzzyIntervalFloat& goal, const FuzzyIntervalFloat& other);

        inline bool operator == (State& other)
        {
            if (name() != other.name())
                return false;

            if (!(stateVariable == other.stateVariable))
                return false;

            if ( !(this->stateOwnerList == other.stateOwnerList))
                return false;

            return true;

        }


    };

    // map to save grounded values for one rule:
    // map<parameter name, grounded value>
    // e.g.: <$Entity0,Robot001>
    //       <$Vector0,Vector(45,82,29)>
    //       <$Entity1,Battery83483>
    typedef map<string, ParamValue> ParamGroundedMapInARule;

    // pair<the state this varaible belongs to, one address of this variable>
    typedef pair<State*,ParamValue*> paramIndex;


    class Effect
    {
    public:
        // the specific state this effect will take effect on
        State* state; // it recoards the orginal value of this state which has not been changed by this effect

        //e.g. when this effect is to add the old ParamValue by 5,
        //then effectOp = OP_ADD, opParamValue = 5
        EFFECT_OPERATOR_TYPE effectOp;
        ParamValue opParamValue;

        Effect(State* _state, EFFECT_OPERATOR_TYPE _op, ParamValue _OPValue);

        // execute the effect to a grounded state. will change this state.
        // will look up the value from groundings if the opParamValue is ungrouned
        // only when there are misusge of the value type of opParamValue, it will return false,
        // e.g. if assign a string to a bool state, it will return false
        static bool executeEffectOp(State* state, Effect* effect, ParamGroundedMapInARule &groundings);

        // make sure the value type of the operater value is the same with the value type of the state
        // and also this value type can be the parameter of this operator
        static bool _AssertValueType(State& _state, EFFECT_OPERATOR_TYPE effectOp, ParamValue &OPValue);


    };

    struct CostHeuristic //  cost = value(cost_cal_state) * cost_coefficient
    {
        State* cost_cal_state;
        float cost_coefficient;
        CostHeuristic(State* _state, float _coefficient)
        {
            cost_cal_state = _state;
            cost_coefficient = _coefficient;
        }
    };


    // the float in this pair is the probability of the happenning of this Effect
    typedef pair<float,Effect*> EffectPair;


    // ToBeImproved, in fact,there should rules for selecting values as well,
    // so that the variable grounding process can be also consided as the same reasoning process as the main planning process.
    // But currently, we don't have enough time to implement this, so we add such predefined BestNumericVariableInqueryStruct for selecting grounding values
    // e.g., for a rule to select a Adjacent location of a given location x, currently we would just assign a function to return its possible 26 neighbours,
    // but the right way, we should define 26 rules to tell the planner what is "Adjacent location"
    struct BestNumericVariableInqueryStruct
    {
        State* goalState;
        InqueryVariableFun bestNumericVariableInqueryFun;
    };

    // the rule to define the preconditions of an action and what effects it would cause
    // A rule will be represented in the Atomspace by ImplicationLink
   /*     ImplicationLink
    *         AndLink
    *             AndLink //(preconditions)
    *                 EvaluationLink
    *                     GroundedPredicateNode "precondition_1_name"
    *                     ListLink
    *                         Node:arguments
    *                         ...
    *                 EvaluationLink
    *                     PredicateNode         "precondition_2_name"
    *                     ListLink
    *                         Node:arguments
    *                         ...
    *                 ...
    *
    *             ExecutionLink //(action)
    *                 GroundedSchemaNode "schema_name"
    *                 ListLink
    *                     Node:arguments
    *                     ...
    *                 ...
    *
    *        AndLink //(effects)
    *             (SimpleTruthValue indicates the probability of the happening of an effect
    *             EvaluationLink <0.5,1.0>
    *                 GroundedPredicateNode "Effect_state_1_name"
    *                 ListLink
    *                     Node:arguments
    *                     ...
    *             EvaluationLink <0.7,1.0>
    *                 GroundedPredicateNode "Effect_stare_2_name"
    *                 ListLink
    *                     Node:arguments
    *                     ...
    */
    class Rule
    {
    public:
        PetAction* action;

        // the actor who carry out this action, usually an Entity
        ParamValue actor;

        // All the precondition required to perform this action
        vector<State*> preconditionList;

        // All the effect this action may cause
        // there are probability for each effect
        vector<EffectPair> effectList;

        // The cost function as heuristics, linearly
        // the total cost = basic_cost + cost_coefficient1 * value(cost_cal_state1) + cost_coefficient2 * value(cost_cal_state2) + ...
        float basic_cost;
        vector<CostHeuristic> CostHeuristics;

        // return if this rule is recursive. A recursive rule means its precondition and effect are the same state
        //  e.g. if can move from A to B & can move from B to C, then can move from A to C , is a recursive rule
        bool IsRecursiveRule;

        // pre-defined or learnt fuction to inquery the best numeric value to ground this rule to achieve a numeric states,
        // which is the most closed to the a grounded numeric goal
        map<string,BestNumericVariableInqueryStruct> bestNumericVariableinqueryStateFuns;

        // ungrounded parameter indexes
        // map<string , vector<ParamValue&> >
        // the string is the string representation of an orginal ungrounded parameter,
        // such like: OCPlanner::vector_var[3].stringRepresentation(), see ActionParameter::stringRepresentation()
        // In vector<ParamValue*>, the ParamValue* is the address of one parameter,help easily to find all using places of this parameter in this rule
        map<string , vector<paramIndex> > paraIndexMap;

        // constructors
        Rule(PetAction* _action, ParamValue _actor, vector<State*> _preconditionList, vector<EffectPair> _effectList, float _basic_cost):
            action(_action) , actor(_actor), preconditionList(_preconditionList), effectList(_effectList), basic_cost(_basic_cost),
            CostHeuristics(), IsRecursiveRule(false), bestNumericVariableinqueryStateFuns(), paraIndexMap(){}

        Rule(PetAction* _action, ParamValue _actor, float _basic_cost):
            action(_action) , actor(_actor), preconditionList(), effectList(), basic_cost(_basic_cost),
            CostHeuristics(), IsRecursiveRule(false), bestNumericVariableinqueryStateFuns(), paraIndexMap(){}

        float getBasicCost();

        // the cost calculation is : basic_cost + cost_coefficient1 * value(cost_cal_state1) + cost_coefficient2 * value(cost_cal_state2) + ...
        static float getCost(float basic_cost,vector<CostHeuristic>& CostHeuristics, ParamGroundedMapInARule& groudings);

        void addEffect(EffectPair effect)
        {
            effectList.push_back(effect);
        }

        void addPrecondition(State* precondition)
        {
            preconditionList.push_back(precondition);
        }

        void addCostHeuristic(CostHeuristic costH)
        {
            CostHeuristics.push_back(costH);
        }

        // need to be called after a rule is finished added all its information (predictions, effects...)
        // to add parameter index and check if it's a recursive rule
        void preProcessRule();

        // in some planning step, need to ground some state to calculate the cost or others
        // return a new state which is the grounded version of s, by a parameter value map
        // if the "groundings" cannot ground all the variables in this state, return 0
       static State* groundAStateByRuleParamMap(State* s, ParamGroundedMapInARule& groundings, bool ifRealTimeQueryStateValue = true);

        bool static isRuleUnGrounded( Rule* rule);

        // Check if a parameter is an ungrounded parameter
        // Compared to the bool_var[PARAMETER_NUM],str_var[PARAMETER_NUM]...in PlanningHeaderFiles.h
        bool static isParameterUnGrounded(ActionParameter &param);
        bool static isParamValueUnGrounded( ParamValue &paramVal);
        bool static isUnGroundedString( string &s);
        bool static isUnGroundedVector(Vector& v);
        bool static isUnGroundedEntity( Entity &e);

    protected:

        // go through all the parameters in this rule and add their indexes to paraIndexMap
        void _preProcessRuleParameterIndexes();

        // Add one parameter to index
        void _addParameterIndex(State *s, ParamValue &paramVal);

        // check if this rule is a recursive rule
        // Recursive rule is to break a problem into the same problems of smaller scales
        bool _isRecursiveRule();

    };


}}


#endif
