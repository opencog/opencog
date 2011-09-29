/*
 * opencog/embodiment/Control/OperationalAvatarController/RuleEngine.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#ifndef RULE_ENGINE_H_
#define RULE_ENGINE_H_

#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/HandleSet.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <opencog/embodiment/Control/EmbodimentConfig.h>

#include <opencog/util/exceptions.h>
#include <opencog/comboreduct/combo/variable_unifier.h>

#include <opencog/embodiment/Control/Procedure/ProcedureInterpreter.h>

using namespace opencog;

struct lua_State;

namespace opencog { namespace oac {

class OAC;
class SchemaRunner;
class RuleEngineUtil;
class RuleEngineLearnedTricksHandler;

enum ReinforcementType {
    REWARD,
    PUNISH
};

/**
 * class RuleEngine
 * This is an action planner. An action planner chose from an action database(rules) one
 * action that is more appropriate to be executed on the current context.
 * It works with a predicates base and a set of rules.
 * The predicate base defines the current state of the pet feelings, traits and
 * context(objects and avatar positions, night/day, near... etc ).
 * Traits are defined once, on a file loaded on the startup process.
 * Feelings are changed when pet interacts with the objects and avatars
 * So, the current state of the predicates will influence the selection of the next pet action
 */
class RuleEngine
{

public:


    // inner classes

    /**
     * class EntityPerception
     * Datatype to handle the related pet's perception to an object or avatar
     */
    class EntityPerception
    {
    public:
        EntityPerception( int firstSeenCycle = 0 ) :
                firstSeenCycle( firstSeenCycle ),
                lastSeenCycle( firstSeenCycle ) { }

        inline int getFirstSeenCycle( void ) const {
            return firstSeenCycle;
        };
        inline int getLastSeenCycle( void ) const {
            return lastSeenCycle;
        };
        inline void setLastSeenCycle( int cycle ) {
            lastSeenCycle = cycle;
        };

    private:
        // the first time(cycle) that pet saw an entity
        int firstSeenCycle;
        // the last time(cycle) that pet saw an entity
        int lastSeenCycle;
    };

    typedef std::pair<std::string, EntityPerception> Id_EntityPerception_Pair;
    typedef std::map<std::string, EntityPerception> Id_EntityPerception_Map;
    typedef Id_EntityPerception_Map::iterator Id_EntityPerception_Map_It;
    typedef Id_EntityPerception_Map::const_iterator Id_EntityPerception_Map_Const_It;

    /**
     * Datatype to handle the actions done by the pet or avatar
     */
    class Action : public std::pair<std::string, std::vector<std::string> >
    {
    public:
        inline Action( const std::string& name = "",
                       const std::vector<std::string>& parameters = std::vector<std::string>() ) :
                std::pair<std::string, std::vector<std::string> >( name, parameters ) { }

        inline const std::string& getName( void ) const {
            return first;
        }
        inline void setName( const std::string& name ) {
            first = name;
        }

        inline const std::vector<std::string>& getParameters( void ) const {
            return second;
        }
        inline void setParameters( const std::vector<std::string>& parameters ) {
            second = parameters;
        }

        inline const std::string getParameter( unsigned int index ) const {
            if ( index >= second.size( ) ) {
                return "";
            } // if
            return second[ index ];
        }

    }; // class action

    /**
     * Relation represents an affective relationship whose pet keeps with some object
     * or avatar. It will influence in the choose of the next action
     */
    class Relation
    {
    private:
        std::string name;
        std::string target;

        float intensity;

    public:
        inline Relation( const std::string& name = "",
                         const std::string& target = "",
                         const float intensity = 0.7) :
                name(name), target(target), intensity(intensity) { }

        inline const std::string& getName( void ) const {
            return name;
        }
        inline const std::string& getTarget( void ) const {
            return target;
        }
        inline const float getIntensity( void ) const {
            return intensity;
        }

    }; // class relation

    struct RelationCompare {
        bool operator()(const Relation& a, const Relation& b) const {
            return (a.getName() < b.getName());
        }
    };

    // contains the set of relation names that has been encounter so far
    // that is used to print on the log all the relations so far and their
    // strength
    std::set<std::string> relationNameSet;

    // print in the log all relations and their strength that has been
    // set so far at the level log l
    void logRelations(opencog::Logger::Level l);

    /**
     * Feeling represents an emotional feeling and its intensity
     */
    class Feeling : public std::pair<std::string, float>
    {
    private:
        std::string name;
        float intensity;

    public:
        inline Feeling( const std::string& name = "",
                        const float intensity = 0.7) :
                name(name), intensity(intensity) { }

        inline const std::string& getName( void ) const {
            return name;
        }
        inline const float getIntensity( void ) const {
            return intensity;
        }

    }; // class feeling

    // effects for each rule
    typedef boost::variant<Action, Relation, Feeling> Effect;

    // methods
    RuleEngine( OAC* oac, const std::string& petName )
    throw(opencog::RuntimeException);

    virtual ~RuleEngine( void );


    // This method use AtomSpaceUtil::addPropertyPredicate
    //  to directly update the AtomSpace
    void setPredicateValue( const std::string& name, float value);

    const std::string& getCurrentAction( void ) const;

    const std::string& getCurrentRule( void ) const;

    const std::string& getNextAction( void );

    void runSchemaForCurrentAction( void );

    inline int getCycle( void ) {
        return this->cycle;
    }

    // run the action planner to select the next action
    void processNextAction( void );

    void tryExecuteSchema( const std::string& schemaName );

    inline RuleEngineUtil* getUtil( void ) {
        return this->util;
    }

    // add to 'learned schema handler' a new learned trick
    void addLearnedSchema( const std::string& schemaName );

    // Reward implication links for all selected rules backwards from the given
    // timestamp.
    void rewardRule(unsigned long timestamp);

    // Punish implication links for all selected rules backwards from the given
    // timestamp.
    void punishRule(unsigned long timestamp);

    bool isExecutingSchemaNow( void ) const;

    Procedure::RunningProcedureID getExecutingSchemaID( void ) const;

    bool isSchemaExecFinished( Procedure::RunningProcedureID schemaID ) const;

private:

    friend class RuleEngineUtil;

    OAC * oac;

    // handle the context of the lua(script language)
    lua_State * luaState;

    // this pet name variables
    Handle petHandle;
    std::string petName;

    RuleEngineUtil * util;

    // runs selected schemata
    SchemaRunner * schemaRunner;

    // used to select a learned trick (from a list of learned tricks
    RuleEngineLearnedTricksHandler * learnedTricksHandler;

    // rule engine cycle
    int cycle;

    // Default schema node truth and attention values
    SimpleTruthValue *defaultTruthValue;
    AttentionValue *longTermAttentionValue;

    // feelings names vector. Used to search feelings and get their values
    std::vector<std::string> feelings;

    /**
     * Vector that holds precomputed values of the Gaussian pdf used to
     * reward/punish rule implication links that were selected to execute or are
     * being executed during a time interval
     */
    std::vector<float> gaussianVector;

    // rule maps storing its type, precondition and effect
    std::map<std::string, int> ruleTypeMap;
    std::map<std::string, Effect> ruleEffectMap;
    std::map<std::string, std::string> rulePreconditionMap;

    // list of know avatars and objects - obtained from the latest
    // space map
    Id_EntityPerception_Map avatars;
    Id_EntityPerception_Map objects;

    // last action done
    Action lastPetActionDone;

    // the action currently selected to be executed (Acttually the action to be
    // tried since SchemaRunner do not garrantee that all actions sent to exec
    // will be executed by ProcedureInterpreter).
    Action currentAction;
    Action candidateAction;

    // the rule that lead to the currently selected action to be executed. This
    // info will be used for Reinforcement Learning
    // Carlos Lopes (17-06-08)
    std::string currentRule;
    std::string candidateRule;

    // when storing suggested actions, it is important to store the rules that
    // lead to that action (and also de strength of the action in the overall
    // action selection). This info will be used for Reinforcement Learning.
    typedef std::pair<std::string, float> RuleStrPair;

    // suggested Relations / Feelings / Actions after processing all rules
    std::multiset<Relation, RelationCompare> suggestedRelations;
    std::multiset<Feeling> suggestedFeelings;
    std::multimap<Action, RuleStrPair> suggestedActions;

    // this vector has all possible objects/avatars ids to replace
    // the special target 'var' on rules
    std::vector<std::string> varBindCandidates;

    // rules inspected control variables
    std::string lastInspectedRuleName;
    std::string currentInspectedRuleName;

    std::string triedSchema;

    // a counter that shows how many times an action was being selected sequentially
    unsigned int currentActionRepetitions;

    // a counter that shows how many cycles were passed since avatar ask pet to try some trick
    unsigned int numberOfCyclesSinceAvatarAskToTry;

    // counter used to keep a avatar command alive during several cycles until it is executed
    int lastRequestedCommandCycles;

    // inform ig the avatar ask to try learned schema
    bool avatarAskedToTry;

    // The last action executed x seconds before the current timestamp will be considered the
    // latest agent action
    int cyclesDuringAgentLastAction;

    // group mode variables
    bool groupingMode;
    std::string groupLeaderId;

    // Random number Generator
    opencog::RandGen * rng;

    /**
     * Process all rules suggesting actions / feelings / relations for the
     * activated ones
     */
    void processRules( void );

    /**
     * Precompute the values of a Gaussian pdf.
     * IMPORTANT:
     *
     * @param window The time window
     * @param mean The gaussian function mean
     * @param stdDeviation The gaussian function standard deviation
     * @return A vector with a discrete gaussian distribution
     */
    void preCompGaussianDistributionVector(float window, float mean,
                                           float stdDeviation);

    /**
     * Generic function that reinforce (reward/punish) the selected rules
     * backwards from the given timestamp.
     *
     * @param type The reinforcement type (REWARD or PUNISH)
     * @param timestamp The timestamp to start searching backwards (given a time
     *                  window) for selected rules.
     */
    void reinforceRule(ReinforcementType type, unsigned long timestamp);

    /**
     * Apply reward or punish the implication link who is the second
     * outgoing atom for the given evaluation link.
     *
     * @param type The reinforcement type
     * @param evalLink The evalution link handle that holds the implication link
     * @param factor The factor used to set the final strength for
     *               reward/punish. Range from 0.0 (no reinforcement) to 1.0
     *               (full reinforcement)
     */
    void applyReinforcement(ReinforcementType type,
                            Handle evalLink, float factor);

    /**
     * suggestion methods
     *
     * This methods suggests relationships, feelings and actions to be executed on the next cycle.
     * A suggestion will be evaluated and ranked. The better/best ranked itens will be selected.
     *
     * Note that for suggestAction the name of the rule that leads to that action is also a
     * parameter. This info will be used for Reinforcement Learning.
     * Carlos Lopes (17-06-08)
     */
    void suggestRelation(const std::string& relation,
                         const std::string& target, float intensity = 0.7 );
    void suggestFeeling(const std::string& feeling, float intensity = 0.7 );
    void suggestAction(const std::string& action, const std::string& rule,
                       const std::vector<std::string>& parameters );

    // add a relationship with an object/avatar
    // it invokes the AtomSpaceUtil methods update setup relations predicates
    void addRelation( const Relation& relation);

    // remove a relationship with an object/avatar (i.e. set TV strength to 0)
    // it invokes the AtomSpaceUtil methods update setup relations predicates
    void removeRelation( const Relation& relation );

    // remove relations that are incompatibles
    void removeOppositeRelation( const Relation& relation );

    /**
     * Update current action repetition information in AtomSpace.
     * Representation:
     *
     * FrequencyLink
     *    ConceptNode:currentActionRepetition
     *    NumberNode:<info>
     */
    void updateCurrentActionRepetitions();

    /**
     * Check SchemaRunner to see if the current executing action is
     * finished and, if so, update last pet action and current action
     * repetition predicates.
     */
    void updateSchemaExecutionStatus();

    /**
     * Add GROUNDED_SCHEMA_NODE for the given schema. These nodes are set with
     * default AttentionValue and default TruthValue
     */
    void addSchemaNode(const std::string& schemaName);

    /**
     * Add a rule
     *
     * @param rule The name of the rule
     * @param type The type of the rule
     * @param strength The strength for the
     * @param preconditions A string based boolean expression that is evaluated
     *        to see if the rule is activated or not during a give cycle.
     * @param schema The name of the schema that
     * @param schemaParam A vector containing string representations of the
     *        schema parameters.
     */
    void addRule(const std::string& rule, const int type,
                 const std::map<std::string, float>& modesStrength,
                 const std::string& precondition, const std::string& effect,
                 const std::vector<std::string>& effectParameters);

    /**
     * Add a rule into AtomSpace. The resulting AtomSpace should be:
     *
     * ReferenceLink
     *    PhraseNode:<rule-name>
     *    ImplicationLink <strength>
     *       EvaluationLink
     *           PredicateNode:<precondition>
     *           ListLink
     *       ExecutionLink
     *          GroundedSchemaNode:<schema-name>
     *          ListLink
     *              Node:params ...
     *
     * IMPORTANT: The function signature, as well the AtomSpace representation
     * are subjected to change.
     *
     * @param rule The name of the rule
     * @param type The type of the rule
     * @param strength The strength for the
     * @param preconditions A string based boolean expression that is evaluated
     *        to see if the rule is activated or not during a give cycle.
     * @param schema The name of the schema that
     * @param schemaParam A vector containing string representations of the
     *        schema parameters.
     */
    void addRuleToAtomSpace(const std::string& rule,
                            const std::map<std::string, float>& modesStrength,
                            const std::string& precondition,
                            const std::string& effect,
                            const std::vector<std::string>& effectParameters);

    /**
     * Helper function used to insert the EvaluationLink for the rule
     * precondition.
     *
     * IMPORTANT: The function signature, as well the AtomSpace representation
     * are subjected to change.
     *
     * @param precondition The rule precondition expression as a string
     * @return The handle of the EvaluationLink just added to the AtomSpace.
     */
    Handle addPreconditionEvalLink(const std::string& precondition);

    /**
     * Helper function used to insert the ExecutionLink for the rule
     * effect.
     *
     * IMPORTANT: The function signature, as well the AtomSpace representation
     * are subjected to change.
     *
     * @param schema The name of the schema
     * @param parameters A vector with string representation of the schema
     *        parameters.
    * @param permanent flag to indicate that atoms should not be forgotten by decay importance mechanism
     * @return The handle of the ExecutionLink just added to the AtomSpace.
     */
    Handle addEffectExecLink(const std::string& effect,
                             const std::vector<std::string> parameters,
                             bool permanent)
    throw (opencog::RuntimeException);

    /**
     * Add SchemaDone predicate for the given executed schema and its parameters
     *
     * @param schema The name of the schema
     * @param parameters A vector with string representation of the schema
     *        parameters.
     * @param result True if the schema was executed successfuly or
     *        false otherwise
     */
    void addSchemaDoneOrFailurePred(const std::string& schema,
                                    const std::vector<std::string> parameters,
                                    bool result);

    // helper function: keep the value at the range 0.0 - 1.0
    inline float limitValue( float number ) {
        return std::min( 1.0f, std::max( 0.0f, number ) );
    }

    // helper function: update the value of a predicate, using it's new and old values
    inline float reviseValue( float oldValue, float newValue ) {
        return ( ( oldValue + newValue ) / 2 );
    };

    // cleanup the temporary vectors and variables to the next excetution of the planner
    void prepareForNextCycle( void );

    /*
     * the targets of an action, relation, etc. may be a wildcard variable called 'var'.
     * When 'var' is used, it may be any object or avatar. So, this method keep track of
     * the objects/avatars names that was evaluated by the current rule and are valid.
     * Ex.: done('var')
     * -> lastPetActionDone = bark
     * done('bark') = true
     * done('nap') = false
     * done('bite') = false ...and so on..
     * getValidTargets will return only 'bark'
     */
    const std::vector<std::string>& getValidTargets( void );

    /**
     *
     */
    void updateValidTargets(combo::variable_unifier& unifier);

    /**
     *
     */
    void updateKnownEntities();

    // default method to handle problems on lua. it will raise an exception if it find some problem
    static int luaThrowException( lua_State* state )
    throw(opencog::RuntimeException);
};

} } // namespace opencog::oac

#endif /*NEWRULEENGINE_H_*/
