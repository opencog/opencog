/*
 * opencog/embodiment/RuleValidation/Processor/RuleProcessor.h
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

#ifndef RULE_PROCESSOR_H
#define RULE_PROCESSOR_H

#include <opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldState.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/embodiment/Control/Procedure/ComboProcedureRepository.h>
#include <opencog/embodiment/Control/Procedure/ComboSelectProcedureRepository.h>

#include <map>
#include <set>
#include <vector>
#include <sstream>
#include <boost/variant.hpp>
#include <opencog/comboreduct/combo/variable_unifier.h>

struct lua_State;

namespace Processor
{

class RuleProcessor
{

public:

    typedef std::map<std::string, std::map<std::string, float> > RuleStrength;
    typedef std::map<std::string, std::map<std::string, float> >::iterator RuleStrengthIt;

    // inner classes
    // action to be executed as an effect of a given rule
    class Action : public std::pair<std::string, std::vector<std::string> >
    {

    public:
        inline Action(const std::string& name = "",
                      const std::vector<std::string>& parameters = std::vector<std::string>()) :
                std::pair<std::string, std::vector<std::string> >(name, parameters) { }

        inline const std::string& getName( void ) const {
            return first;
        }
        inline void  setName( const std::string& name ) {
            first = name;
        }

        inline const std::vector<std::string>& getParameters( void ) const {
            return second;
        }
        inline void  setParameters(const std::vector<std::string>& parameters) {
            second = parameters;
        }

        inline const std::string getParameter( unsigned int index ) const {
            if ( index >= second.size( ) ) {
                return "";
            }
            return second[ index ];
        }

        inline const std::string toString() const {
            std::stringstream ss;
            ss << first << "(";
            for (unsigned int i = 0; i < second.size(); i++) {
                ss << second[i];
                if (i != second.size() - 1) {
                    ss << ", ";
                }
            }
            ss << ")";
            return ss.str();
        }
    }; // inner class action

    // relations to be created as an effect of a given rule
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
        inline const float getIntensity( void ) const {
            return intensity;
        }
        inline const std::string& getTarget( void ) const {
            return target;
        }
        inline const std::string toString( void ) const {
            std::stringstream ss;
            ss << name << "(" << target << ", " << intensity << ")";
            return ss.str();
        }

    }; // inner class relation

    // struct used to compare relations (used to sort suggested relations
    // map)
    struct RelationCompare {
        bool operator()(const Relation& a, const Relation& b) const {
            return (a.getName() < b.getName());
        }
    };

    // feelings to be created as an effect by the given rule
    class Feeling : public std::pair<std::string, float>
    {
    private:
        std::string name;
        float intensity;

    public:
        inline Feeling( const std::string& name = "", const float intensity = 0.7) :
                std::pair<std::string, float>(name, intensity) { }

        inline const std::string& getName( void ) const {
            return first;
        }
        inline void  setName( const std::string& name ) {
            first = name;
        }

        inline const float getIntensity( void ) const {
            return second;
        }
        inline void  setIntensity( float intensity ) {
            second = intensity;
        }

        inline std::string toString( void ) const {
            std::stringstream ss;
            ss << first << "(" << second << ")";
            return ss.str();
        }

    }; // inner class feeling

    // rules effect definition
    typedef boost::variant<Action, Relation, Feeling> Effect;

    RuleProcessor(const std::string & type);
    ~RuleProcessor();


    //
    void evaluateRules(const std::string & filename);

private:

    //
    void evaluatePreconditions();

    //
    void formatOutputResult(const std::string & inputFilename);

    /**
     * Load combo stdlib, rules precondition and rules select precondition
     */
    void loadComboScripts(const std::string & comboLib,
                          const std::string & comboPre,
                          const std::string & comboSel,
                          const std::string & comboAct);

    //
    void loadWorldState(const std::string & filename);

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
     * suggestion methods
     *
     * This methods suggests relationships, feelings and actions to be executed on the next cycle.
     * A suggestion will be evaluated and ranked. The better/best ranked itens will be selected.
     */
    void suggestRelation(const std::string& rule,
                         const std::string& relation,
                         const std::string& target,
                         float intensity = 0.7 );

    void suggestFeeling(const std::string& rule,
                        const std::string& feeling,
                        float intensity = 0.7 );

    void suggestAction(const std::string& rule,
                       const std::string& action,
                       const std::vector<std::string>& parameters );

    void updateValidCandidates(combo::variable_unifier unifier);

    //
    static int luaThrowException( lua_State * state );

    //
    lua_State * luaState;

    //
    VirtualWorldData::VirtualWorldState worldState;

    //
    Procedure::ComboProcedureRepository comboRepository;

    //
    Procedure::ComboSelectProcedureRepository comboSelectRepository;

    //
    std::vector<std::string> validCandidates;

    //
    std::map<std::string, int> ruleTypeMap;

    //
    std::map<std::string, Effect> ruleEffectMap;

    //
    std::map<std::string, std::string> rulePreconditionMap;

    //
    RuleStrength ruleStrengthMap;

    //
    std::multimap<Relation, std::string, RelationCompare> suggestedRelations;

    //
    std::multimap<Feeling, std::string> suggestedFeelings;

    //
    std::multimap<Action, std::string> suggestedActions;

    // the final map informing the rules that lead to the key action
    std::map<Action, std::string> actionsRuleMap;

    //
    Action selectedAction;

    //
    std::string selectedRule;
};
}

#endif
