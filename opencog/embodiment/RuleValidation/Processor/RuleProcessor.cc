/*
 * opencog/embodiment/RuleValidation/Processor/RuleProcessor.cc
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

#include "RuleProcessor.h"

#include <opencog/embodiment/RuleValidation/VirtualWorldData/XmlLoader.h>

#include <cstdio>
#include <lua.hpp>
#include <luabind/error.hpp>
#include <luabind/class.hpp>
#include <luabind/luabind.hpp>
#include <luabind/function.hpp>
#include <luabind/operator.hpp>

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/mt19937ar.h>

#include <boost/format.hpp>
#include <fstream>

#include <opencog/embodiment/Control/Procedure/ComboInterpreter.h>
#include <opencog/embodiment/Control/Procedure/ComboSelectInterpreter.h>

using namespace Processor;
using namespace opencog;

/* ----------------------------------------------------------------------------
 * Public functions
 * ----------------------------------------------------------------------------
 */

RuleProcessor::RuleProcessor(const std::string & type) :
        comboSelectRepository(comboRepository)
{

    // filenames for stdlib combo, combo preconditions and combo selection
    std::string comboLib = config().get("COMBO_STDLIB_REPOSITORY_FILE");
    std::string comboPre = config().get("COMBO_RULES_PRECONDITIONS_REPOSITORY_FILE");
    std::string comboSel = config().get("COMBO_SELECT_RULES_PRECONDITIONS_REPOSITORY_FILE");
    std::string comboAct = config().get("COMBO_RULES_ACTION_SCHEMATA__REPOSITORY_FILE");

    // ... to load combo scritpts
    loadComboScripts(comboLib, comboPre, comboSel, comboAct);

    // enabling lua
    this->luaState = lua_open( );

    luaopen_base  (luaState);
    luaopen_math  (luaState);
    luaopen_table (luaState);
    luaopen_debug (luaState);
    luaopen_string(luaState);

    luabind::open(luaState);
    luabind::set_pcall_callback(&luaThrowException);

    // register class to makes lua understand is's structure
    // TODO C++0x
    // luabind::module(this->luaState) [
    //     luabind::class_<std::vector<std::string> >( "StringVector" )
    //     .def( luabind::constructor<>() )
    //     .def( "push_back", &std::vector<std::string>::push_back )
    // ];

    luabind::module( this->luaState ) [
        luabind::class_< std::map<std::string, float>::value_type >( "StringFloatPair" )
        .def( luabind::constructor<const std::string&, const float&>() )
    ];

    luabind::module( this->luaState ) [
        luabind::class_< std::pair<std::map<std::string,float>::iterator,bool> >( "StringFloatMapInsertReturn" )
    ];

    luabind::module( this->luaState ) [
        luabind::class_<std::map<std::string, float> >( "StringFloatMap" )
        .def( luabind::constructor<>() )
        .def( "insert", (std::pair<std::map<std::string,float>::iterator,bool> (std::map<std::string,float>::*) (const std::map<std::string,float>::value_type&) ) &std::map<std::string,float>::insert )
    ];

    luabind::module( this->luaState ) [
        luabind::class_<RuleProcessor>( "RuleProcessor" )
        .def( "addRule", &RuleProcessor::addRule )
    ];

    luabind::globals( this->luaState )[ "ruleProcessor" ] = this;

    // load core file
    if ( luaL_dofile( this->luaState, config().get("RV_CORE_FILE").c_str() ) ) {
        luaThrowException( this->luaState );
    }

    std::string agentRules = (boost::format(config().get("RE_RULES_FILENAME_MASK")) % type).str();

    // load rules
    if ( luaL_dofile( this->luaState, agentRules.c_str() ) ) {
        luaThrowException( this->luaState );
    }
}

RuleProcessor::~RuleProcessor()
{
}

void RuleProcessor::evaluateRules(const std::string & filename)
{

    // create the world scenario
    loadWorldState(filename);

    // process all preconditions, suggestion actions, relations and feelings to
    // be executed updated
    evaluatePreconditions();

    // select the final rule to be sent to execution
    std::map<Action, std::vector<float> > weightedActions;
    std::map<Action, std::string> actionsRuleMap;
    std::map<Action, float> maxWeightedActions;

    {
        std::multimap<Action, std::string>::iterator it;
        for (it = this->suggestedActions.begin(); it != this->suggestedActions.end(); ++it ) {
            maxWeightedActions[it->first] = -1.0f;
        }

        // grouping actions...
        for (it = this->suggestedActions.begin(); it != this->suggestedActions.end(); ++it) {

            RuleStrengthIt ruleIt = ruleStrengthMap.find(it->second);

            float strength = 0.0f;
            if (ruleIt != ruleStrengthMap.end()) {
                strength = ruleIt->second[worldState.getPetMode()];
            } else {
                logger().error(
                             "evaluateRules - Found not strength for rule precondition '%s'.",
                             it->second.c_str());
            }

            weightedActions[it->first].push_back(strength);

            // if there are more than one rule that leads to the same action,
            // the selected rule should be the one with higher weight
            if (strength > maxWeightedActions[it->first]) {
                maxWeightedActions[it->first] = strength;
                this->actionsRuleMap[it->first] = it->second;
            }
        }
    } // end block


    { // calculating the strength mean of all suggested actions..
        std::map<Action, std::vector<float> >::iterator it;
        float maximumWeight = -1;

        for ( it = weightedActions.begin( ); it != weightedActions.end( ); ++it ) {

            // mean weight
            float candidateWeight = 0;
            for (unsigned int i = 0; i < it->second.size( ); ++i ) {
                candidateWeight += it->second[ i ];
            }
            candidateWeight /= it->second.size( );

            // take the maximum weighted action
            if ( candidateWeight > maximumWeight ) {
                maximumWeight = candidateWeight;
                this->selectedAction = it->first;
                this->selectedRule   = this->actionsRuleMap[it->first];
            }
        }
    } // end block

    // format the output
    formatOutputResult(filename);
}

/* ----------------------------------------------------------------------------
 * Private functions
 * ----------------------------------------------------------------------------
 */
void RuleProcessor::evaluatePreconditions()
{

    opencog::MT19937RandGen rng(0);
    Procedure::ComboInterpreter comboInterpreter(this->worldState, rng);
    Procedure::ComboSelectInterpreter comboSelectInterpreter(this->worldState, rng);

    std::map<std::string, std::string>::iterator it;
    for (it = this->rulePreconditionMap.begin(); it != this->rulePreconditionMap.end(); it++) {
        std::string ruleName = (*it).first;
        std::string precondition = (*it).second;

        this->validCandidates.clear();
        combo::variable_unifier unifier;
        const std::vector<std::string> & entities =  worldState.getWorldEntities();

        for (unsigned int i = 0; i < entities.size(); i++) {
            unifier.insert(entities[i], true);
        }
        this->validCandidates = entities;

        combo::vertex result;
        std::vector<combo::vertex> arguments;
        Procedure::RunningProcedureId procedureId;

        if (comboRepository.contains(precondition)) {
            const Procedure::ComboProcedure& p = comboRepository.get(precondition);
            procedureId = comboInterpreter.runProcedure(p.getComboTree(), arguments, unifier);

            while (!comboInterpreter.isFinished(procedureId)) {
                comboInterpreter.run(NULL);
            }

            if (comboInterpreter.isFailed(procedureId)) {
                logger().error(
                             "evaluatePreconditions - Combo precondition '%s' failed.",
                             precondition.c_str());
                continue;
            }

            result = comboInterpreter.getResult(procedureId);

        } else if (comboSelectRepository.contains(precondition)) {
            const Procedure::ComboSelectProcedure& p = comboSelectRepository.get(precondition);
            procedureId = comboSelectInterpreter.runProcedure(p.getFirstScript(),
                          p.getSecondScript(),
                          arguments, unifier);

            while (!comboSelectInterpreter.isFinished(procedureId)) {
                comboSelectInterpreter.run(NULL);
            }

            if (comboSelectInterpreter.isFailed(procedureId)) {
                logger().error(
                             "evaluatePreconditions - ComboSelect precondition '%s' failed.",
                             precondition.c_str());
                continue;
            }

            result = comboSelectInterpreter.getResult(procedureId);

        } else {
            throw opencog::RuntimeException(TRACE_INFO,
                                            "evaluatePreconditions - Procedure type should be COMBO or COMBO_SELECT.");
        }

        if (is_builtin(result) && (get_builtin(result) == combo::id::logical_true)) {
            logger().debug(
                         "RuleProcessor - Rule '%s' evaluated true.", ruleName.c_str());

            if (procedureId.getType() == Procedure::COMBO) {
                updateValidCandidates(comboInterpreter.getUnifierResult(procedureId));
            } else if (procedureId.getType() == Procedure::COMBO_SELECT) {
                updateValidCandidates(comboSelectInterpreter.getUnifierResult(procedureId));
            } else {
                throw opencog::RuntimeException(TRACE_INFO,
                                                "evaluatePreconditions -  Procedure type should be COMBO or COMBO_SELECT.");
            }

            switch (this->ruleTypeMap[ruleName]) {
            case 0: { // schema effect
                Action action = boost::get<Action>(this->ruleEffectMap[ruleName]);
                suggestAction(ruleName, action.getName(), action.getParameters());
            }
            break;

            case 1: { // feeling effect
                Feeling feeling = boost::get<Feeling>(this->ruleEffectMap[ruleName]);
                suggestFeeling(ruleName, feeling.getName(), feeling.getIntensity());
            }
            break;

            case 2: { // relation effect
                Relation relation = boost::get<Relation>(this->ruleEffectMap[ruleName]);
                suggestRelation(ruleName, relation.getName(),
                                relation.getTarget(), relation.getIntensity());
            }
            break;

            default:
                continue;
            }
        }
    }
}

int RuleProcessor::luaThrowException( lua_State * state )
{
    lua_Debug debugger;
    lua_getfield( state, LUA_GLOBALSINDEX, "f");  /* get global 'f' */
    lua_getinfo( state, "> Sln ", &debugger );
    std::string error = lua_tostring( state, -1);
    lua_pop( state, 1 );

    std::stringstream message;
    message << debugger.short_src << " [line " << debugger.currentline << "]";

    if ( debugger.name != 0) {
        message << " [" << debugger.namewhat << " " << debugger.name << "]";
    } // if
    message << " [" << error << "]";
    std::cerr << message.str( ) << std::endl;

    throw opencog::RuntimeException( TRACE_INFO, message.str( ).c_str( ) );
}

void RuleProcessor::loadWorldState(const std::string & filename)
{
    VirtualWorldData::XmlLoader loader;

    // populate worldState with the xml files
    if (!loader.fromFile(filename, this->worldState)) {
        throw opencog::RuntimeException(TRACE_INFO,
                                        "RuleProcessor - Unable to load WorldState. Check log.");
    }
}

void RuleProcessor::loadComboScripts(const std::string & comboLib,
                                     const std::string & comboPre,
                                     const std::string & comboSel,
                                     const std::string & comboAct)
{
    std::ifstream fin(comboLib.c_str());
    if (fin.good()) {
        comboRepository.loadFromStream(fin);
    } else {
        logger().warn(
                     "loadComboScripts - Unable to load '%s'.",
                     comboLib.c_str());
    }
    fin.close();

    fin.open(comboPre.c_str());
    if (fin.good()) {
        comboRepository.loadFromStream(fin);
    } else {
        logger().warn(
                     "loadComboScripts - Unable to load '%s'.",
                     comboPre.c_str());
    }
    fin.close();

    fin.open(comboSel.c_str());
    if (fin.good()) {
        comboSelectRepository.loadFromStream(fin);
    } else {
        logger().warn(
                     "loadComboScripts - Unable to load '%s'.",
                     comboSel.c_str());
    }
    fin.close();

    fin.open(comboAct.c_str());
    if (fin.good()) {
        comboSelectRepository.loadFromStream(fin);
    } else {
        logger().warn(
                     "loadComboScripts - Unable to load '%s'.",
                     comboAct.c_str());
    }
    fin.close();
}

void RuleProcessor::addRule(const std::string& rule, const int type,
                            const std::map<std::string, float>& modesStrength,
                            const std::string& precondition,
                            const std::string& effect,
                            const std::vector<std::string>& effectParameters)
{

    // all preconditions should be inserted in ComboRepository and ProcedureRepository as combo
    // scripts with no parameters
    if (!comboRepository.contains(precondition) &&
            !comboSelectRepository.contains(precondition)) {
        // error
        return;
    }

    this->ruleTypeMap[rule] = type;
    this->ruleStrengthMap[rule] = modesStrength;
    this->rulePreconditionMap[rule] = precondition;

    switch (type) {
    case 0: // schema rule
        this->ruleEffectMap[rule] = Processor::RuleProcessor::Action(effect, effectParameters);
        break;

    case 1: // feeling rule
        if (effectParameters.size() != 2) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "addRule - Feeling effect need two arguments. Got '%d'.",
                                                 effectParameters.size());
        }
        this->ruleEffectMap [rule] = Processor::RuleProcessor::Feeling(effectParameters[0],
                                     atof(effectParameters[1].c_str()));
        break;

    case 2: // relation rule
        if (effectParameters.size() != 3) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "addRule - Relation effect need three arguments. Got '%d'.",
                                                 effectParameters.size());
        }
        this->ruleEffectMap[rule] = Processor::RuleProcessor::Relation(effectParameters[0],
                                    effectParameters[1],
                                    atof(effectParameters[2].c_str()));
        break;

    default:
        break;
    }
}

void RuleProcessor::suggestFeeling(const std::string& rule, const std::string& feeling, float intensity)
{
    this->suggestedFeelings.insert(std::pair<Feeling, std::string>(Feeling(feeling, intensity), rule));
}

void RuleProcessor::suggestRelation(const std::string& rule, const std::string& relation,
                                    const std::string& target, float intensity)
{
    if (target == "_*_") {
        std::vector<std::string> targets = this->validCandidates; // make a copy
        for (unsigned int i = 0; i < targets.size(); ++i ) {
            suggestRelation(rule, relation, targets[i], intensity );
        } // for
        return;
    } // if

    this->suggestedRelations.insert(std::pair<Relation, std::string>(
                                        Relation(relation, target, intensity), rule));
}

void RuleProcessor::suggestAction(const std::string& rule,
                                  const std::string& action,
                                  const std::vector<std::string>& parameters)
{

    if (std::find(parameters.begin(), parameters.end(), "_*_") != parameters.end()) {

        std::vector<std::string> validTargets = validCandidates; // make a copy
        for (unsigned int i = 0; i < validTargets.size( ); ++i ) {
            std::vector<std::string> bindParameters = parameters;

            // replace all WILD_CARD elements by valid target element
            std::replace_if(bindParameters.begin(), bindParameters.end(),
                            std::bind2nd(std::equal_to<std::string>(), "_*_"), validTargets[i]);

            suggestAction(rule, action, bindParameters);
        } // for
        return;
    } // if

    this->suggestedActions.insert(std::pair<Action, std::string>(
                                      Action(action, parameters), rule));
}

void RuleProcessor::updateValidCandidates(combo::variable_unifier unifier)
{
    this->validCandidates.clear();

    if (!unifier.isUpdated()) {
        return;
    }

    combo::UnifierIt it;
    for (it = unifier.begin(); it != unifier.end(); it++) {
        if ((*it).second) {
            this->validCandidates.push_back((*it).first);
        }
    }
}

void RuleProcessor::formatOutputResult(const std::string & inputFilename)
{

    fprintf(stdout, "Execution Summary\n");
    fprintf(stdout, "-------------------------------------------------\n");
    fprintf(stdout, "World scenario configuration file: '%s'\n", inputFilename.c_str());
    fprintf(stdout, "Action selected: '%s'\n", selectedAction.toString().c_str());
    fprintf(stdout, "Rule selected: '%s'\n", selectedRule.c_str());
    fprintf(stdout, "\n\n");

    fprintf(stdout, "Suggested actions with rules\n");
    fprintf(stdout, "-------------------------------------------------\n");
    std::multimap<Action, std::string>::iterator actionIt;
    for (actionIt = suggestedActions.begin(); actionIt != suggestedActions.end(); actionIt++) {
        fprintf(stdout, "Action: '%s'\n\tRule: '%s'\n",
                (actionIt->first).toString().c_str(),
                actionIt->second.c_str());
    }
    fprintf(stdout, "\n\n");

    fprintf(stdout, "Suggested relations with rules\n");
    fprintf(stdout, "-------------------------------------------------\n");
    std::multimap<Relation, std::string>::iterator relationIt;
    for (relationIt = suggestedRelations.begin(); relationIt != suggestedRelations.end(); relationIt++) {
        fprintf(stdout, "Relation: '%s'\n\tRule: '%s'\n",
                relationIt->first.toString().c_str(),
                relationIt->second.c_str());
    }
    fprintf(stdout, "\n\n");

    fprintf(stdout, "Suggested feelings with rules\n");
    fprintf(stdout, "-------------------------------------------------\n");
    std::multimap<Feeling, std::string>::iterator feelingIt;
    for (feelingIt = suggestedFeelings.begin(); feelingIt != suggestedFeelings.end(); feelingIt++) {
        fprintf(stdout, "Feeling: '%s'\n\tRule: '%s'\n",
                feelingIt->first.toString().c_str(),
                feelingIt->second.c_str());
    }
    fprintf(stdout, "\n\n");

}

