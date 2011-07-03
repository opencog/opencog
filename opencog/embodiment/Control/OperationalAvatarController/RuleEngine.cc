/*
 * opencog/embodiment/Control/OperationalAvatarController/RuleEngine.cc
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

#include <lua.hpp>
#include <luabind/error.hpp>
#include <luabind/class.hpp>
#include <luabind/luabind.hpp>
#include <luabind/function.hpp>
#include <luabind/operator.hpp>

#include <cstdlib>
#include <ctime>

#include <opencog/util/numeric.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/random.h>
#include <opencog/util/StringManipulator.h>

#include "RuleEngine.h"
#include "RuleEngineUtil.h"
#include "RuleEngineLearnedTricksHandler.h"
#include "SchemaRunner.h"

#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/CompositeTruthValue.h>

#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include "OAC.h"
#include <opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h>

#include <opencog/comboreduct/combo/vertex.h>

#include <boost/regex.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/numeric/conversion/cast.hpp>  

#include <stdlib.h>

#define MORE_DEBUG_INFO 1

#define WILD_CARD_STR "_*_"

using namespace opencog;
using namespace oac;
using namespace combo;

RuleEngine::RuleEngine( OAC* opc, const std::string& petName )
throw( RuntimeException ) :
        opc(opc), petName( petName ), cycle(0),
        lastPetActionDone("", std::vector<std::string>()),
        currentAction("", std::vector<std::string>()),
        candidateAction( "", std::vector<std::string>()),
        currentRule(""), candidateRule(""),
        lastRequestedCommandCycles(0), avatarAskedToTry(false),
        groupingMode(false), groupLeaderId("")
{

    this->petHandle = AtomSpaceUtil::getAgentHandle( *(opc->getAtomSpace()),
                      petName );
    rng = new MT19937RandGen(0);
    if ( petHandle == Handle::UNDEFINED ) {
        throw RuntimeException( TRACE_INFO,
                                ( "There is no pet named '"
                                  + petName
                                  + "' inside AtomSpace" ).c_str() );
    } // if
    
    this->util = new RuleEngineUtil( this );
    this->schemaRunner = new SchemaRunner( opc );
    this->learnedTricksHandler = new RuleEngineLearnedTricksHandler( opc );
    this->cyclesDuringAgentLastAction = config().get_int("RE_CYCLES_DURING_AGENT_LAST_ACTION");
    
    this->longTermAttentionValue = AttentionValue::factory(AttentionValue::DEFAULTATOMSTI, 1);
    this->defaultTruthValue = new SimpleTruthValue(config().get_double("RE_DEFAULT_MEAN"), config().get_double("RE_DEFAULT_COUNT"));

    this->currentActionRepetitions = 0;
    updateCurrentActionRepetitions();

    // precompute gaussian distribution used to reward/punish pet behavior
    preCompGaussianDistributionVector(config().get_double("RL_TIME_WINDOW"),
                                      config().get_double("RL_GAUSSIAN_MEAN"),
                                      config().get_double("RL_GAUSSIAN_STD_DEVIATION"));
                                      

    // enabling lua language - load rules from file
    this->luaState = lua_open( );

    luaopen_base(this->luaState);
    luaopen_math(this->luaState);
    luaopen_table(this->luaState);
    luaopen_debug(this->luaState);
    luaopen_string(this->luaState);

    luabind::open( this->luaState );
    luabind::set_pcall_callback( &luaThrowException );

    // register class to makes lua understand is's structure
    luabind::module( this->luaState ) [
        luabind::class_<std::vector<std::string> >( "StringVector" )
        .def( luabind::constructor<>() )
        .def( "push_back", &std::vector<std::string>::push_back )
    ];

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
        luabind::class_<RuleEngine>( "RuleEngine" )
        .def( "addRule", &RuleEngine::addRule )
    ];

    luabind::globals( this->luaState )[ "ruleEngine" ] = this;

    // load core file
    logger().debug(
                 ( "RuleEngine - loading core file: "
                   + config().get("RE_CORE_FILE") ).c_str( ) );
    if (luaL_dofile(this->luaState, config().get("RE_CORE_FILE").c_str())) {
        luaThrowException( this->luaState );
    } // if

    std::string agentRules = (boost::format(config().get("RE_RULES_FILENAME_MASK")) % this->opc->getPet().getType()).str();

    // load rules
    logger().debug(
                 ("RuleEngine - loading rules file: " + agentRules).c_str());
    if ( luaL_dofile( this->luaState, agentRules.c_str() ) ) {
        luaThrowException( this->luaState );
    } // if

    // TODO change this loading, get from somewhere else (opc, pet)
    this->feelings.push_back(std::string("happiness"));
    this->feelings.push_back(std::string("fear"));
    this->feelings.push_back(std::string("pride"));
    this->feelings.push_back(std::string("love"));
    this->feelings.push_back(std::string("hate"));
    this->feelings.push_back(std::string("anger"));
    this->feelings.push_back(std::string("gratitude"));
    this->feelings.push_back(std::string("excitement"));

    // TODO Remove this code, probabily when removing all built-in schemas.
    // IMPORTANT: do not make confusion with combo buit-ins
    addSchemaNode("GRAB");
    addSchemaNode("JUMP");
    addSchemaNode("DROP");
    addSchemaNode("SNIFF");
    addSchemaNode("BARK");
    addSchemaNode("LOOK_UP_TURN_HEAD");
    addSchemaNode("BARETEETH");
    addSchemaNode("WAGTAIL");
    addSchemaNode("STRETCH");
    addSchemaNode("SIT");
    addSchemaNode("BEG");
    addSchemaNode("HEEL");
    addSchemaNode("PAYATTENTION");
    addSchemaNode("GRAB_NEAREST");
}

RuleEngine::~RuleEngine( )
{
    delete(longTermAttentionValue);
    delete(defaultTruthValue);
    delete(rng);
}

/* ----------------------------------------------------------------------------
 * getters and setters
 * ----------------------------------------------------------------------------
 */
void RuleEngine::setPredicateValue( const std::string& name, float value)
{
    SimpleTruthValue truthValue( value, 1.0 );
    AtomSpaceUtil::setPredicateValue( *(opc->getAtomSpace()), name,
                                      truthValue, this->petHandle );
}

const std::string& RuleEngine::getCurrentAction( void ) const
{
    return currentAction.getName( );
}

const std::string& RuleEngine::getCurrentRule( void ) const
{
    return currentRule;
}

const std::string& RuleEngine::getNextAction( void )
{
    processNextAction( );
    return getCurrentAction( );
}

/* -----------------------------------------------------------------------
 * lua functions
 * -----------------------------------------------------------------------
 */
int RuleEngine::luaThrowException( lua_State* state ) throw(RuntimeException)
{
    lua_Debug debugger;
    lua_getfield( state, LUA_GLOBALSINDEX, "f");  /* get global 'f' */
    lua_getinfo( state, ">Sln", &debugger );
    std::string error = lua_tostring( state, -1);
    lua_pop( state, 1 );

    std::stringstream message;
    message << debugger.short_src << " [line " << debugger.currentline << "]";

    if ( debugger.name != 0) {
        message << " [" << debugger.namewhat << " " << debugger.name << "]";
    } // if
    message << " [" << error << "]";
    std::cerr << message.str( ) << std::endl;

    throw RuntimeException( TRACE_INFO, message.str( ).c_str( ) );
}

/* -----------------------------------------------------------------------
 * Auxiliar functions
 * -----------------------------------------------------------------------
 */
void RuleEngine::preCompGaussianDistributionVector(float window,
        float mean,
        float stdDeviation)
{
    logger().debug(
                 "RuleEngine - window '%f' mean '%f' std '%f'.",
                 window, mean, stdDeviation);

    // a twenty standard deviation vector
    float slots = ceil( 20.0 * stdDeviation);
    float timePerSlot = window / slots;
    logger().debug(
                 "RuleEngine - Gaussian vector slots '%.3f'"
                 " timePerSlot '%.3f'.",
                 slots, timePerSlot);

    float x = (0.0);
    float c = (2 * stdDeviation * stdDeviation);
    //float a = (1.0 / (stdDeviation * (sqrt(2 * PI))));

    for (int i = 0 ; i < (int)slots; i++) {
        float b = x - mean;
        //this->gaussianVector.push_back(a * exp(-((b * b) / c)));
        this->gaussianVector.push_back(exp(-((b * b) / c)));
        x += timePerSlot;
    }
}

void RuleEngine::addSchemaNode(const std::string& schemaName)
{
    Handle result = AtomSpaceUtil::addNode(*(opc->getAtomSpace()),
                                           GROUNDED_SCHEMA_NODE,
                                           schemaName, true);

    opc->getAtomSpace()->setAV(result, *longTermAttentionValue);
    opc->getAtomSpace()->setTV(result, *defaultTruthValue);

    logger().debug(
                 "addSchemaNode - Added GROUNDED_SCHEMA_NODE '%s'.",
                 schemaName.c_str());
}

void RuleEngine::addLearnedSchema( const std::string& schemaName )
{
    this->learnedTricksHandler->addLearnedSchema( schemaName );
}

const std::vector<std::string>& RuleEngine::getValidTargets( void )
{

    if ( this->lastInspectedRuleName != this-> currentInspectedRuleName ) {

        this->varBindCandidates.clear( );
        this->lastInspectedRuleName = this->currentInspectedRuleName;

        Id_EntityPerception_Map_Const_It it;
        for ( it = this->objects.begin( ); it != this->objects.end( ); ++it ) {
            this->varBindCandidates.push_back( it->first );
        } // for

        for ( it = this->avatars.begin( ); it != this->avatars.end( ); ++it ) {
            this->varBindCandidates.push_back( it->first );
        } // for
    } // if

    return this->varBindCandidates;
}

void RuleEngine::updateValidTargets(variable_unifier& unifier)
{
    this->varBindCandidates.clear();

    if (!unifier.isUpdated()) {
        return;
    }

    UnifierIt it;
    for (it = unifier.begin(); it != unifier.end(); it++) {
        if (it->second) {
            this->varBindCandidates.push_back((*it).first);

            logger().debug(
                         "RuleEngine - Unified entity '%s'.",
                         (it->first).c_str());
        }
    }
}

void RuleEngine::updateKnownEntities()
{

    // update objects, avatars
    const SpaceServer::SpaceMap& spaceMap =
        this->opc->getAtomSpace()->getSpaceServer().getLatestMap();

    std::vector<std::string> entities;
    spaceMap.findAllEntities(back_inserter(entities));

    foreach(std::string entity, entities) {
        logger().debug(
                     "RuleEngine - Inspecting entity '%s'",
                     entity.c_str());

        std::vector<Handle> objectHandle;
        opc->getAtomSpace()->getHandleSet(back_inserter(objectHandle),
                                          OBJECT_NODE, entity, true);

        if (objectHandle.size() != 1) {
            logger().debug(
                         "RuleEngine - There is no entity '%s' registered"
                         " on atomspace (# of elements found: '%d')",
                         entity.c_str( ), objectHandle.size() );
            continue;
        } // if

        Handle entityHandle = objectHandle[0];
        if (entityHandle == petHandle) {
            continue;
        }// if

        bool isNextEntity = AtomSpaceUtil::isPredicateTrue(*(opc->getAtomSpace()),
                            "next",
                            entityHandle,
                            petHandle);
        logger().debug(
                     ("RuleEngine - is '" + entity + "' next '"
                      + this->petName + "': "
                      + ((isNextEntity) ? "true" : "false")).c_str());

        Type t = opc->getAtomSpace()->getType(entityHandle);


        if (isNextEntity) {
            if (t == AVATAR_NODE) {
                if (avatars.find(entity) == avatars.end()) {
                    Id_EntityPerception_Pair
                    iep(entity, EntityPerception(cycle));
                    avatars.insert(iep);
                } // if
                avatars[entity].setLastSeenCycle(cycle);
            } else if (t == PET_NODE
                       || t == OBJECT_NODE
                       || t == ACCESSORY_NODE) { // STRUCTURE_NODE
                if (objects.find(entity) == objects.end()) {
                    Id_EntityPerception_Pair
                    iep(entity, EntityPerception(cycle));
                    objects.insert(iep);
                } // if
                objects[entity].setLastSeenCycle(cycle);
            }
        }
    } // foreach

    // update has_novelty predicate
    if (this->util->isNovelty()) {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "has_novelty",
                                         SimpleTruthValue(1.0f, 0.0f),
                                         this->petHandle);
    } else {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "has_novelty",
                                         SimpleTruthValue(0.0f, 0.0f),
                                         this->petHandle);
    }

    // update has_learned_tricks predicate
    if (this->learnedTricksHandler->hasLearnedTricks()) {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "has_learned_tricks",
                                         SimpleTruthValue(1.0f, 0.0f),
                                         this->petHandle);
    } else {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "has_learned_tricks",
                                         SimpleTruthValue(0.0f, 0.0f),
                                         this->petHandle);
    }

    // update requested schema predicates, that is, if the owner has requested a schema to be executed
    if (this->util->isThereARequestedSchema()) {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "has_requested_schema",
                                         SimpleTruthValue(1.0f, 0.0f),
                                         this->petHandle);
    } else {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "has_requested_schema",
                                         SimpleTruthValue(0.0f, 0.0f),
                                         this->petHandle);
    }

    switch (this->opc->getPet().getMode()) {
    case PLAYING:
    case SCAVENGER_HUNT:
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "is_learning",
                                         SimpleTruthValue(0.0f, 0.0f),
                                         this->petHandle);
        break;
    case LEARNING:
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "is_learning",
                                         SimpleTruthValue(1.0f, 0.0f),
                                         this->petHandle);
        break;
    }

    if (this->avatarAskedToTry) {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "avatar_asked_to_try",
                                         SimpleTruthValue(1.0f, 0.0f),
                                         this->petHandle);
        ++this->numberOfCyclesSinceAvatarAskToTry;

    } else {
        AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                         "avatar_asked_to_try",
                                         SimpleTruthValue(0.0f, 0.0f),
                                         this->petHandle);
    }
}

/* ----------------------------------------------------------------------------
 * Feelings functions
 * ----------------------------------------------------------------------------
 */
void RuleEngine::suggestFeeling( const std::string& feeling, float intensity )
{
    suggestedFeelings.insert(Feeling(feeling, intensity));
}

/* ----------------------------------------------------------------------------
 * Actions functions
 * ----------------------------------------------------------------------------
 */
void RuleEngine::suggestAction( const std::string& rule,
                                const std::string& action,
                                const std::vector<std::string>& parameters )
{

    if (std::find( parameters.begin(), parameters.end(), WILD_CARD_STR)
            != parameters.end()) {

        const std::vector<std::string>& validTargets = getValidTargets( );
        unsigned int i;
        for ( i = 0; i < validTargets.size( ); ++i ) {
            std::vector<std::string> bindParameters = parameters;

            // replace all WILD_CARD elements by valid target element
            std::replace_if(bindParameters.begin(), bindParameters.end(),
                            std::bind2nd(std::equal_to<std::string>(),
                                         WILD_CARD_STR),
                            validTargets[i]);

            suggestAction( rule, action, bindParameters );
        } // for
        return;
    } // if

    std::string finalAction = action;
    std::vector<std::string> finalParameters = parameters;

    //Id_EntityPerception_Map_It it; //Nil : apparently it is not used at all

    // handle physiological needs functions
    if ( action == "try" ) {
        finalAction = this->triedSchema;

    } else if ( action == "executeRequestedAction" ) {
        unsigned int procArity;
        finalAction = this->opc->getPet( ).getLatestRequestedCommand( ).name;
        logger().debug( "RuleEngine::%s - Handling requested action %s ", __FUNCTION__,  finalAction.c_str() );
        
        procArity = opc->getProcedureRepository().get(finalAction).getArity();

        if (procArity == this->opc->getPet().getLatestRequestedCommand().arguments.size()) {
            std::vector<std::string>& arguments =
                this->opc->getPet( ).getLatestRequestedCommand( ).arguments;
            finalParameters.resize( arguments.size() );
            std::copy( arguments.begin( ), arguments.end( ), finalParameters.begin( ) );
        } else {
            logger().error( "RuleEngine - An invalid list of arguments was sent to the action '%s'. %d arguments was sent but should be %d",
                            finalAction.c_str(), procArity, this->opc->getPet().getLatestRequestedCommand().arguments.size() );
        }
    } else if ( action == "runRandomTopFiveLearnedTrick" ) {
        std::set<std::string> arguments;
        this->learnedTricksHandler->selectLearnedTrick( finalAction, arguments );

        std::copy( arguments.begin( ), arguments.end( ), finalParameters.begin( ) );
    } // else if

    logger().debug(
                 "RuleEngine - Rule '%s' action '%s' finalAction '%s'",
                 rule.c_str(), action.c_str(), finalAction.c_str());

    // get rule -> action implication link strength
    float strength = AtomSpaceUtil::getRuleImplicationLinkStrength(*(this->opc->getAtomSpace()), rule, this->opc->getPet( ).getCurrentModeHandler( ).getModeName( ) );
    suggestedActions.insert(std::pair<Action, RuleEngine::RuleStrPair>(Action(finalAction,
                            finalParameters),
                            RuleEngine::RuleStrPair(rule, strength)));
}

/* ----------------------------------------------------------------------------
 * Relations functions
 * ----------------------------------------------------------------------------
 */
void RuleEngine::suggestRelation( const std::string& relation,
                                  const std::string& target, float intensity )
{
    if (target == WILD_CARD_STR) {
        const std::vector<std::string>& validTargets = getValidTargets( );
        unsigned int i;
        for ( i = 0; i < validTargets.size( ); ++i ) {
            suggestRelation( relation, validTargets[ i ], intensity );
        } // for
        return;
    } // if

    suggestedRelations.insert(Relation(relation, target, intensity));
}

void RuleEngine::addRelation(const Relation& relation)
{

    // verify if the target of relation is a valid object or avatar
    HandleSeq objHandle;
    this->opc->getAtomSpace()->getHandleSet(back_inserter(objHandle),
                                            OBJECT_NODE,
                                            relation.getTarget(), true);

    // found no handle - ERROR
    if ( objHandle.size( ) < 1 ) {
        logger().error(
                     "RuleEngine - Found no Handle for SpaceMap object %s.",
                     relation.getTarget( ).c_str( ) );
        return;

        // found more than one handle - WARNING, return the first one
    } else if (objHandle.size() > 1) {
        logger().error(
                     "RuleEngine - Found more than one Handle for SpaceMap"
                     " object '%s'. Using the first one.",
                     relation.getTarget( ).c_str( ));
    } // else if

    Handle targetHandle = objHandle[0];
    if ( targetHandle == Handle::UNDEFINED ) {
        logger().error(
                     "RuleEngine - attempted to add a relation to"
                     " an avatar/object that not exists");
        return;
    } // if

    // yes! the target is a valid object, so setup relation with it
    SimpleTruthValue truthValue(relation.getIntensity(), 1.0 );
    AtomSpaceUtil::setPredicateValue(*(opc->getAtomSpace()), relation.getName(),
                                     truthValue, this->petHandle,
                                     targetHandle);

    // add that relation name in relationNameSet
    relationNameSet.insert(relation.getName());

    logger().debug(
                 "RuleEngine - Relation '%s' added to target '%s'"
                 " with intensity '%f'.",
                 relation.getName( ).c_str( ),
                 relation.getTarget( ).c_str( ), relation.getIntensity() );
}

void RuleEngine::logRelations(Logger::Level l) 
{
    // it looks at the existing relating in the atomSpace
    // log them at level l
    const AtomSpace& as = *(opc->getAtomSpace());

    logger().log(l, "RuleEngine - TruthValue relations at RuleEngine cycle %d", getCycle());

    std::list<Handle> ret;
    as.getHandleSet(back_inserter(ret), EVALUATION_LINK, false);
    for (std::list<Handle>::const_iterator h_it = ret.begin();
            h_it != ret.end(); ++h_it) {

        //get predicate name
        const string& pred_name = as.getName(as.getOutgoing(*h_it, 0));
        std::set<string>::iterator r_it = relationNameSet.find(pred_name);
        TruthValuePtr tv = as.getTV(*h_it);
        if (r_it != relationNameSet.end() && tv->getMean() != 0.0) {

            //get the predicate arguments
            Handle list_h = as.getOutgoing(*h_it, 1);
            const HandleSeq& args_hs = as.getOutgoing(list_h);
            string message_str("RuleEngine - TruthValue relation '");
            message_str += pred_name + string("(");

            // add arguments in message_str
            for (HandleSeq::const_iterator hs_it = args_hs.begin();
                    hs_it != args_hs.end();) {
                message_str += as.getName(*hs_it);
                ++hs_it;
                if (hs_it != args_hs.end())
                    message_str += string(", ");
            }
            message_str += string(") ");
            message_str += tv->toString();

            // log the message
            logger().log(l, message_str);
        }
    }
}


void RuleEngine::removeRelation(const Relation& relation)
{

    Handle relationHandle = opc->getAtomSpace()->getHandle(PREDICATE_NODE,
                            relation.getName() );
    if ( relationHandle == Handle::UNDEFINED ) {
        logger().warn(
                     "RuleEngine - attempted to remove a relation"
                     " that not exists");
        return;
    } // if

    // verify if the target of relation is a valid object or avatar
    HandleSeq objHandle;
    this->opc->getAtomSpace()->getHandleSet(back_inserter(objHandle),
                                            OBJECT_NODE,
                                            relation.getTarget(), true);

    // found no handle - ERROR
    if ( objHandle.size( ) < 1 ) {
        logger().error(
                     "RuleEngine - Found no Handle for SpaceMap object %s.",
                     relation.getTarget( ).c_str( ) );
        return;

        // found more than one handle - WARNING, return the first one
    } else if (objHandle.size() > 1) {
        logger().error(
                     "RuleEngine - Found more than one Handle for"
                     " SpaceMap object '%s'. Using the first one.",
                     relation.getTarget( ).c_str( ));
    } // else if

    Handle targetHandle = objHandle[0];
    if ( targetHandle == Handle::UNDEFINED ) {
        logger().error(
                     "RuleEngine - attempted to add a relation to"
                     " an avatar/object that not exists");
        return;
    } // if

    SimpleTruthValue truthValue( 0.0f, 1.0 );
    AtomSpaceUtil::setPredicateValue(*(opc->getAtomSpace()),
                                     relation.getName(),
                                     truthValue, this->petHandle,
                                     targetHandle );

    logger().debug(
                 "RuleEngine - relation removed '%s' target '%s'",
                 relation.getName().c_str(), relation.getTarget( ).c_str());
}

void RuleEngine::removeOppositeRelation( const Relation& relation )
{
    std::vector<string> opposite;
    if ( relation.getName( ) == "friend" ) {
        opposite.push_back( "enemy" );
    } else if ( relation.getName( ) == "enemy" ) {
        opposite.push_back( "friend" );
    } else if ( relation.getName( ) == "anger" ) {
        opposite.push_back( "gratitude" );
    } else if ( relation.getName( ) == "gratitude" ) {
        opposite.push_back( "anger" );
    } else if ( relation.getName() == "know" ) {
        opposite.push_back( "curious_about" );
        opposite.push_back( "familiar_with" );
    } else {
        return;
    } // else

    unsigned i;
    for ( i = 0; i < opposite.size( ); ++i ) {
        Relation r(opposite[i], relation.getTarget(), 0.0);
        removeRelation(r);
    } // for
}

/* ----------------------------------------------------------------------------
 * Rule management functions
 * ----------------------------------------------------------------------------
 */
void RuleEngine::addRule(const std::string& rule, const int type,
                         const std::map<std::string, float>& modesStrength,
                         const std::string& precondition,
                         const std::string& effect,
                         const std::vector<std::string>& effectParameters)
{

    // all preconditions should be inserted in ProcedureRepository as combo scripts with
    // no parameters
    if (!this->opc->getProcedureRepository().contains(precondition)) {
        throw InvalidParamException(TRACE_INFO, "RuleEngine - Precondition '%s' not found in system ProcedureRepository.", precondition.c_str());
    }

    this->ruleTypeMap[rule] = type;
    this->rulePreconditionMap[rule] = precondition;

    switch (type) {
    case 0: // schema rule
        this->ruleEffectMap[rule] = Action(effect, effectParameters);
        break;

    case 1: // feeling rule
        if (effectParameters.size() != 2) {
            throw InvalidParamException(TRACE_INFO, "RuleEngine - Feeling effect need two arguments. Got '%d'.", effectParameters.size());
        }
        this->ruleEffectMap[rule] = Feeling(effectParameters[0],
                                            atof(effectParameters[1].c_str()));
        break;

    case 2: // relation rule
        if (effectParameters.size() != 3) {
            throw InvalidParamException(TRACE_INFO, "RuleEngine - Relation effect need three arguments. Got '%d'.", effectParameters.size());
        }
        this->ruleEffectMap[rule] = Relation(effectParameters[0],
                                             effectParameters[1],
                                             atof(effectParameters[2].c_str()));
        break;

    default:
        break;
    }


    addRuleToAtomSpace(rule, modesStrength,
                       precondition, effect, effectParameters);
}

void RuleEngine::addRuleToAtomSpace(const std::string& rule,
                                    const std::map<std::string, float>& modesStrength,
                                    const std::string& precondition,
                                    const std::string& effect,
                                    const std::vector<std::string>& effectParameters)
{

    AtomSpace& atomSpace = *(opc->getAtomSpace());

    Handle preconditionEvalLink = addPreconditionEvalLink(precondition);
    Handle schemaExecLink = addEffectExecLink(effect, effectParameters, true);

    CompositeTruthValue truthValue(TruthValue::DEFAULT_TV(),
                                   NULL_VERSION_HANDLE);
    std::map<std::string, float>::const_iterator it;
    for ( it = modesStrength.begin( ); it != modesStrength.end( ); ++it ) {
        Handle agentModeNode = AtomSpaceUtil::addNode(atomSpace,
                               CONCEPT_NODE,
                               it->first, true);
        VersionHandle versionHandle( CONTEXTUAL, agentModeNode );
        SimpleTruthValue tv( it->second, 1.0f );
        truthValue.setVersionedTV( tv, versionHandle );
    } // for

    HandleSeq list;
    list.push_back(preconditionEvalLink);
    list.push_back(schemaExecLink);

    Handle implicationLink = AtomSpaceUtil::addLink(atomSpace,
                             IMPLICATION_LINK,
                             list, true);
    atomSpace.setAV(implicationLink, *longTermAttentionValue);
    atomSpace.setTV(implicationLink, truthValue);

    Handle rulePhraseNode = AtomSpaceUtil::addNode(atomSpace,
                            PHRASE_NODE,
                            rule, true);
    atomSpace.setAV(rulePhraseNode, *longTermAttentionValue);
    atomSpace.setTV(rulePhraseNode, *defaultTruthValue);

    list.clear();
    list.push_back(rulePhraseNode);
    list.push_back(implicationLink);
    Handle referenceLink = AtomSpaceUtil::addLink(atomSpace,
                           REFERENCE_LINK,
                           list, true);
    atomSpace.setAV(referenceLink, *longTermAttentionValue);
    atomSpace.setTV(referenceLink, *defaultTruthValue);
}

Handle RuleEngine::addEffectExecLink(const std::string& effect,
                                     const std::vector<std::string> parameters,
                                     bool permanent)
throw (RuntimeException)
{

    AtomSpace& atomSpace = *(opc->getAtomSpace());

    HandleSeq list;
    foreach( std::string param, parameters ) {

        if (param == " ") {
            continue;

        } else if (param == WILD_CARD_STR) {
            Handle variableNode = AtomSpaceUtil::addNode(atomSpace,
                                  VARIABLE_NODE,
                                  "VariableNode",
                                  permanent);
            atomSpace.setAV(variableNode, *longTermAttentionValue);
            atomSpace.setTV(variableNode, *defaultTruthValue);

            list.push_back(variableNode);

        } else if (param == "self") {
            list.push_back(petHandle);

        } else if (param == "owner") {

            Handle ownerHandle = atomSpace.getHandle(AVATAR_NODE,
                                 opc->getPet().getOwnerId());
            if (ownerHandle == Handle::UNDEFINED) {
                throw RuntimeException(TRACE_INFO, "addSchemaExecLink - Found no SlAvatarNode for pet owner '%s'.",
                                                opc->getPet().getOwnerId().c_str());
            }

            list.push_back(ownerHandle);

        } else if (StringManipulator::isNumber(param)) {

            Handle numberNode = AtomSpaceUtil::addNode(atomSpace,
                                NUMBER_NODE,
                                param, permanent);
            atomSpace.setAV(numberNode, *longTermAttentionValue);
            atomSpace.setTV(numberNode, *defaultTruthValue);

            list.push_back(numberNode);

        } else {

            size_t openParentesis = param.find('(');

            // check if the parameter is a new function
            if (openParentesis != std::string::npos) {

                size_t closeParentesis = param.rfind(')');
                if (closeParentesis == std::string::npos) {
                    throw RuntimeException(TRACE_INFO, "addSchemaExecLink - Unmatch parentesis in effect '%s' parameter '%s'.",
                                                    effect.c_str(), param.c_str());
                }

                std::string subSchema;
                subSchema = param.substr(0, openParentesis);

                std::string subSchemaStringParam;
                if (closeParentesis - openParentesis > 1) {
                    subSchemaStringParam = param.substr(openParentesis + 1, (closeParentesis - openParentesis - 1));

                    std::vector<std::string> subSchemaParam;
                    boost::algorithm::split_regex(subSchemaParam,
                                                  subSchemaStringParam,
                                                  boost::regex("\\s*,\\s*"));

                    // TODO Need to know if every sub function is realy
                    // an ExectuionLink, if not how to differentiate them
                    Handle execLink = addEffectExecLink(subSchema,
                                                        subSchemaParam,
                                                        permanent);
                    list.push_back(execLink);
                }


                // simple word node
            } else {

                Handle wordNode = AtomSpaceUtil::addNode(atomSpace,
                                  WORD_NODE,
                                  param,
                                  permanent);
                atomSpace.setAV(wordNode, *longTermAttentionValue);
                atomSpace.setTV(wordNode, *defaultTruthValue);

                list.push_back(wordNode);
            }
        }
    }

    Handle listLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK,
                      list, permanent);
    atomSpace.setAV(listLink, *longTermAttentionValue);
    atomSpace.setTV(listLink, *defaultTruthValue);

    Handle schemaNode = AtomSpaceUtil::addNode(atomSpace,
                        GROUNDED_SCHEMA_NODE,
                        effect, permanent);

    list.clear();
    list.push_back(schemaNode);
    list.push_back(listLink);
    Handle executionLink = AtomSpaceUtil::addLink(atomSpace,
                           EXECUTION_LINK,
                           list, permanent);
    atomSpace.setAV(executionLink, *longTermAttentionValue);
    atomSpace.setTV(executionLink, *defaultTruthValue);

    return executionLink;
}

Handle RuleEngine::addPreconditionEvalLink(const std::string& precondition)
{
    AtomSpace& atomSpace = *(opc->getAtomSpace());

    logger().debug(
                 "addPreconditionEvalLink - preCond %s."
                 " GROUNDER_PREDICATE_NODE",
                 precondition.c_str());
    Handle preconditionNode = AtomSpaceUtil::addNode(atomSpace,
                              GROUNDED_PREDICATE_NODE,
                              precondition, true);
    atomSpace.setAV(preconditionNode, *longTermAttentionValue);
    atomSpace.setTV(preconditionNode, *defaultTruthValue);

    HandleSeq list;
    Handle emptyListLink = AtomSpaceUtil::addLink(atomSpace,
                           LIST_LINK,
                           list, true);
    atomSpace.setAV(emptyListLink, *longTermAttentionValue);
    atomSpace.setTV(emptyListLink, *defaultTruthValue);

    list.clear();
    list.push_back(preconditionNode);
    list.push_back(emptyListLink);
    Handle evaluationLink = AtomSpaceUtil::addLink(atomSpace,
                            EVALUATION_LINK,
                            list, true);
    atomSpace.setAV(evaluationLink, *longTermAttentionValue);
    atomSpace.setTV(evaluationLink, *defaultTruthValue);

    return evaluationLink;
}

/* ----------------------------------------------------------------------------
 * Schema functions
 * ----------------------------------------------------------------------------
 */
void RuleEngine::tryExecuteSchema( const std::string& schemaName )
{
    this->avatarAskedToTry = true;
    this->triedSchema = schemaName;

    // reset cycles counter when avatar ask to try again
    this->numberOfCyclesSinceAvatarAskToTry = 0;
}

void RuleEngine::updateCurrentActionRepetitions()
{
    AtomSpace& atomSpace = *(opc->getAtomSpace());

    Handle handle = AtomSpaceUtil::addNode(atomSpace,
                                           CONCEPT_NODE,
                                           "currentActionRepetition", true);

    HandleSeq result;
    atomSpace.getHandleSet(back_inserter(result), handle,
                           FREQUENCY_LINK, false);

    Handle oldHandle = Handle::UNDEFINED;
    // find and remove old info
    foreach(Handle h, result) {
        if (atomSpace.getArity(h) == 2) {
            Handle node = atomSpace.getOutgoing(h, 0);
            if (atomSpace.getType(node) == CONCEPT_NODE &&
                    atomSpace.getName(node) == "currentActionRepetition") {
                oldHandle = h;
            }
        }
    }
    if (oldHandle == Handle::UNDEFINED) {
        logger().info(
                     "RuleEngine - No FREQUECY_LINK for"
                     " \"currentActionRepetition\" found."
                     " Creating it for the first time...");
    } else if (!atomSpace.removeAtom(oldHandle)) {
        logger().error(
                     "RuleEngine - Unable to remove old FREQUECY_LINK: %s",
                     atomSpace.atomAsString(oldHandle).c_str());
    }

    // add new one
    HandleSeq freqLink;
    freqLink.push_back(handle);
    freqLink.push_back(AtomSpaceUtil::addNode(atomSpace,
                       NUMBER_NODE,
                       toString(this->currentActionRepetitions)));

    AtomSpaceUtil::addLink(atomSpace, FREQUENCY_LINK, freqLink, true);
}

void RuleEngine::addSchemaDoneOrFailurePred(const std::string& schema,
        const std::vector<std::string> parameters,
        bool result)
{
    if (schema == "") {
        return;
    }
    AtomSpace& atomSpace = *(opc->getAtomSpace());

    Handle execLink = addEffectExecLink(schema, parameters, false);

    Handle predicateNode = AtomSpaceUtil::addNode(atomSpace,
                           PREDICATE_NODE,
                           result ? "SchemaDone" : "SchemaFailed");

    HandleSeq listLinkOutgoing;
    listLinkOutgoing.push_back(execLink);
    Handle listLink = AtomSpaceUtil::addLink(atomSpace, LIST_LINK,
                      listLinkOutgoing);

    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(predicateNode);
    evalLinkOutgoing.push_back(listLink);

    Handle evalLink = AtomSpaceUtil::addLink(atomSpace, EVALUATION_LINK,
                      evalLinkOutgoing);
    Handle atTimeLink = atomSpace.getTimeServer().addTimeInfo(evalLink, this->opc->getPAI().getLatestSimWorldTimestamp());
    AtomSpaceUtil::updateLatestSchemaPredicate(atomSpace, atTimeLink,
            predicateNode);


    logger().debug(
                 "RuleEngine - Added '%s' pred for schema"
                 " '%s' at timestamp '%lu'.",
                 (result ? "schemaDone" : "schemaFailure"),
                 schema.c_str(),
                 this->opc->getPAI().getLatestSimWorldTimestamp());
}

void RuleEngine::updateSchemaExecutionStatus()
{

    if (schemaRunner->isSchemaExecFinished()) {

        if ( this->lastPetActionDone == this->currentAction ) {
            ++this->currentActionRepetitions;
        } else {
            this->currentActionRepetitions = 0;
        } // else

        this->lastPetActionDone = this->currentAction;
        this->currentAction.setName("");
        this->currentAction.setParameters(std::vector<std::string>());

        // update atomspace representation of the current action repetition
        updateCurrentActionRepetitions();

        bool petActionDoneResult;
        vertex result = schemaRunner->getSchemaExecResult();

        if (is_action_result(result)) {
            if (get_action(result) == id::action_success) {
                petActionDoneResult = true;
            } else {
                petActionDoneResult = false;
            }

        } else if (is_builtin(result)) {
            if (get_builtin(result) == id::logical_true) {
                petActionDoneResult = true;
            } else {
                petActionDoneResult = false;
            }

        } else {
            stringstream unexpected_result;
            unexpected_result << result;
            logger().warn(
                         "RuleEngine - Procedure result should be"
                         " built-in or action result. Got '%s'.",
                         unexpected_result.str().c_str());
            return;
        }


        // add lastPetActionDone action done predicate in atomSpace
        addSchemaDoneOrFailurePred(this->lastPetActionDone.getName(),
                                   this->lastPetActionDone.getParameters(),
                                   petActionDoneResult);
    }
}

void RuleEngine::runSchemaForCurrentAction( void )
{
    //  this->schemaRunner->updateStatus( );

    if ( this->candidateAction.getName( ).empty( ) ) {
        getNextAction( );
    } // if

    if ( this->candidateAction.getName( ).empty( ) ) {
        return;
    } // if

    const std::vector<std::string>& arguments =
        this->candidateAction.getParameters( );

    bool wasSelectedToExecute =
        this->schemaRunner->runSchema(this->candidateRule,
                                      this->candidateAction.getName(),
                                      arguments);

    if (wasSelectedToExecute) {

        this->currentAction = this->candidateAction;
        this->currentRule = this->candidateRule;

        string argListStr;
        foreach(string arg, arguments) {
            argListStr.append(arg);
            argListStr.append(" ");
        }
        logger().debug("RuleEngine - cycle '%d', action sent to execution:"
                     " '%s' with %d parameters: '%s' - rule '%s'",
                     this->cycle, getCurrentAction().c_str(),
                     arguments.size(), argListStr.c_str(),
                     getCurrentRule().c_str() );

        // reset requested command cycles counter if it was executed
        if ( this->lastRequestedCommandCycles > 0
                &&
                ( this->opc->getPet().getLatestRequestedCommand( ).name == getCurrentAction() )
           ) {
            this->lastRequestedCommandCycles = 0;
        } // if

        // if more then 10 cycles where passed after avatar ask to try clean
        if ( this->numberOfCyclesSinceAvatarAskToTry > 10 ||
                ( this->avatarAskedToTry && this->triedSchema == getCurrentAction())) {

            this->triedSchema = "";
            this->avatarAskedToTry = false;
            this->numberOfCyclesSinceAvatarAskToTry = 0;
        } // if

        this->opc->getPet().schemaSelectedToExecute(getCurrentAction( ));

        if ( getCurrentAction( ) == "group_command" ) {
            const std::vector<std::string>& parameters = this->currentAction.getParameters( );

            if ( parameters[0] == "join_group" ) {
                this->groupLeaderId = parameters[1];
                this->groupingMode = true;
                logger().debug(
                             "RuleEngine - Pet is in grouping mode."
                             " Leader: %s.",
                             parameters[1].c_str());

            } else if ( parameters[0] == "abandon_group" ) {
                this->groupLeaderId = "";
                this->groupingMode = false;
                logger().debug(
                             "RuleEngine - Pet is not in grouping mode.");
            } // else if
        } // if
    } // if
}

/* ----------------------------------------------------------------------------
 * Cycle functions
 * ----------------------------------------------------------------------------
 */
void RuleEngine::processRules( void )
{
    Procedure::ComboInterpreter comboInterpreter(this->opc->getPAI(), *rng);
    Procedure::ComboSelectInterpreter comboSelectInterpreter(this->opc->getPAI(), *rng);
    //const Procedure::ComboProcedureRepository& repository = this->opc->getProcedureRepository().getComboRepository();
    const Procedure::ProcedureRepository& repository = this->opc->getProcedureRepository();

    std::map<std::string, std::string>::iterator it;

    // debug log
    Logger::Level logLevel = Logger::FINE;
    if (logger().isEnabled(logLevel)) { 
        logRelations(logLevel);
    }
    // ~debug log

    for (it = this->rulePreconditionMap.begin();
            it != this->rulePreconditionMap.end(); it++) {

        this->currentInspectedRuleName = (*it).first;
        std::string precondition = (*it).second;

        variable_unifier unifier;
        const std::vector<std::string>& validTargets = getValidTargets();

        // all targets are initially considered part of the final unification
        foreach(std::string target, validTargets) {
            unifier.insert(target, true);
        }

        if (unifier.empty()) {
            unifier.insert(this->opc->getPet().getPetId(), true);
        }

        std::vector<vertex> arguments;

        const Procedure::GeneralProcedure& genp = repository.get(precondition);

        vertex result;
        Procedure::RunningProcedureId procedureId;
      

        if (genp.getType() == Procedure::COMBO) {
            const Procedure::ComboProcedure& cp =
                static_cast<const Procedure::ComboProcedure&>(genp);

            procedureId = comboInterpreter.runProcedure(cp.getComboTree(),
                          arguments, unifier);

            while (!comboInterpreter.isFinished(procedureId)) {
                comboInterpreter.run(NULL);
            }

            if (comboInterpreter.isFailed(procedureId)) {
                logger().error(
                             "RuleEngine - Precondition '%s' exec failed,"
                             " continuing to next rule.",
                             precondition.c_str());
                continue;
            }

            result = comboInterpreter.getResult(procedureId);

        } else if (genp.getType() == Procedure::COMBO_SELECT) {
            const Procedure::ComboSelectProcedure& csp =
                static_cast<const Procedure::ComboSelectProcedure&>(genp);
            procedureId =
                comboSelectInterpreter.runProcedure(csp.getFirstScript(),
                                                    csp.getSecondScript(),
                                                    arguments, unifier);

            while (!comboSelectInterpreter.isFinished(procedureId)) {
                comboSelectInterpreter.run(NULL);
            }

            if (comboSelectInterpreter.isFailed(procedureId)) {
                logger().error(
                             "RuleEngine - Precondition '%s' exec failed,"
                             " continuing to next rule.",
                             precondition.c_str());
                continue;
            }

            result = comboSelectInterpreter.getResult(procedureId);

        } else {
            throw RuntimeException(TRACE_INFO, "processRules - Invalid procedure type. Accepted ones: COMBO and COMBO_SELECT.");
        }

        // TODO fix that thing
        // see comment in combo/vertex.h
        if (combo::operator==(result,combo::id::logical_true)) {
        // if (result == id::logical_true) {

            if (procedureId.getType() == Procedure::COMBO) {
                updateValidTargets(comboInterpreter.getUnifierResult(procedureId));
            } else if (procedureId.getType() == Procedure::COMBO_SELECT) {
                updateValidTargets(comboSelectInterpreter.getUnifierResult(procedureId));
            } else {
                throw RuntimeException(TRACE_INFO, "processRules - Invalid procedure type. Accepted ones: COMBO and COMBO_SELECT.");
            }

            switch (this->ruleTypeMap[this->currentInspectedRuleName]) {
            case 0: { // schema effect

                Action action = boost::get<Action>(this->ruleEffectMap[this->currentInspectedRuleName]);
                suggestAction(this->currentInspectedRuleName,
                              action.getName(), action.getParameters());

                logger().debug(
                             "RuleEngine - Precondition '%s' exec true."
                             " Suggesting action '%s'.",
                             precondition.c_str(),
                             action.getName().c_str());
            }
            break;

            case 1: { // feeling effect
                Feeling feeling = boost::get<Feeling>(this->ruleEffectMap[this->currentInspectedRuleName]);
                suggestFeeling(feeling.getName(), feeling.getIntensity());
                logger().debug(
                             "RuleEngine - Precondition '%s' exec true."
                             " Suggesting feeling '%s'.",
                             precondition.c_str(),
                             feeling.getName().c_str());
            }
            break;

            case 2: { // relation effect
                Relation relation = boost::get<Relation>(this->ruleEffectMap[this->currentInspectedRuleName]);

                suggestRelation(relation.getName(),
                                relation.getTarget(),
                                relation.getIntensity());

                // debug log
                if (logger().isDebugEnabled()) {
                    const std::vector<std::string> vt = getValidTargets();
                    //list_of_targets <- "'target1' and 'target2' and ..."
                    string list_of_targets;
                    if (relation.getTarget() == WILD_CARD_STR) {
                        for (std::vector<std::string>::const_iterator
                                vsci = vt.begin(); vsci != vt.end();) {
                            list_of_targets += (string("'")
                                                + *vsci
                                                + string("'"));
                            ++vsci;
                            if (vsci != vt.end()) {
                                list_of_targets += string(" and ");
                            }
                        }
                    } else {
                        list_of_targets = (string("'")
                                           + relation.getTarget()
                                           + string("'"));
                    }
                    logger().debug(
                                 "RuleEngine - Precondition '%s' exec true."
                                 " Suggesting relation '%s' with %s.",
                                 precondition.c_str(),
                                 relation.getName().c_str(),
                                 list_of_targets.c_str());
                }
                // ~debug log

            }
            break;

            default:
                logger().error(
                             "RuleEngine - Unknown type for rule '%s'."
                             " Got type '%d', continuing to next rule.",
                             this->currentInspectedRuleName.c_str(),
                             this->ruleTypeMap[this->currentInspectedRuleName]);
                continue;
            }

        } else {
            logger().fine(
                         "RuleEngine - Precondition '%s' exec returned"
                         " false, continuing to next rule.",
                         precondition.c_str());
            continue;
        }
    }
}

void RuleEngine::processNextAction( void )
{

    //std::cout << "PROCESS NEXT ACTION - CYCLE: " << cycle << std::endl;

    // introduce a noise in the selection of the action
    // if bias is 1 then no selected is introduced
    // if is it 0 then maximal noise is introduced
    float bias = 1 - config().get_double("RE_SCHEMA_SELECTION_RANDOM_NOISE");

    // if enable then the wild_card candidate is choosen randomly
    // with uniform distribution
    bool enable_rand_wild_card = config().get_bool("RE_WILD_CARD_RANDOM_SELECTION");

    // Process next action only if there is map info data available...
    if ( this->opc->getAtomSpace()->getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED ) {
        logger().warn(
                     "RuleEngine - There is no map info available yet!");
        return;
    }
    // ... and pet spatial info is already received
    if ( !this->opc->getAtomSpace()->getSpaceServer().getLatestMap().containsObject(petName) ) {
        logger().error(
                     "RuleEngine - Pet was not inserted in the space map yet!" );
        return;
    }

    logger().debug(
                 "RuleEngine - start processing next action. Cycle '%d'.",
                 this->cycle );

    if (getCurrentAction().length() == 0) {
        std::map<std::string, float> feelingsUpdatedMap;
        foreach(std::string feeling, feelings) {
            float value = AtomSpaceUtil::getPredicateValue(*(opc->getAtomSpace()),
                          feeling, petHandle);
            feelingsUpdatedMap[ feeling ] = value;
        } // foreach

        opc->getPAI().sendEmotionalFeelings(this->petName,
                                            feelingsUpdatedMap);
    }

    // update novelty predicates
    updateKnownEntities();

    // update last pet action done and current action repetitions predicates
    updateSchemaExecutionStatus();

    logger().debug(
                 "RuleEngine - processing all rules. Cycle '%d'.",
                 this->cycle );
    processRules();
    logger().debug(
                 "RuleEngine - all rules processed");

    std::map<Action, std::vector<float> > weightedActions;
    std::map<Action, std::string> actionsRuleMap;
    std::map<Action, float> maxWeightedActions;

    {
        std:: string suggestedRules = "Cycle: ";
        suggestedRules += toString(cycle);
        suggestedRules += " - Suggested Rules {";
        std::multimap<Action, RuleEngine::RuleStrPair>::iterator it;
        for ( it = this->suggestedActions.begin( );
                it != this->suggestedActions.end( ); ++it ) {
            // initializing max weight for all actions
            maxWeightedActions[it->first] = -1.0f;
            suggestedRules += (it->second).first ;
            suggestedRules += ("-"
                               + toString((it->second).second)
                               + " ");
        } // for
        suggestedRules += "}";
        logger().debug(suggestedRules.c_str()) ;

        // grouping actions...
        for ( it = this->suggestedActions.begin( );
                it != this->suggestedActions.end( ); ++it ) {
            float weight = it->second.second;
            if (weight != 0.0f) { // this prevent to select 0 weighted actions
                // even when there is random noise selection
                weightedActions[it->first].push_back(weight);
                // if there are more than one rule that
                // leads to the same action,
                // the selected rule should be the one with higher weight
                if (it->second.second > maxWeightedActions[it->first]) {
                    maxWeightedActions[it->first] = weight;
                    actionsRuleMap[it->first] = it->second.first;
                }
            }
        } // for
    } // end block

    OC_ASSERT(!weightedActions.empty(), "RuleEngine - weightedActions should not be empty, maybe you forgot to affect a non-null weight to defaultAction rules");

    { // calculating the strength mean of all suggested actions..
        std::map<Action, std::vector<float> >::iterator it, it_chosen;
        float maximumWeight = -1;

        //to be sure that RuleEngine will select a candidate
        //we choose randomly one now that will be checked in the
        //"take the maximum weighted action" below
        int chosen_i = rng->randint(weightedActions.size());
        it_chosen = weightedActions.begin();
        for (int i = 0; i < chosen_i; ++i)
            ++it_chosen;

        for ( it = weightedActions.begin( );
                it != weightedActions.end( ); ++it ) {
            float candidateWeight = 0;
            unsigned int i;
            for ( i = 0; i < it->second.size( ); ++i ) {
                candidateWeight += it->second[ i ];
            } // for
            candidateWeight /= it->second.size( );

            //the following is added to perturbate randomly the
            //candidate schema selection and introduce undeterminism.
            //Note that this matters only if it is not the last
            //candidate so that at least one candidate is choosen
            bool take_candidate_into_account
                = (it == it_chosen || biased_randbool(bias, *rng));

            //std::cout << "TAKE CANDIDATE INTO ACCOUNT: "
            //        << take_candidate_into_account << std::endl;

            // take the maximum weighted action
            if ( take_candidate_into_account
                    && (candidateWeight > maximumWeight)) {
                maximumWeight = candidateWeight;
                this->candidateAction = it->first;
                this->candidateRule = actionsRuleMap[this->candidateAction];

                const std::vector<std::string>& parameters = it->first.getParameters( );
                if ( std::find( parameters.begin( ), parameters.end( ), WILD_CARD_STR ) != parameters.end( ) ) {

                    if ( this->varBindCandidates.size( ) == 0 ) {
                        logger().error(
                                     "No candidate found." );

                    } else {

                        // pci stands for parameter candidate index
                        // if the option rwpc is enable then it
                        // choose the parameter randomly
                        int pci = (enable_rand_wild_card ?
                                   rng->randint(varBindCandidates.size())
                                   : 0);

                        std::cout << "CHOOSE INDEX WILD_CARD CANDIDATE: "
                                  << pci << std::endl;

                        std::vector<std::string> bindParameters = parameters;
                        std::replace_if( bindParameters.begin( ),
                                         bindParameters.end( ),
                                         std::bind2nd( std::equal_to<std::string>(), WILD_CARD_STR ),
                                         this->varBindCandidates[pci] );
                        this->candidateAction.setParameters( bindParameters );
                    } // else
                } // if
            } // if
        } // for

        logger().debug(
                     "RuleEngine - Selected candidate rule '%s',"
                     " schema '%s' with weigth '%.3f'.",
                     this->candidateRule.c_str(),
                     this->candidateAction.getName( ).c_str( ),
                     maximumWeight);

    } // end block


//    logger().debug("RuleEngine - Last schema '%s' with '%d' parameters.",
//        this->lastPetActionDone.getName( ).c_str( ), this->lastPetActionDone.getParameters( ).size( ) );

    // update feelings
    std::map<std::string, std::vector<float> > weightedFeelings;
    { // calculating the mean of all suggested feelings
        std:: string suggestedFeelings = "Cycle: ";
        suggestedFeelings += toString(cycle);
        suggestedFeelings += " - Suggested Feelings {";

        std::set<Feeling>::iterator it;
        for ( it = this->suggestedFeelings.begin( );
                it != this->suggestedFeelings.end( ); ++it ) {

            weightedFeelings[(*it).getName()].push_back((*it).getIntensity());
            suggestedFeelings += ((*it).getName()
                                  + "-"
                                  + toString((*it).getIntensity())
                                  + " ");
        } // for
        suggestedFeelings += "}";
        logger().debug(suggestedFeelings.c_str()) ;

    } // end block


    { // updating feelings values...
        std::map<std::string, float> feelingsUpdatedMap;
        std::map<std::string, std::vector<float> >::iterator it;
        for ( it = weightedFeelings.begin( );
                it != weightedFeelings.end( ); ++it ) {
            unsigned int i;
            float newValue = 0;
            for ( i = 0; i < it->second.size( ); ++i ) {
                newValue += it->second[ i ];
            } // for

            newValue /= it->second.size( );

            float oldValue =
                AtomSpaceUtil::getPredicateValue(*(this->opc->getAtomSpace()),
                                                 it->first, this->petHandle);

            float revisedValue =  reviseValue( oldValue, newValue );

            AtomSpaceUtil::setPredicateValue(*(this->opc->getAtomSpace()),
                                             it->first,
                                             SimpleTruthValue(revisedValue,
                                                              0.0f),
                                             this->petHandle);

            feelingsUpdatedMap[ it->first ] = revisedValue;
        } // for

        opc->getPAI().sendEmotionalFeelings(this->petName, feelingsUpdatedMap);
    } // end block

    logger().debug("RuleEngine - feelings updated");

    // update relations
    { // updating relations...
        std:: string suggestedRelations = "Cycle: ";
        suggestedRelations += toString(cycle);
        suggestedRelations += " - Suggested Relations {";

        std::multiset<Relation>::iterator it;
        for ( it = this->suggestedRelations.begin();
                it != this->suggestedRelations.end(); ++it ) {
            Relation r = (*it);
            logger().debug(
                         "RuleEngine - Suggested relation '%s',"
                         " target '%s', intensity '%f'.",
                         r.getName( ).c_str( ),
                         r.getTarget( ).c_str( ), r.getIntensity() );

            suggestedRelations += (r.getName() + "-" + r.getTarget());
            suggestedRelations += " ";
            if ( r.getIntensity() > 0 ) {
                // remove opposites
                removeOppositeRelation(r);
                addRelation(r);

            } else {
                removeRelation(r);
            } // else
        } // for

        suggestedRelations += "}";
        logger().debug(suggestedRelations.c_str()) ;

    } // end block

    logger().debug("RuleEngine - relations updated");

    { // decrease and log current feelings
        std::map<std::string, float> feelingsUpdatedMap;
        float feelingsDecreaseFactor =
            config().get_double("RE_FEELINGS_DECREASE_FACTOR");

        foreach(std::string feeling, feelings) {

            float oldValue =
                AtomSpaceUtil::getPredicateValue(*(opc->getAtomSpace()),
                                                 feeling, petHandle);
            float revisedValue =
                limitValue( oldValue - feelingsDecreaseFactor );

            AtomSpaceUtil::setPredicateValue(*(opc->getAtomSpace()), feeling,
                                             SimpleTruthValue(revisedValue,
                                                              0.0f),
                                             petHandle);

            feelingsUpdatedMap[ feeling ] = revisedValue;

            if ( oldValue != revisedValue ) {
                std::string frameInstanceName = this->opc->getPet( ).getPetId( ) + "_" + feeling + "_emotion_directed";
                try {
                    std::map<std::string, Handle> elements;
                    elements["Experiencer"] = opc->getAtomSpace()->addNode( SEME_NODE, this->opc->getPet( ).getPetId( ) );
                    elements["State"] = opc->getAtomSpace()->addNode( CONCEPT_NODE, feeling );
                    AtomSpaceUtil::setPredicateFrameFromHandles( 
                        *(opc->getAtomSpace()), "#Emotion_directed", frameInstanceName,
                            elements, SimpleTruthValue( (revisedValue < 0.5) ? 0.0 : revisedValue, 0.0 ) );        
                        
                } catch ( const NotFoundException& ex ) {
                    Handle predicateNode = opc->getAtomSpace()->getHandle( PREDICATE_NODE, frameInstanceName );
                    if ( predicateNode != Handle::UNDEFINED ) {
                        AtomSpaceUtil::deleteFrameInstance( *this->opc->getAtomSpace(), predicateNode );
                    } // if
                } // catch
                
            } // if


        } // foreach

        opc->getPAI().sendEmotionalFeelings(this->petName, feelingsUpdatedMap);
    } // end block

    { // log known objects and avatars
        std::stringstream message;
        message.clear( );

        Id_EntityPerception_Map_Const_It it;
        message << "RuleEngine - Known objects( ";
        for ( it = this->objects.begin( ); it != this->objects.end( ); ++it ) {
            message << it->first << " ";
        } // for
        message << ") Known avatars( ";
        for ( it = this->avatars.begin( ); it != this->avatars.end( ); ++it ) {
            message << it->first << " ";
        } // for
        message << ")";
        logger().debug(message.str( ).c_str( ) );
    } // end block

    // update the STI values of the learned tricks
    this->learnedTricksHandler->update();


    // WARNING: this must be the last action of this method
    logger().debug(
                 "RuleEngine - preparing for next cycle...");
    prepareForNextCycle( );
    // WARNING: this must be the last action of this method
}

void RuleEngine::prepareForNextCycle( void )
{

    this->suggestedActions.clear( );
    this->suggestedRelations.clear( );
    this->suggestedFeelings.clear( );
    this->lastInspectedRuleName = "";

    cycle++;
}

/* ----------------------------------------------------------------------------
 * Reinforcement learning functions
 * ----------------------------------------------------------------------------
 */
void RuleEngine::rewardRule(unsigned long timestamp)
{
    reinforceRule(REWARD, timestamp);
}

void RuleEngine::punishRule(unsigned long timestamp)
{
    reinforceRule(PUNISH, timestamp);
}

bool RuleEngine::isSchemaExecFinished( Procedure::RunningProcedureID schemaID ) const
{
    return this->schemaRunner->isSchemaExecFinished( schemaID );
}

bool RuleEngine::isExecutingSchemaNow( void ) const
{
    return !this->schemaRunner->isSchemaExecFinished( );
}

Procedure::RunningProcedureID RuleEngine::getExecutingSchemaID( void ) const
{
    return this->schemaRunner->getExecutingSchemaID( );
}

void RuleEngine::reinforceRule(ReinforcementType type, unsigned long timestamp)
{
    unsigned long timeWindow =
        boost::numeric_cast<unsigned long>(config().get_double("RL_TIME_WINDOW") *
                                           pai::PAIUtils::getTimeFactor());
    float petaverseTimePerSlot =
        (float)timeWindow / this->gaussianVector.size();

    // avoid usigned long underflow
    unsigned long t;
    if (timeWindow > timestamp) {
        t = 0;
    } else {
        t = timestamp - timeWindow;
    } // if
    Temporal temporal(t, timestamp);


    // get all selected rules executed during within the time interval
    // (temporal)
    std::vector<HandleTemporalPair> pairs;
    logger().debug(
                 "RuleEngine - Getting EvalLinks.");

    AtomSpaceUtil::getAllEvaluationLinks(*(this->opc->getAtomSpace()), pairs,
                                         SELECTED_RULE_PREDICATE_NAME,
                                         temporal, TemporalTable::OVERLAPS,
                                         true);

    logger().debug("RuleEngine - Got EvalLinks.");

    if (pairs.empty()) {
        logger().warn(
                     "RuleEngine - Found mo EvaluationLink with"
                     " SelectedRule pred.");

    } else if (pairs.size() == 1) {
        logger().warn(
                     "RuleEngine - Found only one EvaluationLink with"
                     " SelectedRule pred.");

        // only one selected rule, apply full reward/punish

        Handle evalLink = pairs[0].getHandle();
        if (opc->getAtomSpace()->getType(evalLink) == EVALUATION_LINK) {

            Handle listLink = opc->getAtomSpace()->getOutgoing(evalLink, 1);
            if (opc->getAtomSpace()->getType(listLink) == LIST_LINK) {

                logger().debug(
                             "RuleEngine - Apply reinfocement.");
                applyReinforcement(type,
                                   opc->getAtomSpace()->getOutgoing(listLink, 0),
                                   1.0);
                logger().debug(
                             "RuleEngine - Applied reinfocement.");

            } else {
                logger().error(
                             "RuleEngine - Handle informed do not represent"
                             " list link.");
            }

        } else {
            logger().error(
                         "RuleEngine - Handle informed do not represent"
                         " an evaluation link.");
        }

    } else {
        logger().warn(
                     "RuleEngine - Found '%d' one EvaluationLink"
                     " with SelectedRule pred.", pairs.size());

        foreach(HandleTemporalPair pair, pairs) {

            // the initial timestamp t is used here because the discrete
            // gaussian distribuition is calculeted starting from the least
            // timestamp value from time window.
            unsigned int index =
                (int)floor((pair.getTemporal()->getUpperBound() - t)
                           / petaverseTimePerSlot);
            if (index > this->gaussianVector.size()) {
                logger().error(
                             "RuleEngine - Calculated index '%d' greater"
                             " than GaussianVector size '%d'.",
                             index, this->gaussianVector.size());
                continue;
            }

            Handle evalLink = pair.getHandle();
            if (opc->getAtomSpace()->getType(evalLink) != EVALUATION_LINK) {
                logger().error(
                             "RuleEngine - Handle informed do not"
                             " represent an evaluation link.");
                continue;
            }

            Handle listLink = opc->getAtomSpace()->getOutgoing(evalLink, 1);
            if (opc->getAtomSpace()->getType(listLink) != LIST_LINK) {
                logger().error(
                             "RuleEngine - Handle informed do not"
                             " represent list link.");
                continue;
            }

            logger().debug(
                         "RuleEngine - Apply reinfocement.");
            applyReinforcement(type,
                               opc->getAtomSpace()->getOutgoing(listLink, 0),
                               this->gaussianVector[index]);
            logger().debug(
                         "RuleEngine - Applied reinfocement.");

        } // foreach
    } // if

    // check if there is an executing schema, and if it has started before the
    // reinforcement timestamp, apply the reward/punish accordingly
    if (schemaRunner->isExecutingSchema()) {
        logger().info(
                     "RuleEngine - Applying reinforce to executing schema.");

        unsigned long executingSchemaTimestamp =
            schemaRunner->getExecutingSchemaTimestamp();

        // dot need to reinforce if executing action started after command
        // timestamp
        if (executingSchemaTimestamp >= timestamp) {
            return;

            // executiong action started before the time window, reinforce it with
            // maximum factor
        } else if (executingSchemaTimestamp < t) {

            logger().debug(
                         "RuleEngine - Apply reinfocement.");
            applyReinforcement(type,
                               schemaRunner->getExecutingSchemaImplicationLink(),
                               1.0);
            logger().debug(
                         "RuleEngine - Applied reinfocement.");

            // default, executing actions started within the time window
        } else {
            unsigned int index = (int)floor((executingSchemaTimestamp - t)
                                            / petaverseTimePerSlot);

            if (index <  this->gaussianVector.size()) {
                applyReinforcement(type, schemaRunner->getExecutingSchemaImplicationLink(), this->gaussianVector[index]);
            } else {
                logger().debug("RuleEngine - Calculated index '%d' greater than GaussianVector size '%d'.",
                             index, this->gaussianVector.size());
            } // if
        } // if
    } // if
}

void RuleEngine::applyReinforcement(ReinforcementType type,
                                    Handle implicationLink, float factor)
{
    AtomSpace& atomSpace = *(this->opc->getAtomSpace());

    if (atomSpace.getType(implicationLink) != IMPLICATION_LINK) {
        logger().error(
                     "RuleEngine - Handle isn't an ImplicationLink.");
        return;
    }

    float strength = 0.0f;
    TruthValuePtr tv = atomSpace.getTV(implicationLink);
    switch (type) {
    case REWARD: {
        float reinforce = factor * config().get_double("RL_REWARD");
        strength = tv->getMean() + reinforce;

        if (strength > config().get_double("MAX_RULE_STRENGTH")) {
            strength = config().get_double("MAX_RULE_STRENGTH");
        }
        break;
    }

    case PUNISH: {
        float reinforceFactor = factor * config().get_double("RL_PUNISH");
        strength = tv->getMean() - reinforceFactor;
        break;
    }

    default:
        break;
    }

#ifdef MORE_DEBUG_INFO
    vector<Handle> seq;
    atomSpace.getHandleSet(back_inserter(seq),
                           implicationLink, REFERENCE_LINK, false);
    if (seq.size() == 1) {
        Handle phraseNode = atomSpace.getOutgoing(seq[0], 0);
        logger().debug("RuleEngine - Update ImplicationLink TV mean for rule '%s'. OldTV '%.3f', newTV '%.3f'.",
                     atomSpace.getName(phraseNode).c_str(), tv->getMean(), strength);
    }

#else
    logger().debug("RuleEngine - Updating ImplicationLink TV mean. OldTV '%.3f', newTV '%.3f'.",
                 tv.getMean(), strength);
#endif

    atomSpace.setMean(implicationLink, strength);
}
