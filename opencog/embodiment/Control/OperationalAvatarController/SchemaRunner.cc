/*
 * opencog/embodiment/Control/OperationalAvatarController/SchemaRunner.cc
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

#include "SchemaRunner.h"
#include <opencog/atomspace/HandleTemporalPair.h>
#include <boost/regex.hpp>

using namespace opencog::oac;

SchemaRunner::SchemaRunner( OAC* oac ) : oac( oac )
{
    this->defaultMean = config().get_double("RE_DEFAULT_MEAN");
    this->defaultCount = config().get_double("RE_DEFAULT_COUNT");
    this->defaultTruthValue = new SimpleTruthValue(defaultMean, defaultCount);
    this->defaultAttentionValue = AttentionValue::factory(AttentionValue::DEFAULTATOMSTI, 0);
    this->procedureExecutionTimeout = config().get_long("PROCEDURE_EXECUTION_TIMEOUT");
    this->allowWalkingCancelation = config().get_bool("ALLOW_WALKING_CANCELATION");
    this->petIsMoving = false;

    this->currentWalkingProcedure = "";
    this->currentWalkingTargetId = "";

    this->executingSchemaID = 0;
    this->executingSchema = false;
    this->executingSchemaTimestamp = 0;
    this->executingSchemaNode = Handle::UNDEFINED;
    this->executingSchemaImplicationLink = Handle::UNDEFINED;
}

SchemaRunner::~SchemaRunner( )
{
    delete(defaultTruthValue);
    delete(defaultAttentionValue);
}

Handle SchemaRunner::addLink( Type linkType, const HandleSeq& outgoing )
{
    logger().fine("SchemaRunner - addLink - init");
    Handle result = AtomSpaceUtil::addLink(*(oac->getAtomSpace()), linkType, outgoing );
    oac->getAtomSpace()->setAV(result, *defaultAttentionValue);
    oac->getAtomSpace()->setTV(result, *defaultTruthValue);
    logger().fine("SchemaRunner - addLink - end");
    return result;
}


bool SchemaRunner::runSchema(const std::string& ruleName,
                             const std::string& schemaName,
                             const std::vector<std::string>& arguments )
{
    logger().fine(
                 ("SchemaRunner - Executing runSchema: " + schemaName).c_str());

    const AtomSpace& atomSpace = *(this->oac->getAtomSpace( ));

    // Cannot select a schema to execute while
    // there is no map info data available...
    if (this->oac->getAtomSpace()->getSpaceServer().getLatestMapHandle() == Handle::UNDEFINED) {
        logger().warn(
                     "SchemaRunner - Cannot select any schema to be executed"
                     " because there is no map info available yet!");
        return false;
    } // if

    if ( this->executingSchema ) {
        if ( this->oac->getProcedureInterpreter( ).isFinished( this->executingSchemaID ) ) {

            HandleSeq listLinkOutgoing;
            listLinkOutgoing.push_back(this->executingSchemaImplicationLink);

            Handle listLink = addLink(LIST_LINK, listLinkOutgoing);
            Handle selectedRulePredicateNode =
                AtomSpaceUtil::addNode(*(this->oac->getAtomSpace()),
                                       PREDICATE_NODE,
                                       SELECTED_RULE_PREDICATE_NAME);

            HandleSeq evalLinkOutgoing;
            evalLinkOutgoing.push_back(selectedRulePredicateNode);
            evalLinkOutgoing.push_back(listLink);

            Handle evalLink = addLink(EVALUATION_LINK, evalLinkOutgoing);

            // adding atTimeLink
            this->oac->getAtomSpace()->getTimeServer().addTimeInfo(evalLink, this->oac->getPAI( ).getLatestSimWorldTimestamp( ) );

            // clear executingSchema variables
            this->petIsMoving = false;
            this->executingSchema = false;
            this->executingSchemaTimestamp = 0;
            this->executingSchemaNode = Handle::UNDEFINED;
            this->executingSchemaImplicationLink = Handle::UNDEFINED;

        } else {
            logger().info(
                         "SchemaRunner - Previous schema still executing");
            // Check for timeout
            time_t now = time(NULL);
            if ((now - executingSchemaRealTime) <= procedureExecutionTimeout) {

                if (allowWalkingCancelation) {
                    bool walkCommand = ( petIsMoving && schemaName.substr( 0, 2 ) == "go" );
                    if ( walkCommand && currentWalkingProcedure == schemaName &&

                            // using strcmp to compare these two strings since the
                            // string ways wasn't working for a reason that I haven't
                            // found way - Carlos Lopes
                            strcmp(currentWalkingTargetId.c_str(), arguments[0].c_str()) == 0) {

                        // check if the target move, thus replan
                        std::pair<std::string, spatial::Point> lastTargetObject = this->oac->getPet( ).getLatestGotoTarget( );
                        const SpaceServer::SpaceMap& spaceMap =
                            this->oac->getAtomSpace()->getSpaceServer( ).getLatestMap( );
                        try {
                            //const spatial::Object& targetObject = spaceMap.getObject( lastTargetObject.first );
                            const spatial::EntityPtr& targetEntity =
                                spaceMap.getEntity( lastTargetObject.first );
                            spatial::Point targetCenterPosition( targetEntity->getPosition( ).x, targetEntity->getPosition( ).y );//targetObject.metaData.centerX, targetObject.metaData.centerY );

                            if ( lastTargetObject.second == targetCenterPosition ) {
                                // there is no need to replan walking
                                return false;
                            } // if

                            this->oac->getProcedureInterpreter( ).stopProcedure(executingSchemaID );
                            logger().info("SchemaRunner - Replanning walk (new: %s, %s[%f,%f] old: %s, %s[%f,%f])...",  schemaName.c_str(), lastTargetObject.first.c_str(), targetCenterPosition.first, targetCenterPosition.second, currentWalkingProcedure.c_str(), currentWalkingTargetId.c_str(), lastTargetObject.second.first, lastTargetObject.second.second );
                        } catch ( opencog::NotFoundException& ex ) {
                            // it is impossible to determine if the replanning is needed
                            return false;
                        } // catch
                    } else if ( walkCommand && currentWalkingProcedure != schemaName ) {
                        // logger().info("SchemaRunner - Replanning walk(new: %s args: %d old: %s, %s)...", schemaName.c_str(), arguments.size(), currentWalkingProcedure.c_str(), currentWalkingTargetId.c_str() );
                        // this->oac->getProcedureInterpreter( ).stopProcedure(executingSchemaID );

                        if (this->oac->getPet().getMode() == opencog::oac::SCAVENGER_HUNT) {
                            logger().info("SchemaRunner - Replanning walk(new: %s args: %d old: %s, %s)...", schemaName.c_str(), arguments.size(), currentWalkingProcedure.c_str(), currentWalkingTargetId.c_str() );

                            this->oac->getProcedureInterpreter( ).stopProcedure(executingSchemaID );
                        } else {
                            return false;
                        }
                    } else {
                        return false;
                    } // else

                    /*
                      std::pair<std::string, spatial::Point> targetObject =
                      this->oac->getPet( ).getLatestGotoTarget( );

                      // check if pet is moving to the right position
                      if ( this->petIsMoving && schemaName == "keepMoving" &&
                      targetObject.first.length( ) > 0 ) {

                      const SpaceServer::SpaceMap& spaceMap = this->oac->getAtomSpace().getSpaceServer().getLatestMap();
                      spatial::Point currentTargetPosition = spaceMap.centerOf( targetObject.first );
                      // verify if the distance between the objects start position and current position
                      // is greater than tolerance
                      double dist = SpaceServer::SpaceMap::eucDist
                      ( targetObject.second, currentTargetPosition );

                      // maximum tolerance is 2% of the map width
                      double maxDistance = ( spaceMap.xMax( ) - spaceMap.xMin( ) ) * 0.02;
                      if ( dist > maxDistance ) {

                      this->oac->getProcedureInterpreter( ).stopProcedure(executingSchemaID );

                      logger().debug("SchemaRunner - Replanning %s to target %s.", currentWalkingProcedure.c_str( ), targetObject.first.c_str( ) );
                      // replan walking to a new target position
                      std::vector<combo::vertex> schemaArguments;
                      schemaArguments.push_back( targetObject.first );
                      const Procedure::GeneralProcedure& procedure =
                      this->oac->getProcedureRepository( ).get( currentWalkingProcedure );

                      this->executingSchemaID = this->oac->getProcedureInterpreter( ).runProcedure( procedure, schemaArguments );

                      } // if

                      } // if
                    */
                } else { // allow walking cancelation
                    return false;
                }
            } else {
                logger().error("SchemaRunner - Previous schema execution timeout: now = %lu, executingSchemaRealTime = %lu (timeout = %lu)", now, executingSchemaRealTime, procedureExecutionTimeout);
                this->oac->getProcedureInterpreter( ).stopProcedure(executingSchemaID);
                this->executingSchema = false;
                this->executingSchemaTimestamp = 0;
                this->executingSchemaNode = Handle::UNDEFINED;
                this->executingSchemaImplicationLink = Handle::UNDEFINED;
                this->petIsMoving = false;
            } // else
        } // else

    } // if

    Handle schemaNode = atomSpace.getHandle(GROUNDED_SCHEMA_NODE, schemaName);
    if (schemaNode == Handle::UNDEFINED) {
        logger().error(
                     "SchemaRunner - invalid selected schema: %s",
                     schemaName.c_str());

        schemaNode = this->oac->getAtomSpace()->getHandle(GROUNDED_SCHEMA_NODE,
                     "full_of_doubts" );
        if ( schemaNode == Handle::UNDEFINED ) {
            return false;
        } // if
    } // if

    // caching executing rule implication link
    Handle ruleImplicationLink = AtomSpaceUtil::getRuleImplicationLink(atomSpace,
                                 ruleName);
    if (ruleImplicationLink == Handle::UNDEFINED) {
        logger().error(
                     "SchemaRunner - Found no ImplicationLink for rule: %s",
                     ruleName.c_str( ) );
        return false;
    }// if

    // Get the procedure from ProcedureRepository
    if ( this->oac->getProcedureRepository().contains(schemaName) ) {
        const Procedure::GeneralProcedure& procedure =
            this->oac->getProcedureRepository( ).get( schemaName );

        std::vector<combo::vertex> schemaArguments;

        boost::regex isNumberPattern("^(\\d+|\\d+\\.\\d+)$");

        combo::arity_t arity = procedure.getArity();
        bool fixed_arity = arity >= 0; //the procedure gets fix number of
        //input arguments
        combo::arity_t abs_min_arity = combo::abs_min_arity(arity);

        for ( std::string argument : arguments ) {
            logger().debug(
                         "SchemaRunner - Adding argument '%s' to Procedure '%s'.",
                         argument.c_str(), procedure.getName().c_str());
            if ( boost::regex_match(argument, isNumberPattern ) ) {
                schemaArguments.push_back( combo::contin_t( boost::lexical_cast<combo::contin_t>( argument ) ) );
            } else {
                schemaArguments.push_back( argument );
            } // else
        } // foreach

        //the number of input arguments that are to be applied
        combo::arity_t ap_input_args = schemaArguments.size();

        if (fixed_arity) {
            if (arity != (int)ap_input_args) {
                logger().error(
                             "SchemaRunner - Number of arguments %d for"
                             " Procedure '%s' does not match arity %d",
                             ap_input_args,
                             procedure.getName().c_str(), arity);
                return false;
            }
        } else {
            if (abs_min_arity > (int)ap_input_args) {
                logger().error(
                             "SchemaRunner - Number of arguments %d for"
                             " Procedure '%s' is too few,"
                             " the minimum number expected is %d.",
                             ap_input_args,
                             procedure.getName().c_str(),
                             abs_min_arity);
                return false;
            }
        }


        logger().info(
                     "SchemaRunner - Running Procedure %s, type %d.",
                     procedure.getName().c_str(), procedure.getType());

        // if the schema is a walking schema, put the pet in walking mode
        if (allowWalkingCancelation && schemaName.substr( 0, 2 ) == "go") {
            //          this->oac->getPet( ).setLatestGotoTarget( std::pair<std::string,spatial::Point>( "", spatial::Point( 0, 0 ) ) );
            this->petIsMoving = true;
            this->currentWalkingProcedure = schemaName;

            OC_ASSERT(!arguments.empty(),
                    "There must be at least one argument applied"
                    " to procedure '%s', instead there is no argument",
                    schemaName.c_str());
            this->currentWalkingTargetId = arguments[0];
        } // if

        executingSchemaID = this->oac->getProcedureInterpreter( ).runProcedure( procedure, schemaArguments );

        logger().debug(
                     "SchemaRunner - Procedure %s sent to execution.",
                     procedure.getName().c_str());

        this->executingSchema = true;
        this->executingSchemaNode = schemaNode;
        this->executingSchemaRealTime = time(NULL);
        this->executingSchemaImplicationLink = ruleImplicationLink;
        this->executingSchemaTimestamp = this->oac->getPAI().getLatestSimWorldTimestamp();

        return true;

    } else {
        logger().error(
                     "SchemaRunner - Cannot execute grounded schema '%s'."
                     " Schema not found in Procedure Repository.",
                     schemaName.c_str());
        return false;
    } // else

}

void SchemaRunner::updateStatus( void )
{
    if ( this->executingSchema ) {
        if ( this->oac->getProcedureInterpreter( ).isFinished( this->executingSchemaID ) ) {
            this->executingSchema = false;
            this->petIsMoving = false;
        } // if
    } // if
}

combo::vertex SchemaRunner::getSchemaExecResult()
{
    // id equals 0, not action sent to execution yet
    // return action success
    if (this->executingSchemaID == 0) {
        return combo::id::action_success;
    }

    if (!this->oac->getProcedureInterpreter().isFailed(this->executingSchemaID)) {
        return (this->oac->getProcedureInterpreter().getResult(this->executingSchemaID));
    }
    return combo::id::action_failure;
}
