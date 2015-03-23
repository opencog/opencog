/*
 * opencog/embodiment/Control/OperationalAvatarController/SchemaRunner.h
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

#ifndef SCHEMARUNNER_H
#define SCHEMARUNNER_H

#include <string>
#include <vector>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/atomspace/Node.h>
#include "OAC.h"

namespace opencog { namespace oac {

/**
 * class SchemaRunner
 * this class is responsible to receive a schemaname and execute it
 */
class SchemaRunner
{
public:

    SchemaRunner( OAC* oac );

    virtual ~SchemaRunner( );

    Handle addLink( Type linkType, const HandleSeq& outgoing );

    /**
     * Returns true if the schema with the given name was really
     * sent to ProcedureInterpreter to be executed.
     */
    bool runSchema(const std::string& ruleName,
                   const std::string& schemaName,
                   const std::vector<std::string>& arguments);

    /**
     * Update local variables that shows if the previous selected schema is running or finished
     */
    void updateStatus( void );

    /**
     * Return the rule implication link for the current executing schema.
     * UndefinedHandle is returned if there is no executing schema.
     */
    inline Handle getExecutingSchemaImplicationLink() {
        return this->executingSchemaImplicationLink;
    }

    /**
     * Return the start timestamp for the current executing schema.
     */
    inline unsigned long getExecutingSchemaTimestamp() {
        return this->executingSchemaTimestamp;
    }

    /**
     * Return true if there is an schema being executed or false otherwise.
     */
    inline bool isExecutingSchema() {
        return !isSchemaExecFinished();//this->executingSchema;
    }

    /**
     * Return true if the current executing schema Id has finished its
     * execution, false otherwise.
     */
    inline bool isSchemaExecFinished() {
        return isSchemaExecFinished( this->executingSchemaID );
    }

    /**
     * Return the result for the executing schema Id. If called before the
     * schema has finished its execution an assertion exception will be raised.
     * So always use  isSchemaExecFinished() before getting the result.
     *
     * @return the schema execution result
     */
    combo::vertex getSchemaExecResult();

    inline bool isSchemaExecFinished( Procedure::RunningProcedureID schemaId ) {
        if (schemaId) {
            return (this->oac->getProcedureInterpreter().isFinished( schemaId ));
        }

        // there is no schema being executed, so it is finished
        return true;
    }

    inline Procedure::RunningProcedureID getExecutingSchemaID( void ) const {
        return this->executingSchemaID;
    }

private:
    OAC* oac;

    // Executing schema state variables
    bool executingSchema;
    Handle executingSchemaNode;
    time_t executingSchemaRealTime;
    Handle executingSchemaImplicationLink;
    unsigned long executingSchemaTimestamp;

    Procedure::RunningProcedureID executingSchemaID;
    SimpleTruthValue *defaultTruthValue;
    AttentionValue *defaultAttentionValue;

    float defaultMean;
    float defaultCount;
    long procedureExecutionTimeout;
    bool allowWalkingCancelation;

    bool petIsMoving;
    std::string currentWalkingProcedure;
    std::string currentWalkingTargetId;
};

} } // namespace opencog::oac

#endif // SCHEMARUNNER_H
