/*
 * opencog/embodiment/Learning/LearningServerMessages/SchemaMessage.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#ifndef SCHEMAMESSAGE_H_
#define SCHEMAMESSAGE_H_

#include <opencog/embodiment/Control/MessagingSystem/Message.h>
#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace learningserver { namespace messages {

/**
 * This message type carries the schemas learned in LS. The candidate and the final
 * ones.
 */
class SchemaMessage : public opencog::messaging::Message
{

private:
    std::string schema;
    std::string schemaName;
    std::string candidateSchemaName;

    // the full message to be sent
    std::string message;

public:

    /**
     * Constructors and destructor
     */
    ~SchemaMessage();
    SchemaMessage(const std::string &from, const std::string &to);
    SchemaMessage(const std::string &from, const std::string &to,
                  const std::string &msg, int msgType);

    // If candidate schema name isn't set then it is assumed that the
    // message carries the final schema
    SchemaMessage(const std::string &from, const std::string &to,
                  const opencog::combo::combo_tree & comboSchema, const std::string &schemaName,
                  const std::string &candidateSchemaName = "");

    /**
     * Return A (char *) representation of the message, a c-style string terminated with '\0'.
     * Returned string is a const pointer hence it shaw not be modified and there is no need to
     * free/delete it.
     *
     * @return A (char *) representation of the message, a c-style string terminated with '\0'
     */
    const char *getPlainTextRepresentation();

    /**
     * Factory a message using a c-style (char *) string terminated with `\0`.
     *
     * @param strMessage (char *) representation of the message to be built.
     */
    void loadPlainTextRepresentation(const char *strMessage) throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * Set the schema to be sent by the message. This combo combo_tree
     * representation is converted to a string form.
     *
     * @param schema A combom combo_tree representation of the schema
     */
    void setSchema(const opencog::combo::combo_tree & comboSchema);

    /**
     * Get the combo combo_tree representation of the schema
     *
     * @return combo_tree representing the schema
     */
    const opencog::combo::combo_tree getComboSchema();

    /**
     * Get the string representation of the schema
     *
     * @return string representing the schema
     */
    const std::string & getSchema();

    void setSchemaName(const std::string & schemaName);
    const std::string & getSchemaName();

    void setCandidateSchemaName(const std::string & candidateSchemaName);
    const std::string & getCandidateSchemaName();

}; // class
} } } // namespace opencog::learningserver::messages

#endif
