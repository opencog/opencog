/*
 * opencog/embodiment/Learning/LearningServerMessages/RewardMessage.h
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

#ifndef REWARDMESSAGE_H_
#define REWARDMESSAGE_H_

#include <vector>
#include <opencog/embodiment/Control/MessagingSystem/Message.h>

namespace opencog { namespace learningserver { namespace messages {

class RewardMessage : public opencog::messaging::Message
{

private:
    double reward;
    std::string schema;
    std::string candidateSchema;
    std::vector<std::string> schemaArguments;

    // the full message to be sent
    std::string message;

public:

    static const double POSITIVE;
    static const double NEGATIVE;

    ~RewardMessage();
    RewardMessage(const std::string &from, const std::string &to);
    RewardMessage(const std::string &from, const std::string &to,
                  const std::string &msg);
    RewardMessage(const std::string &from, const std::string &to,
                  const std::string &schema, const std::vector<std::string> & schemaArguments,
                  const std::string &candidateSchema, double reward);

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
    void loadPlainTextRepresentation(const char *strMessage);

    /**
     * Atribute methods
     */
    void  setSchema(const std::string &schema);
    const std::string getSchema();

    void  setCandidateSchema(const std::string &candidadteSchema);
    const std::string getCandidateSchema();

    void setReward(double reward);
    double  getReward();

    void setSchemaArguments(const std::vector<std::string> & schemaArgs) {
        this->schemaArguments = schemaArgs;
    };
    const std::vector<std::string> getSchemaArguments() {
        return schemaArguments;
    };
    static int _rewardMsgType;
private:
    static int init();

}; // class
} } } // namespace opencog::learningserver::messages

#endif
