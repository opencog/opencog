/*
 * tests/embodiment/Control/OperationalAvatarController/LSMessageSenderMock.h
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
#ifndef LSMESSAGESENDERMOCK_H_
#define LSMESSAGESENDERMOCK_H_

#include <opencog/embodiment/Control/OperationalAvatarController/MessageSender.h>
using namespace opencog::oac;

#include <stdio.h>

class LSMessageSenderMock : public MessageSender
{

public:
    LSMessageSenderMock() {}
    ~LSMessageSenderMock() {}

    bool sendReward(const std::string &schema, const std::vector<std::string> & schemaArgs, const std::string &triedSchema, const double reward) {
        fprintf(stdout, "Sending reward message. schema: '%s', triedSchema: '%s', reward: '%f'\n", schema.c_str(), triedSchema.c_str(), reward);
        return true;
    }

    bool sendExemplar(const std::string &schema, const std::vector<std::string> &schemaArgs, const std::string &ownerId, const std::string &avatarId, AtomSpace &atomSpace) {
        fprintf(stdout, "Sending exemplar. schema: '%s', owner: '%s', avatar: '%s'\n", schema.c_str(), ownerId.c_str(), avatarId.c_str());
        return true;
    }

    bool sendCommand(const std::string &command, const std::string &schema) {
        fprintf(stdout, "Sending command. command: '%s', schema: '%s'\n", command.c_str(), schema.c_str());
        return true;
    }

    bool sendFeedback(const std::string &ownerId, const std::string &feedback) {
        fprintf(stdout, "Sending feedback. ownerId: '%s', feedback: '%s'\n", ownerId.c_str(), feedback.c_str());
        return true;
    }

    bool sendTrySchema(const std::string &schemaName, const std::vector<std::string> & schemaArgs) {
        fprintf(stdout, "Sending Try command. schema name: '%s'.\n", schemaName.c_str());

        return true;
    }

    bool sendStopLearning(const std::string &schemaName, const std::vector<std::string> & schemaArgs) {
        fprintf(stdout, "Sending stop learning command. schema name: '%s'.\n", schemaName.c_str());

        return true;
    }
};

#endif /*LSMESSAGESENDERMOCK_H_*/
