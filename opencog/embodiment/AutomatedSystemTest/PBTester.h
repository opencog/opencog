/*
 * opencog/embodiment/AutomatedSystemTest/PBTester.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


#ifndef _PB_TESTER_H
#define _PB_TESTER_H

#include <string>
#include <map>
#include <set>

#include <opencog/embodiment/Control/MessagingSystem/MessageCogServer.h>
#include <opencog/embodiment/Control/MessagingSystem/Message.h>
#include "TestConfig.h"
#include "GoldStdGen.h"

namespace AutomatedSystemTest
{

class PBTester : public opencog::messaging::MessageCogServer
{

private:

    bool failed;
    unsigned long numberOfReceivedMessages;

    void initialize();

    std::vector<opencog::messaging::Message*> expectedMessages;
    std::vector<unsigned long> receivedTimeMessages;

    GoldStdGen* goldStdGen;

public:

    static opencog::BaseServer* createInstance();

    PBTester();
    void init();
    void init(const std::string &myId, const std::string &ip, int portNumber);
    ~PBTester();

    bool processNextMessage(opencog::messaging::Message *message);
    void addExpectedMessage(opencog::messaging::Message* message, unsigned long time);
    bool hasExpectedMessages();
    void notifyEndOfGoldStdFile();
    GoldStdGen* getGoldStdGen();
    unsigned long getReceivedTimeOfCurrentExpectedMessage() {
        return receivedTimeMessages.back();
    }

}; // class
}  // namespace

#endif
