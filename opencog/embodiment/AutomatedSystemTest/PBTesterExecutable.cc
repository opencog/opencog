/*
 * opencog/embodiment/AutomatedSystemTest/PBTesterExecutable.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Luigi
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

/**
 * PBTester.cc
 * This program performs automated system tests on PB-side of Petaverse code.
 * It simulates the PVP Proxy by reading from a file a sequence of messages to be sent and
 * received to/from Petaverse ROUTER.
 *
 * Author: Welter Luigi
 */


#include "PBTester.h"
#include <exception>
#include <unistd.h>
#include "util/files.h"
#include "GoldStdReaderAgent.h"

using namespace AutomatedSystemTest;

int main(int argc, char *argv[])
{

    // Open/read the data file passed as argument
    if (argc < 2) {
        printf("Wrong number of arguments:\nExpected: %s <Gold Standard Filename>\n", argv[0]);
    }
    const char* filename = argv[1];

    Control::SystemParameters parameters;
    if (fileExists(parameters.get("CONFIG_FILE").c_str())) {
        parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    TestParameters testParameters;
    if (fileExists(testParameters.get("CONFIG_FILE").c_str())) {
        testParameters.loadFromFile(testParameters.get("CONFIG_FILE"));
    }

    server(PBTester::createInstance);
    PBTester& pbTester = static_cast<PBTester&>(server());
    pbTester.init(parameters, testParameters, parameters.get("PROXY_ID"), testParameters.get("PROXY_IP"), atoi(testParameters.get("PROXY_PORT").c_str()));

    Factory<GoldStdReaderAgent, Agent> goldStdReaderAgentFactory;

    pbTester.registerAgent(GoldStdReaderAgent::info().id, &goldStdReaderAgentFactory);
    GoldStdReaderAgent* goldStdReaderAgent = static_cast<GoldStdReaderAgent*>(
                pbTester.createAgent(GoldStdReaderAgent::info().id, false));
    goldStdReaderAgent->init(testParameters, filename);
    pbTester.startAgent(goldStdReaderAgent);

    try {
        pbTester.serverLoop();
    } catch (std::bad_alloc) {
        opencog::logger().log(opencog::Logger::ERROR, "PBTesterExec - PBTester raised a bad_alloc exception.");

    } catch (...) {
        opencog::logger().log(opencog::Logger::ERROR,
                              "PBTesterExec - An exceptional situation occured. Check log for information.");
    }

    return 0;
}

