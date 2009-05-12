/*
 * opencog/embodiment/AutomatedSystemTest/TestConfig.cc
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


#include "TestConfig.h"

using namespace AutomatedSystemTest;
using namespace opencog;

TestConfig::TestConfig()
{

    table["TEST_CONFIG_FILE"] = "test.conf";

    // Flag to enable/disable the saving of messages (for using in automated tests).
    table["SAVE_MESSAGES_TO_FILE"] = "true";
    // Name of the file where the messages will be saved, if the previous parameter is enabled.
    table["MESSAGES_FILENAME"] = "PBTesterMessages.txt";

    table["PROXY_IP"] = "127.0.0.1";
    table["PROXY_PORT"] = "16315";
}

void TestConfig::reset() {
    EmbodimentConfig::reset();
    //Note that C++ calls Config::Config(), that itself calls Config::reset()
    //so there is no need to call it here in order to
    //inherit the default parameters.
    //Also, EmbodimentConfig::reset() overwrites existing default paramters
    //defined in Config::reset()
    
    // load embodiment default configuration
    for (unsigned int i = 0; TEST_DEFAULT()[i] != ""; i += 2) {
        table[TEST_DEFAULT()[i]] = TEST_DEFAULT()[i + 1];
    }
}

TestConfig::~TestConfig()
{
}

Config* TestConfig::testCreateInstance()
{
    return new TestConfig();
}
