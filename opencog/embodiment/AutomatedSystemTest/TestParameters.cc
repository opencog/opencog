/*
 * opencog/embodiment/AutomatedSystemTest/TestParameters.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
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

#include "TestParameters.h"

using namespace AutomatedSystemTest;

TestParameters::TestParameters()
{

    table["CONFIG_FILE"] = "test.cfg";

    // Flag to enable/disable the saving of messages (for using in automated tests).
    table["SAVE_MESSAGES_TO_FILE"] = "1";
    // Name of the file where the messages will be saved, if the previous parameter is enabled.
    table["MESSAGES_FILENAME"] = "PBTesterMessages.txt";

    table["PROXY_IP"] = "127.0.0.1";
    table["PROXY_PORT"] = "16315";
}

TestParameters::~TestParameters()
{
}

