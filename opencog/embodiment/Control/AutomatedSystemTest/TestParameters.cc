/**
 * TestParameters.cc
 *
 * Author: Welter Luigi
 */

#include "TestParameters.h"

using namespace AutomatedSystemTest;

TestParameters::TestParameters() {

    table["CONFIG_FILE"] = "test.cfg";

    // Flag to enable/disable the saving of messages (for using in automated tests).
    table["SAVE_MESSAGES_TO_FILE"] = "1";
    // Name of the file where the messages will be saved, if the previous parameter is enabled.
    table["MESSAGES_FILENAME"] = "PBTesterMessages.txt";

    table["PROXY_IP"] = "127.0.0.1";
    table["PROXY_PORT"] = "16315";
}

TestParameters::~TestParameters() {
}

