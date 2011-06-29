/*
 * opencog/embodiment/Control/LoggerFactory.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include "LoggerFactory.h"
#include "stdlib.h"
#include "string.h"
#include <opencog/util/Config.h>

using namespace opencog;
using namespace opencog::control;

LoggerFactory::LoggerFactory()
{
}

#define USER_FLAG "$USER"

opencog::Logger LoggerFactory::getLogger(const std::string &id)
{

    std::string logFile;
    std::string slash("/");
    Logger::Level level;
    bool timestampEnabled;

    logFile.assign(config().get("LOG_DIR"));
    size_t user_index = logFile.find(USER_FLAG, 0);
    if (user_index != std::string::npos) {
        //const char* username = getlogin();
        const char* username = getenv("LOGNAME");
        logger().info("LoggerFactory::getLogger: processing $USER flag => username = %s\n", username);
        if (username == NULL)
            username = "unknown_user";
        logFile = logFile.replace(user_index, strlen(USER_FLAG), username);
    }
    logFile.append(slash);
    logFile.append(id);
    logger().info(
                 "LoggerFactory:getLogger(): log dir: %s\n",
                 logFile.c_str());

    if (id == config().get("SPAWNER_ID")) {
        level = Logger::getLevelFromString(config().get("SPAWNER_LOG_LEVEL"));
    } else if (id == config().get("ROUTER_ID")) {
        level = Logger::getLevelFromString(config().get("ROUTER_LOG_LEVEL"));
    } else if (id == config().get("LS_ID")) {
        level = Logger::getLevelFromString(config().get("LS_LOG_LEVEL"));
    } else if (id == config().get("PROXY_ID")) {
        level = Logger::getLevelFromString(config().get("PROXY_LOG_LEVEL"));
    } else {
        level = Logger::getLevelFromString(config().get("OAC_LOG_LEVEL"));
    }

    timestampEnabled = config().get_bool("TIMESTAMP_ENABLED_IN_LOGS");

    Logger result(logFile,
                  Logger::Level(level),
                  timestampEnabled);

    if (config().get_bool("PRINT_LOG_TO_STDOUT")) {
        result.setPrintToStdoutFlag(true);
    }

    return result;
}
