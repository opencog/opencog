/*
 * opencog/embodiment/Control/LoggerFactory.cc
 *
 * Copyright (C) 2007-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna
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
using namespace Control;

LoggerFactory::LoggerFactory()
{
}

#define USER_FLAG "$USER"

opencog::Logger LoggerFactory::getLogger(const SystemParameters &parameters,
        const std::string &id)
{

    std::string logFile;
    std::string slash("/");
    int level;
    bool timestampEnabled;

    logFile.assign(parameters.get("LOG_DIR"));
    size_t user_index = logFile.find(USER_FLAG, 0);
    if (user_index != std::string::npos) {
        //const char* username = getlogin();
        const char* username = getenv("LOGNAME");
        opencog::logger().log(opencog::Logger::INFO, "LoggerFactory::getLogger: processing $USER flag => username = %s\n", username);
        if (username == NULL)
            username = "unknown_user";
        logFile = logFile.replace(user_index, strlen(USER_FLAG), username);
    }
    logFile.append(slash);
    logFile.append(id);
    opencog::logger().log(opencog::Logger::INFO,
                          "LoggerFactory:getLogger(): log dir: %s\n",
                          logFile.c_str());

    opencog::config().set("BACK_TRACE_LOG_LEVEL", parameters.get("BACK_TRACE_LOG_LEVEL"));

    if (id == parameters.get("SPAWNER_ID")) {
        level = atoi(parameters.get("SPAWNER_LOG_LEVEL").c_str());
    } else if (id == parameters.get("ROUTER_ID")) {
        level = atoi(parameters.get("ROUTER_LOG_LEVEL").c_str());
    } else if (id == parameters.get("LS_ID")) {
        level = atoi(parameters.get("LS_LOG_LEVEL").c_str());
    } else if (id == parameters.get("PROXY_ID")) {
        level = atoi(parameters.get("PROXY_LOG_LEVEL").c_str());
    } else {
        level = atoi(parameters.get("OPC_LOG_LEVEL").c_str());
    }

    if (atoi(parameters.get("TIMESTAMP_ENABLED_IN_LOGS").c_str()) == 0) {
        timestampEnabled = false;
    } else {
        timestampEnabled = true;
    }

    opencog::Logger result(logFile,
                           opencog::Logger::Level(level),
                           timestampEnabled);

    if (atoi(parameters.get("PRINT_LOG_TO_STDOUT").c_str()) == 1) {
        result.setPrintToStdoutFlag(true);
    }

    return result;
}
