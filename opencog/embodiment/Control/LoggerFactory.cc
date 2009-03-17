/**
 * LoggerFactory.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jul 10 12:34:20 BRT 2007
 */

#include "LoggerFactory.h"
#include "stdlib.h"
#include "string.h"
using namespace Control;

LoggerFactory::LoggerFactory() {
}

#define USER_FLAG "$USER"

LADSUtil::Logger* LoggerFactory::getLogger(const SystemParameters &parameters, const std::string &id) {

    std::string logFile;
    std::string slash("/");
    int level;
    bool timestampEnabled;

    logFile.assign(parameters.get("LOG_DIR"));
    size_t user_index = logFile.find(USER_FLAG, 0);
    if (user_index != std::string::npos) {
		//const char* username = getlogin();
		const char* username = getenv("LOGNAME");
		MAIN_LOGGER.log(LADSUtil::Logger::WARNING, "LoggerFactory::getLogger: processing $USER flag => username = %s\n", username);
		if (username == NULL)
			username = "unknown_user";
        logFile = logFile.replace(user_index, strlen(USER_FLAG), username);
    }
    logFile.append(slash);
    logFile.append(id);
    MAIN_LOGGER.log(LADSUtil::Logger::WARNING, "LoggerFactory:getLogger(): log dir: %s\n", logFile.c_str());

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

    LADSUtil::Logger* result = new LADSUtil::Logger(logFile, level, timestampEnabled);

    if (atoi(parameters.get("PRINT_LOG_TO_STDOUT").c_str()) == 1) {
        result->setPrintToStdoutFlag(true);
    }

    return result;
}
