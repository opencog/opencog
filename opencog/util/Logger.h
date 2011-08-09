/*
 * opencog/util/Logger.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_LOGGER_H
#define _OPENCOG_LOGGER_H

#include <string>

#include <cstdarg>
#include <pthread.h>

#define ASYNC_LOGGING

#ifdef ASYNC_LOGGING
#include <boost/thread.hpp>
#include <opencog/util/concurrent_queue.h>
#endif

//#undef ERROR
//#undef DEBUG

namespace opencog
{

class Logger
{

public:

    //WARNING: if you change the levels don't forget to update
    //levelStrings[] in Logger.cc
    enum Level { NONE, ERROR, WARN, INFO, DEBUG, FINE, BAD_LEVEL=255 };

    /** Convert from 'enum' (int) to 'string' and vice-versa */
    static const Level getLevelFromString(const std::string&);
    static const char* getLevelString(const Level);

#ifdef ASYNC_LOGGING
    /** This thread does all writing of log messages */
    boost::thread m_Thread;

    /** Queue for log messages */
    concurrent_queue< std::string* > pendingMessagesToWrite;

    void startWriteLoop();
    void stopWriteLoop();
    void writingLoop();
    void flush();
#endif
    Logger& operator=(const Logger& log);
    void writeMsg(std::string &msg);

    // ***********************************************/
    // Constructors/destructors
    ~Logger();

    /**
     * Messages will be appended to the passed file.
     *
     * @param fileName The log file
     * @param level Only messages with log-level lower than or equals to level will be logged
     * @param timestampEnabled If true, a timestamp will be pre-fixed in every log message
     */
    Logger(const std::string &fileName = "opencog.log", Level level = INFO, bool timestampEnabled = true);

    Logger(const Logger&);
    void set(const Logger&);

    // ***********************************************/
    // API

    /**
     * Reset the level of messages which will be logged. Every message with
     * log-level lower than or equals to newLevel will be logged.
     */
    void setLevel(Level);

    /**
     * Get the current log level that determines which messages will be
     * logged: Every message with log-level lower than or equals to returned
     * level will be logged.
     */
    Level getLevel() const;

    /**
     * Set the level of messages which should be logged with back trace. 
     * Every message with log-level lower than or equals to the given argument
     * will have back trace.
     */
    void setBackTraceLevel(Level);

    /**
     * Get the current back trace log level that determines which messages
     * should be logged with back trace. 
     */
    Level getBackTraceLevel() const;

    /* filename property */
    void setFilename(const std::string&);
    const std::string& getFilename();

    /**
     * Reset the flag that indicates whether a timestamp is to be prefixed
     * in every message or not.
     */
    void setTimestampFlag(bool flag);

    /**
     * Reset the flag that indicates whether the log messages should be
     * printed to the stdout or not.
     */
    void setPrintToStdoutFlag(bool flag);

    /**
     * set the main logger to prints only
     * error level log on stdout (useful when one is only interested
     * in printing cassert logs)
     */
    void setPrintErrorLevelStdout();

    /**
     * Logs a message into log file (passed in constructor) if and only if passed level is
     * lower than or equals to the current log level of Logger.
     */
    void log  (Level level, const std::string &txt);
    void error(const std::string &txt);
    void warn (const std::string &txt);
    void info (const std::string &txt);
    void debug(const std::string &txt);
    void fine (const std::string &txt);

    /**
     * Logs a message (printf style) into log file (passed in constructor) if and only if passed level is
     * lower than or equals to the current log level of Logger.
     *
     * You may use these methods as any printf-style call, eg: fine("Count = %d", count)
     */
    void logva(Level level, const char *, va_list args);
    void log  (Level level, const char *, ...);
    void error(const char *, ...);
    void warn (const char *, ...);
    void info (const char *, ...);
    void debug(const char *, ...);
    void fine (const char *, ...);

    /** 
     * Methods to check if a given log level is enabled. This is useful for
     * avoiding unnecessary code for logger. For example: 
     * if (isDebugEnabled())  debug(...);
     */
    bool isEnabled(Level level) const;
    bool isErrorEnabled() const;
    bool isWarnEnabled() const;
    bool isInfoEnabled() const;
    bool isDebugEnabled() const;
    bool isFineEnabled() const;

    /**
     * Enable logging messages.
     */
    void enable();

    /**
     * Disable logging messages.
     */
    void disable();

private:

    std::string fileName;
    bool timestampEnabled;
    Level currentLevel;
    Level backTraceLevel;
    bool logEnabled;
    bool printToStdout;
    pthread_mutex_t lock;
    FILE *f;

}; // class

// singleton instance (following meyer's design pattern)
Logger& logger();

}  // namespace opencog

#endif // _OPENCOG_LOGGER_H
