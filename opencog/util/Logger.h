/*
 * opencog/util/Logger.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <cstdarg>
#include <mutex>
#include <string>
#include <thread>

#include <opencog/util/concurrent_queue.h>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

//! logging evens
class Logger
{

    void set(const Logger&);
    bool writingLoopActive;
public:

    // WARNING: if you change the levels don't forget to update
    // levelStrings[] in Logger.cc
    enum Level { NONE, ERROR, WARN, INFO, DEBUG, FINE, BAD_LEVEL=255 };

    /** Convert from 'enum' (int) to 'string' and vice-versa */
    static const Level getLevelFromString(const std::string&);
    static const char* getLevelString(const Level);

    // ***********************************************/
    // Constructors/destructors

    /**
     * Messages will be appended to the passed file.
     *
     * @param fileName The log file
     * @param level Only messages with log-level less than or equal to
     *        level will be logged
     * @param timestampEnabled If true, a timestamp will be prefixed to
              every log message
     */
    Logger(const std::string &fileName = "opencog.log",
           Level level = INFO, bool timestampEnabled = true);

    Logger(const Logger&);

    ~Logger();
    Logger& operator=(const Logger& log);

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
     * Set the main logger to print only
     * error level log on stdout (useful when one is only interested
     * in printing cassert logs)
     */
    void setPrintErrorLevelStdout();

    /**
     * Log a message into log file (passed in constructor) if and only
     * if passed level is lower than or equal to the current log level
     * of this Logger instance.
     */
    void log  (Level level, const std::string &txt);
    // void error(const std::string &txt);
    // void warn (const std::string &txt);
    // void info (const std::string &txt);
    // void debug(const std::string &txt);
    // void fine (const std::string &txt);

    /**
     * Log a message (printf style) into log file (passed in constructor)
     * if and only if passed level is lower than or equals to the current
     * log level of this Logger instance.
     *
     * You may use these methods as any printf-style call, eg:
     * fine("Count = %d", count)
     */
    void logva(Level level, const char *, va_list args);
    void log  (Level level, const char *, ...);
    // void error(const char *, ...);
    // void warn (const char *, ...);
    // void info (const char *, ...);
    // void debug(const char *, ...);
    // void fine (const char *, ...);

    class Base
    {
    public:
        Base(const Base& b) : logger(b.logger), lvl(b.lvl) {}
        std::stringstream& operator<<(const std::string& s)
        {
            ss << s;
            return ss;
        }
        std::stringstream& operator<<(const char *s)
        {
            ss << s;
            return ss;
        }
        ~Base()
        {
            if (0 < ss.str().length())
                logger.log(lvl, ss.str());
        }
    protected:
        friend class Logger;
        Base(Logger& l, Level v) : logger(l), lvl(v) {}
    private:
        Logger& logger;
        std::stringstream ss;
        Level lvl;
    };

    class Error : public Base
    {
    public:
        void operator()(const std::string &txt) { logger.log(ERROR, txt); }
        void operator()(const char *, ...);
        Base operator()() { return *this; }
    protected:
        friend class Logger;
        Error(Logger& l) : Base(l, ERROR) {}
    };
    Error error;

    class Warn : public Base
    {
    public:
        void operator()(const std::string &txt) { logger.log(WARN, txt); }
        void operator()(const char *, ...);
        Base operator()() { return *this; }
    protected:
        friend class Logger;
        Warn(Logger& l) : Base(l, WARN) {}
    };
    Warn warn;

    class Info : public Base
    {
    public:
        void operator()(const std::string &txt) { logger.log(INFO, txt); }
        void operator()(const char *, ...);
        Base operator()() { return *this; }
    protected:
        friend class Logger;
        Info(Logger& l) : Base(l, INFO) {}
    };
    Info info;

    class Debug : public Base
    {
    public:
        void operator()(const std::string &txt) { logger.log(DEBUG, txt); }
        void operator()(const char *, ...);
        Base operator()() { return *this; }
    protected:
        friend class Logger;
        Debug(Logger& l) : Base(l, DEBUG) {}
    };
    Debug debug;

    class Fine : public Base
    {
    public:
        void operator()(const std::string &txt) { logger.log(FINE, txt); }
        void operator()(const char *, ...);
        Base operator()() { return *this; }
    protected:
        friend class Logger;
        Fine(Logger& l) : Base(l, FINE) {}
    };

    Fine fine;

public:
    /**
     * Methods to check if a given log level is enabled. This is useful for
     * avoiding unnecessary code for logger. For example:
     * if (isDebugEnabled())  debug(...);
     */
    bool isEnabled(Level level) const { return level <= currentLevel; }
    bool isErrorEnabled() const { return ERROR <= currentLevel; }
    bool isWarnEnabled() const { return WARN <= currentLevel; }
    bool isInfoEnabled() const { return INFO <= currentLevel; }
    bool isDebugEnabled() const { return DEBUG <= currentLevel; }
    bool isFineEnabled() const { return FINE <= currentLevel; } 

    /**
     * Block until all messages have been written out.
     */
    void flush();

private:

    std::string fileName;
    bool timestampEnabled;
    Level currentLevel;
    Level backTraceLevel;
    bool logEnabled;
    bool printToStdout;
    FILE *f;

    /** One single thread does all writing of log messages */
    std::thread writer_thread;
    std::mutex the_mutex;

    /** Queue for log messages */
    concurrent_queue< std::string* > msg_queue;
    bool pending_write;

    void startWriteLoop();
    void stopWriteLoop();
    void writingLoop();
    void writeMsg(std::string &msg);

    /**
     * Enable logging messages.
     */
    void enable();

    /**
     * Disable logging messages.
     */
    void disable();

}; // class

// singleton instance (following Meyer's design pattern)
Logger& logger();

/** @}*/
}  // namespace opencog

#endif // _OPENCOG_LOGGER_H
