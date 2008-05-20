/**
 * Logger.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Thu Jul  5 20:56:04 BRT 2007
 */
#ifdef DEBUG
#undef DEBUG
#endif

#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <pthread.h>

namespace opencog {

class Logger {

public:

    enum Level { ERROR, WARN, INFO, DEBUG, FINE };

    /** Convert from 'enum' (int) to 'string' and vice-versa */
    static const Level getLevelFromString(const std::string&);
    static const char* getLevelString(const Level);


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
     * You may use this method as any printf-style call, eg: log(FINE, "Count = %d", count)
     */
    void logva(Level level, const char *, va_list args);
    void log  (Level level, const char *, ...);
    void error(const char *, ...);
    void warn (const char *, ...);
    void info (const char *, ...);
    void debug(const char *, ...);
    void fine (const char *, ...);

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
    bool logEnabled;
    bool printToStdout;
    pthread_mutex_t lock;
    FILE *f;
 
}; // class

// singleton instance (following meyer's design pattern)
Logger& logger();

}  // namespace opencog

#endif
