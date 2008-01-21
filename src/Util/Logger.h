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

// Macro for the main logger, which is a static member of this class
#define MAIN_LOGGER Util::Logger::getMainLogger()

namespace Util {

class Logger {

    private:

        std::string fileName;
        bool timestampEnabled;
        int currentLevel;
        bool logEnabled;
        bool printToStdout;
        pthread_mutex_t lock;
        
        static Logger* mainLogger;
		FILE *f;
		
    public:

        static const int ERROR = 0;
        static const int WARNING = 1;
        static const int INFO = 2;
        static const int DEBUG = 3;
        static const int FINE = 4;

        /**
         * Initializes the main logger (this will be the main logger of each process) with specific arguments. 
         * For each program/process, this method should be called before the first call to the getMainLogger() method, 
         * so that it uses the given specific logger.
         */
        static void initMainLogger(Logger* specificLogger);

        /**
         * Releases the main logger if it is already allocated/initialized
         * Processes must call this at the end of thep program
         **/ 
        static void releaseMainLogger();

        /**
         * Gets the main logger (this will be the main logger of each process)
         * The initMainLogger() method should be called before the first call to this method
         * so that it can be initialized with specific arguments for each process/program.
         */
        static Logger& getMainLogger();

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
        Logger(const std::string &fileName = "logFile.txt", int level = FINE, bool timestampEnabled = true);

        // ***********************************************/
        // API

        /**
         * Reset the level of messages which will be logged. Every message with log-level
         * lower than or equals to newLevel will be logged.
         */
        void setLevel(int newLevel);

        /**
         * Get the current log level that determines which messages will be logged: Every message with log-level
         * lower than or equals to returned level will be logged.
         */
        int getLevel() const;

        /**
         * Reset the flag that indicates whether a timestamp is to be prefixed in every message or not.
         */
        void setTimestampFlag(bool flag);

        /**
         * Reset the flag that indicates whether the log messages should be printed to the stdout or not.
         */
        void setPrintToStdoutFlag(bool flag);

        /**
         * Logs a message into log file (passed in constructor) if and only if passed level is
         * lower than or equals to the current log level of Logger.
         */
        void log(int level, const std::string &txt);

        /**
         * Logs a message (printf style) into log file (passed in constructor) if and only if passed level is
         * lower than or equals to the current log level of Logger.
         *
         * You may use this method as any printf-style call, eg: log(FINE, "Count = %d", count)
         */
        void log(int level, const char *, ...);

        /**
         * Enable logging messages.
         */
        void enable();

        /**
         * Disable logging messages.
         */
        void disable();

}; // class
}  // namespace

#endif
