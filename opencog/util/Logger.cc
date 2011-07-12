/*
 * opencog/util/Logger.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * Copyright (C) 2009, 2011 Linas Vepstas
 * Copyright (C) 2010 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
 *            Joel Pitt <joel@opencog.org>
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

#include "Logger.h"
#include "Config.h"

#ifndef WIN32
#include <cxxabi.h>
#include <execinfo.h>
#endif

#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#ifdef WIN32_NOT_UNIX
#include <winsock2.h>
#undef ERROR
#undef DEBUG
#else
#include <sys/time.h>
#endif

#include <boost/algorithm/string.hpp>

#include <opencog/util/platform.h>

using namespace opencog;

// messages greater than this will be truncated
#define MAX_PRINTF_STYLE_MESSAGE_SIZE (1<<15)
const char* levelStrings[] = {"NONE", "ERROR", "WARN", "INFO", "DEBUG", "FINE"};

Logger::~Logger()
{
    if (f != NULL) fclose(f);
}

Logger::Logger(const std::string &fileName, Logger::Level level, bool timestampEnabled)
{
    this->fileName.assign(fileName);
    this->currentLevel = level;
    this->backTraceLevel = getLevelFromString(opencog::config()["BACK_TRACE_LOG_LEVEL"]); 
    this->timestampEnabled = timestampEnabled;
    this->printToStdout = false;

    this->logEnabled = true;
    this->f = NULL;

    pthread_mutex_init(&lock, NULL);
}

Logger::Logger(const Logger& log) {
    set(log);
}

void Logger::set(const Logger& log) {
    this->fileName.assign(log.fileName);
    this->currentLevel = log.currentLevel;
    this->backTraceLevel = log.backTraceLevel;
    this->timestampEnabled = log.timestampEnabled;
    this->printToStdout = log.printToStdout;

    this->logEnabled = log.logEnabled;
    this->f = log.f;

    this->lock = log.lock; //Nil: I'm not sure about that    
}

// ***********************************************/
// API

void Logger::setLevel(Logger::Level newLevel)
{
    currentLevel = newLevel;
}

Logger::Level Logger::getLevel() const
{
    return currentLevel;
}

void Logger::setBackTraceLevel(Logger::Level newLevel)
{
    backTraceLevel = newLevel;
}

Logger::Level Logger::getBackTraceLevel() const
{
    return backTraceLevel;
}

void Logger::setFilename(const std::string& s)
{
    fileName.assign(s);
    if (f != NULL) fclose(f);
    f = NULL;
    enable();
}

const std::string& Logger::getFilename()
{
    return fileName;
}

void Logger::setTimestampFlag(bool flag)
{
    timestampEnabled = flag;
}

void Logger::setPrintToStdoutFlag(bool flag)
{
    printToStdout = flag;
}

void Logger::setPrintErrorLevelStdout() {
    setPrintToStdoutFlag(true);
    setLevel(Logger::ERROR);
}

void Logger::enable()
{
    logEnabled = true;
}

void Logger::disable()
{
    logEnabled = false;
}

#ifndef WIN32 /// @todo backtrace and backtrace_symbols is UNIX, we
              /// may need a WIN32 version
static void prt_backtrace(FILE *fh)
{
#define BT_BUFSZ 50
	void *bt_buf[BT_BUFSZ];

	int stack_depth = backtrace(bt_buf, BT_BUFSZ);
	char **syms = backtrace_symbols(bt_buf, stack_depth);

	// Start printing at a bit into the stack, so as to avoid recording
	// the logger functions in the stack trace.
	fprintf(fh, "\tStack Trace:\n");
	for (int i=2; i< stack_depth; i++)
	{
		// Most things we'll print are mangled C++ names,
		// So demangle them, get them to pretty-print.
		char * begin = strchr(syms[i], '(');
		char * end = strchr(syms[i], '+');
		if (!(begin && end) || end <= begin)
		{
			// Failed to pull apart the symbol names
			fprintf(fh, "\t%d: %s\n", i, syms[i]);
		}
		else
		{
			*begin = 0x0;
			fprintf(fh, "\t%d: %s ", i, syms[i]);
			*begin = '(';
			size_t sz = 250;
			int status;
			char *fname = (char *) malloc(sz);
			*end = 0x0;
			char *rv = abi::__cxa_demangle(begin+1, fname, &sz, &status);
			*end = '+';
			if (rv) fname = rv; // might have re-alloced
			fprintf(fh, "(%s ", fname);
			free(fname);
			fprintf(fh, "%s\n", end);
		}
	}
	fprintf(fh, "\n");
	free(syms);
}
#endif

void Logger::log(Logger::Level level, const std::string &txt)
{
    if (!logEnabled) return;
    // delay opening the file until the first logging statement is issued;
    // this allows us to set the main logger's filename without creating
    // a useless log file with the default filename
    if (f == NULL) {
        if ((f = fopen(fileName.c_str(), "a")) == NULL) {
            fprintf(stderr, "[ERROR] Unable to open log file \"%s\"\n", fileName.c_str());
            disable();
            return;
        } else enable();
    }

    if (level <= currentLevel) {
        pthread_mutex_lock(&lock);
        if (timestampEnabled) {
            char timestamp[64];
            struct timeval stv;
            struct tm stm;

            ::gettimeofday(&stv, NULL);
            time_t t = stv.tv_sec;
            gmtime_r(&t, &stm);
            strftime(timestamp, sizeof(timestamp), "%F %T", &stm);
            fprintf(f, "[%s:%03ld] ", timestamp, stv.tv_usec / 1000);
            if (printToStdout) fprintf(stdout, "[%s:%03ld] ", timestamp, stv.tv_usec / 1000);
        }

        fprintf(f, "[%s] %s\n", getLevelString(level), txt.c_str());
        if (printToStdout) fprintf(stdout, "[%s] %s\n", getLevelString(level), txt.c_str());
        fflush(f);

        if (level <= backTraceLevel)
        {
#ifndef WIN32
            prt_backtrace(f);
#endif
        }
        fflush(f);
        pthread_mutex_unlock(&lock);
    }
}
void Logger::error(const std::string &txt)
{
    log(ERROR, txt);
}
void Logger::warn (const std::string &txt)
{
    log(WARN,  txt);
}
void Logger::info (const std::string &txt)
{
    log(INFO,  txt);
}
void Logger::debug(const std::string &txt)
{
    log(DEBUG, txt);
}
void Logger::fine (const std::string &txt)
{
    log(FINE,  txt);
}

void Logger::logva(Logger::Level level, const char *fmt, va_list args)
{
    if (level <= currentLevel) {
        char buffer[MAX_PRINTF_STYLE_MESSAGE_SIZE];
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        std::string msg = buffer;
        log(level, msg);
    }
}

void Logger::log(Logger::Level level, const char *fmt, ...)
{
    va_list args; va_start(args, fmt); logva(level, fmt, args); va_end(args);
}
void Logger::error(const char *fmt, ...)
{
    va_list args; va_start(args, fmt); logva(ERROR, fmt, args); va_end(args);
}
void Logger::warn (const char *fmt, ...)
{
    va_list args; va_start(args, fmt); logva(WARN,  fmt, args); va_end(args);
}
void Logger::info (const char *fmt, ...)
{
    va_list args; va_start(args, fmt); logva(INFO,  fmt, args); va_end(args);
}
void Logger::debug(const char *fmt, ...)
{
    va_list args; va_start(args, fmt); logva(DEBUG, fmt, args); va_end(args);
}
void Logger::fine (const char *fmt, ...)
{
    va_list args; va_start(args, fmt); logva(FINE,  fmt, args); va_end(args);
}

bool Logger::isEnabled(Level level) const
{
    return level <= currentLevel;
}

bool Logger::isErrorEnabled() const
{
    return ERROR <= currentLevel;
}

bool Logger::isWarnEnabled() const
{
    return WARN <= currentLevel;
}

bool Logger::isInfoEnabled() const
{
    return INFO <= currentLevel;
}

bool Logger::isDebugEnabled() const
{
    return DEBUG <= currentLevel;
}

bool Logger::isFineEnabled() const
{
    return FINE <= currentLevel;
}


const char* Logger::getLevelString(const Logger::Level level)
{
    if (level == BAD_LEVEL)
        return "Bad level";
    else 
        return levelStrings[level];
}

const Logger::Level Logger::getLevelFromString(const std::string& levelStr)
{
    unsigned int nLevels = sizeof(levelStrings) / sizeof(levelStrings[0]);
    for (unsigned int i = 0; i < nLevels; ++i) {
        if (boost::iequals(levelStrings[i], levelStr))
            return ((Logger::Level) i);
    }
    return ((Logger::Level) BAD_LEVEL);
}

// create and return the single instance
Logger& opencog::logger()
{
    static Logger instance;
    return instance;
}
