/**
 * Logger.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Thu Jul  5 20:56:04 BRT 2007
 */

#include <stdarg.h>
#include "Logger.h"
#include <time.h>

using namespace Util;

Logger* Logger::mainLogger = NULL;

// messages greater than this will be truncated
#define MAX_PRINTF_STYLE_MESSAGE_SIZE (1<<15)

void Logger::initMainLogger(Logger* specificLogger) {
    if (mainLogger) {
        delete mainLogger;
    }
    mainLogger = specificLogger;
}

void Logger::releaseMainLogger() {
    if (mainLogger) {
        delete mainLogger;
        mainLogger = NULL;
    }
}

Logger& Logger::getMainLogger() {

    if (!mainLogger) {
        mainLogger = new Logger();
    }
    return *mainLogger; 
}

Logger::~Logger() {
	if (f != NULL)
        fclose(f);
}

Logger::Logger(const std::string &fileName, int level, bool timestampEnabled) {

    this->fileName.assign(fileName);
    this->currentLevel = level;
    this->timestampEnabled = timestampEnabled;
    this->printToStdout = false;

		pthread_mutex_init(&lock, NULL);

	    f = fopen(fileName.c_str(), "a");
	    if (f == NULL) {
	        disable();
	    } else {
	//        fclose(f);
	        enable();
	    }

}

// ***********************************************/
// API

void Logger::setLevel(int newLevel) {
    currentLevel = newLevel;
}

int Logger::getLevel() const {
    return currentLevel;
}

void Logger::setTimestampFlag(bool flag) {
    timestampEnabled = flag;
}

void Logger::setPrintToStdoutFlag(bool flag) {
    printToStdout = flag;
}

void Logger::enable() {
    logEnabled = true;
}

void Logger::disable() {
    logEnabled = false;
}

void Logger::log(int level, const std::string &txt) {



    if (logEnabled) {
        if (level <= currentLevel) {
            pthread_mutex_lock(&lock);
            
//            FILE *f = fopen(fileName.c_str(), "a");
            if (f == NULL) {
           		f = fopen(fileName.c_str(), "a");
            }
            if (timestampEnabled) {
                char timestamp[64]; //= (char *) malloc(64 * sizeof(char));
                time_t t = time(NULL);
				
				struct timespec ts;
                int mSecs = 0;
				if (0 == clock_gettime(CLOCK_REALTIME, &ts))
                	mSecs = ts.tv_nsec / 1000;

                //timestamp = 
                ctime_r(&t, (char *) timestamp);
//cut year
				timestamp[19] = '\0';
//                    if (timestamp[strlen(timestamp) - 1] == '\n') {
//                        timestamp[strlen(timestamp) - 1] = '\0';
//                    }
                int ret = fprintf(f, "%s.%06d: ", timestamp, mSecs);
          		if (ret < 0) {
          			if (f != NULL)
						fclose(f);
					f = fopen(fileName.c_str(), "a");
                    ret = fprintf(f, "%s.%06d: ", timestamp, mSecs);
              	}
  
                    if (printToStdout) fprintf(stdout, "%s: ", timestamp);
//                    free(timestamp);
            }
            switch(level){
                case ERROR:
                    fprintf(f, "ERROR - %s\n", txt.c_str());
                    if (printToStdout) fprintf(stdout, "ERROR - %s\n", txt.c_str());
                    fflush(f);
                    break;

                case WARNING:
                    fprintf(f, "WARNING - %s\n", txt.c_str());
                    if (printToStdout) fprintf(stdout, "WARNING - %s\n", txt.c_str());
                    fflush(f);
                    break;

                case INFO:
                    fprintf(f, "INFO - %s\n", txt.c_str());
                    if (printToStdout) fprintf(stdout, "INFO - %s\n", txt.c_str());
                    break;

                case DEBUG:
                    fprintf(f, "DEBUG - %s\n", txt.c_str());
                    if (printToStdout) fprintf(stdout, "DEBUG - %s\n", txt.c_str());
                    fflush(f);
                    break;

                case FINE:
                    fprintf(f, "FINE - %s\n", txt.c_str());
                    if (printToStdout) fprintf(stdout, "FINE - %s\n", txt.c_str());
                    break;

                default:
                    // should not get here!
                    break;
            }
            if (fileName == "logFile.txt") {
				
            	fclose(f);
            	f = NULL;
            }
            pthread_mutex_unlock(&lock);
        }
    }
}

void Logger::log(int level, const char *fmt, ...) {

    if (level <= currentLevel) {
	    va_list ap;
	    va_start(ap, fmt);
	
        char buffer[MAX_PRINTF_STYLE_MESSAGE_SIZE];
        vsnprintf(buffer, sizeof(buffer), fmt, ap);
        std::string msg = buffer;
        log(level, msg);
        va_end(ap);
    }
}
