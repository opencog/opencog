/**
 * exceptions.cc
 *
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "exceptions.h"
#include "Logger.h"

/**
 * ----------------------------------------------------------------------------
 * StandardException class
 * ----------------------------------------------------------------------------
 */
void StandardException::parseErrorMessage(const char* fmt, va_list ap)
{
    char    buf[1024];

    vsnprintf(buf, sizeof(buf), fmt, ap);
    MAIN_LOGGER.log(Util::Logger::ERROR, buf);

    setMessage(buf);
}

void StandardException::parseErrorMessage(const char * trace, const char * msg, va_list ap)
{
    char * concatMsg = new char[strlen(msg) + strlen(trace) + 1];
    *concatMsg = '\0'; // empty c-string

    strcat(concatMsg, msg);
    strcat(concatMsg, trace);

    parseErrorMessage(concatMsg, ap);

    delete [] concatMsg;
}

StandardException::StandardException()
{
    message = NULL;
}

StandardException::~StandardException()
{
    // clear memory
    if(message != NULL){
        delete [] message;
    }
}

const char * StandardException::getMessage()
{
    if(message == NULL){
        return "";
    }
    return message;
}

void StandardException::setMessage(const char * msg)
{
    // clear msg
    if(message != NULL){
        delete [] message;
    }

    message = new char[strlen(msg) + 1];
    strcpy(message, msg);
}

/**
 * ----------------------------------------------------------------------------
 * RuntimeException class
 * ----------------------------------------------------------------------------
 */
RuntimeException::RuntimeException(const char *trace, const char* fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

RuntimeException::RuntimeException() {
}

RuntimeException::~RuntimeException() {
}

/**
 * ----------------------------------------------------------------------------
 * XMLException class
 * ----------------------------------------------------------------------------
 */
XMLException::XMLException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * IOException class
 * ----------------------------------------------------------------------------
 */
IOException::IOException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * ComboException class
 * ----------------------------------------------------------------------------
 */
ComboException::ComboException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * IndexErrorException class
 * ----------------------------------------------------------------------------
 */
IndexErrorException::IndexErrorException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * InvalidException class
 * ----------------------------------------------------------------------------
 */
InvalidParamException::InvalidParamException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * InconsistenceException class
 * ----------------------------------------------------------------------------
 */
InconsistenceException::InconsistenceException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * FatalErrorException class
 * ----------------------------------------------------------------------------
 */
FatalErrorException::FatalErrorException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * NetworkException class
 * ----------------------------------------------------------------------------
 */
NetworkException::NetworkException(const char * trace, const char * fmt, ...)
{
    va_list  ap;
    va_start(ap, fmt);
    parseErrorMessage(trace, fmt, ap);
    va_end(ap);
}

/**
 * ----------------------------------------------------------------------------
 * AssertionException class
 * ----------------------------------------------------------------------------
 */
AssertionException::AssertionException(const char* fmt, ...)
{
    char    buf[1024];

    va_list        ap;
    va_start(ap, fmt);

    vsnprintf(buf, sizeof(buf), fmt, ap);
    MAIN_LOGGER.log(Util::Logger::ERROR, buf);
    va_end(ap);
}

AssertionException::AssertionException(const char* fmt, va_list ap)
{
    char    buf[1024];

    vsnprintf(buf, sizeof(buf), fmt, ap);
    MAIN_LOGGER.log(Util::Logger::ERROR, buf);
}

void cassert(const char * trace, bool condition, const char *fmt, ...)
{
    if (condition) return;

    va_list        ap;
    va_start(ap, fmt);

    char * c_msg = new char[strlen(fmt) + strlen(trace) + 1];
    *c_msg = '\0'; // empty c-string

    strcat(c_msg, fmt);
    strcat(c_msg, trace);

    AssertionException ex = AssertionException(c_msg, ap);
    va_end(ap);

    delete [] c_msg;
    throw ex;
}

void cassert(const char * trace, bool condition)
{

    if (condition) return;

    AssertionException ex = AssertionException(trace);
    throw ex;
}

