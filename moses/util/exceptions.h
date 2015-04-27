/*
 * moses3/util/exceptions.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_EXCEPTIONS_H
#define _OPENCOG_EXCEPTIONS_H

#include <string>
#include <iostream>

#include <stdarg.h>
#include <string.h>
#include "macros.h"

namespace moses3
{
/** \addtogroup grp_cogutil
 *  @{
 */

/**
 * Base exception class from which all other exceptions should inheritates.
 */
class StandardException : public std::exception
{

private:
    /**
     * c-string error message
     */
    char * message;

protected:

    /**
     * Parse error message substituting scape characters like (%s, %d, etc)
     * with their corresponding values.
     */
    void parseErrorMessage(const char* fmt, va_list ap, bool logError=true);
    void parseErrorMessage(const char * trace, const char* fmt, va_list ap, bool logError=true);

public:

    /**
     * Construtor and destructor.
     */
    StandardException();
    StandardException(const StandardException&);
    StandardException& operator=(const StandardException&);
    virtual ~StandardException();
    virtual const char* what() const _GLIBCXX_USE_NOEXCEPT {
        return getMessage();
    }

    /**
     * Get error message.
     * @return A c-string representing the error message. If no message have
     * been created just return an empty string.
     */
    const char* getMessage() const;

    /**
     * Set the error message.
     * @param A c-string representing the error message. The caller is
     * responsable to free the memory allocated in the c-string parameter.
     */
    void setMessage(const char *);

}; // StandardException

/**
 * Generic exception to be called in runtime, whenever an unexpected condition
 * is detected.
 */
class RuntimeException : public StandardException
{

public:

    /**
     * Generic exception to be called in runtime, whenever an unexpected
     * condition is detected.
     *
     * @param Exception message in printf standard format.
     */
    RuntimeException(const char*, const char*, ...);

    /**
     * Default constructor used for inheritance
     */
    RuntimeException();

}; // RuntimeException

/**
 * Exception to be thrown when a XML operation (processing, creation) fails.
 */
class XMLException : public RuntimeException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    XMLException(const char*, const char*, ...);

}; // XMLException

/**
 * Exception to be thrown when an I/O operation (reading, writing, open) fails.
 */
class IOException : public RuntimeException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    IOException(const char*, const char*, ...);

}; // IOException

/**
 * Exception to be thrown when a Combo operation (parsing, executing) fails.
 */
class ComboException : public RuntimeException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    ComboException(const char*, const char*, ...);

}; // ComboException

/**
 * Exception to be thrown when an out of range index is used.
 */
class IndexErrorException : public RuntimeException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    IndexErrorException(const char*, const char*, ...);

}; // IndexErrorException

/**
 * Exception to be thrown when an invalid parameter is used within a function or
 * an object initalization.
 *
 * This exception will not log an error when throwed, because the error must be
 * handled inside the code
 */
class InvalidParamException : public RuntimeException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    InvalidParamException(const char*, const char*, ...);

}; // InvalidParamException

/**
 * Exception to be thrown when a consistency check (equal to, different,
 * etc.) fails.
 */
class InconsistenceException : public RuntimeException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    InconsistenceException(const char*, const char*, ...);

}; // InconsistenceException

/**
 * Exception to be called when an unrecoverable error has occured. When catching
 * such exception all state savings should be done.
 */
class FatalErrorException : public StandardException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    FatalErrorException(const char*, const char*, ...);

}; // FatalErrorException

/**
 * Exception to be called when the searched item was not found
 *
 * This exception will not log an error when thrown, because the
 * error must be handled inside the code
 */
class NotFoundException : public StandardException {
public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    NotFoundException(const char*, const char*, ...);

}; // NotFoundException

/**
 * Exception thrown when the DeleteLink executes.
 *
 * This exception will not log an error when thrown, because the
 * error must be handled inside the code
 */
class DeleteException : public StandardException {
public:

    /**
     * Constructor
     * Nothing to be logged; tis simply breaks us out of inner loops.
     */
    DeleteException(void);

}; // DeleteException

/**
 * Exception to be called when a network error  has occured. When catching
 * such exception all state savings should be done.
 */
class NetworkException : public StandardException
{

public:

    /**
     * Constructor
     *
     * @param Trace information (filename:line-number). Use TRACE_INFO
     * macro.
     * @param Exception message in printf standard format.
     */
    NetworkException(const char*, const char*, ...);

}; // NetworkException


/**
 * Exception to be called when an assertion fails to pass a cassert function.
 */
class AssertionException : public StandardException
{

public:

    AssertionException(const char*, ...);
    AssertionException(const char* fmt, va_list ap);
};

inline std::ostream& operator<<(std::ostream& out, const StandardException& ex)
{
    out << ex.what();
    return out;
}

/** @}*/
} // namespace moses3

#endif // _OPENCOG_EXCEPTIONS_H
