/**
 * MessagingSystemExceptions.h
 *
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#ifndef MESSAGINGSYSTEMEXCEPTIONS_H
#define MESSAGINGSYSTEMEXCEPTIONS_H

#include <stdarg.h>

namespace MessagingSystem {

/**
 * Thrown when a listener unsucessfully tries to bind to a given port 
 */
class CantBindToPortException {

    public:

        /**
         * Thrown when a listener unsucessfully tries to bind to a given port 
         *
         * @param Port number the listener tried to bind to.
         */
        CantBindToPortException(int port);
};

/**
 * Thrown when a listener sucessfully binded to a given port but this binding was broken afterwards for some reason.
 */
class BrokedPortBindingException {

    public:

        /**
         * Thrown when a listener sucessfully binded to a given port but this binding was broken afterwards for some reason.
         *
         * @param Port number the listener tried to bind to.
         */
        BrokedPortBindingException(int port);
};

} // namespace

#endif
