/*
 * opencog/embodiment/Control/MessagingSystem/MessagingSystemExceptions.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Thiago Maia, Andre Senna
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


#ifndef MESSAGINGSYSTEMEXCEPTIONS_H
#define MESSAGINGSYSTEMEXCEPTIONS_H

#include <stdarg.h>

namespace opencog { namespace messaging {

/**
 * Thrown when a listener unsucessfully tries to bind to a given port
 */
class CantBindToPortException
{

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
class BrokedPortBindingException
{

public:

    /**
     * Thrown when a listener sucessfully binded to a given port but this binding was broken afterwards for some reason.
     *
     * @param Port number the listener tried to bind to.
     */
    BrokedPortBindingException(int port);
};

} } // namespace opencog::messaging

#endif
