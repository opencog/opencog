/*
 * opencog/embodiment/Control/MessagingSystem/MessagingSystemExceptions.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Thiago Maia, Andre Senna, Andre Senna
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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "MessagingSystemExceptions.h"

namespace opencog { namespace messaging {

CantBindToPortException::CantBindToPortException(int port)
{
    fprintf(stderr, "Can't bind to port %d\n", port);
    fflush(stdout);
}

BrokedPortBindingException::BrokedPortBindingException(int port)
{
    fprintf(stderr, "Binding to port %d is broken\n", port);
    fflush(stdout);
}

} } // namespace opencog::messaging
