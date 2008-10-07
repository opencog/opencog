/*
 * src/server/IRPCSocket.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_I_RPC_SOCKET_H
#define _OPENCOG_I_RPC_SOCKET_H

#include <string>

namespace opencog
{

/**
 * This class defines an interface that sockets compatible with opencog requests
 * must implement. It just defines an extra callback -- OnRequestComplete() --
 * which is used by the request to notify the client socket that it has
 * finished.
 *
 * This mechanism is required to enable the request to be processed in one thread
 * (the main cogserver thread) while the socket handling is handled by a
 * separate thread.
 */
class IRPCSocket
{

public:

    virtual ~IRPCSocket() {};
    virtual void OnRequestComplete() = 0;

}; // class

}  // namespace

#endif // _OPENCOG_I_RPC_SOCKET_H
