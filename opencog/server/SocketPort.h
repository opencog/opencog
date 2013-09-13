/*
 * opencog/server/SocketPort.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Luigi <welter@vettalabs.com>
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

#ifndef _OPENCOG_SOCKET_PORT_H
#define _OPENCOG_SOCKET_PORT_H

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class stores a socket port
 */
class SocketPort
{

private:

    int port;

public:

    SocketPort(int _port) : port(_port) {}
    const int getPort() { return port; }

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_SOCKET_PORT_H

