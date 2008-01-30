/** \file HttpDebugSocket.h
 **	\date  2004-09-27
**/
/*
Copyright (C) 2004-2007  Anders Hedström (grymse@alhem.net)

This library is made available under the terms of the GNU GPL.

If you would like to use this library in a closed-source application,
a separate license agreement is available. For information about 
the closed-source license agreement for the C++ sockets library,
please visit http://www.alhem.net/Sockets/license.html and/or
email license@alhem.net.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/
#ifndef _SOCKETS_HttpDebugSocket_H
#define _SOCKETS_HttpDebugSocket_H

#include "sockets-config.h"
#include "HTTPSocket.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif

class ISocketHandler;

/** HTTP request "echo" class. This class echoes a http request/body
with a html formatted page. 
	\ingroup http */
class HttpDebugSocket : public HTTPSocket
{
public:
	HttpDebugSocket(ISocketHandler&);
	~HttpDebugSocket();

	void Init();

	void OnFirst();
	void OnHeader(const std::string& key,const std::string& value);
	void OnHeaderComplete();
	void OnData(const char *,size_t);

private:
	HttpDebugSocket(const HttpDebugSocket& s) : HTTPSocket(s) {} // copy constructor
	HttpDebugSocket& operator=(const HttpDebugSocket& ) { return *this; } // assignment operator
	int m_content_length;
	int m_read_ptr;
};


#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_HttpDebugSocket_H

