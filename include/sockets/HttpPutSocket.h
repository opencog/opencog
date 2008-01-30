/** \file HttpPutSocket.h
 **	\date  2004-10-30
 **	\author grymse@alhem.net
**/
/*
Copyright (C) 2004-2007  Anders Hedstrom

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
#ifndef _SOCKETS_HttpPutSocket_H
#define _SOCKETS_HttpPutSocket_H

#include "sockets-config.h"
#include "HttpClientSocket.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class ISocketHandler;

/** Put http page. 
	\ingroup http */
class HttpPutSocket : public HttpClientSocket
{
public:
	HttpPutSocket(ISocketHandler&);
	/** client constructor, 
		\param url_in = 'http://host:port/resource' */
	HttpPutSocket(ISocketHandler&,const std::string& url_in);
	~HttpPutSocket();

	// these must be specified before connecting / adding to handler
	/** Set filename to send. */
	void SetFile(const std::string& );
	/** Set mimetype of file to send. */
	void SetContentType(const std::string& );

	/** connect to host:port derived from url in constructor */
	void Open();

	/** http put client implemented in OnConnect */
	void OnConnect();

private:
	HttpPutSocket(const HttpPutSocket& s) : HttpClientSocket(s) {} // copy constructor
	HttpPutSocket& operator=(const HttpPutSocket& ) { return *this; } // assignment operator
	//
	std::string m_filename;
	std::string m_content_type;
	long m_content_length;
};




#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_HttpPutSocket_H

