/** \file HttpPostSocket.h
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
#ifndef _SOCKETS_HttpPostSocket_H
#define _SOCKETS_HttpPostSocket_H

#include "sockets-config.h"
#include "HttpClientSocket.h"
#include "Mutex.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class ISocketHandler;

/** Generate a http post request, get response. 
	\ingroup http */
class HttpPostSocket : public HttpClientSocket
{
public:
	HttpPostSocket(ISocketHandler&);
	/* client constructor, 
		\param url_in = 'http://host:port/resource' */
	HttpPostSocket(ISocketHandler&,const std::string& url_in);
	~HttpPostSocket();

	// these must be specified before connecting / adding to handler
	/** Add field to post. */
	void AddField(const std::string& name,const std::string& value);
	/** Add multiline field to post. */
	void AddMultilineField(const std::string& name,std::list<std::string>& values);
	/** Add file to post. */
	void AddFile(const std::string& name,const std::string& filename,const std::string& type);

	/** use this to post with content-type multipart/form-data.
	// when adding a file to the post, this is the default and only content-type */
	void SetMultipart();

	/** connect to host:port derived from url in constructor */
	void Open();

	/** http put client implemented in OnConnect */
	void OnConnect();

private:
	HttpPostSocket(const HttpPostSocket& s) : HttpClientSocket(s) {} // copy constructor
	HttpPostSocket& operator=(const HttpPostSocket& ) { return *this; } // assignment operator
	void DoMultipartPost();
	//
	std::map<std::string,std::list<std::string> > m_fields;
	std::map<std::string,std::string> m_files;
	std::string m_boundary;
	std::map<std::string,long> m_content_length;
	std::map<std::string,std::string> m_content_type;
	bool m_bMultipart;
static	int m_boundary_count;
static	Mutex m_boundary_mutex;
};


#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_HttpPostSocket_H

