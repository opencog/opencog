/** \file HttpdSocket.h
*/
/*
Copyright (C) 2001-2007  Anders Hedstrom (grymse@alhem.net)

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
#ifndef _SOCKETS_HttpdSocket_H
#define _SOCKETS_HttpdSocket_H

#include "sockets-config.h"
#include "HTTPSocket.h"
#include "ISocketHandler.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class HttpdCookies;
class HttpdForm;
class IFile;

/** \defgroup webserver Webserver framework */
/** Web server socket framework. 
	\ingroup webserver */
class HttpdSocket : public HTTPSocket
{
public:
	HttpdSocket(ISocketHandler& );
	~HttpdSocket();

	void OnFirst();
	void OnHeader(const std::string& key,const std::string& value);
	void OnHeaderComplete();
	void OnData(const char *,size_t);

	/** This method needs to be implemented with logic to produce
		a response to an incoming request. */
	virtual void Exec() = 0;
	/** Get current date in http rfc format. */
	const std::string& GetHttpDate() const;
	/** Get pointer to cookie class. */
	HttpdCookies *GetCookies();
	/** Get pointer to query string/form data class. */
	const HttpdForm *GetForm() const;

	size_t ContentLength() const { return m_content_length; }
	const IFile *Body() const { return m_file; }
	int RequestId() const { return m_request_id; }

protected:
	HttpdSocket(const HttpdSocket& s) : HTTPSocket(s) {}
	/** Decode and send a base64-encoded string. 
		\param str64 Base64-encoded string
		\param type Mime type of content (content-type header) */
	void Send64(const std::string& str64, const std::string& type);
	std::string datetime2httpdate(const std::string& dt);
	std::string GetDate();
	void Reset();
	// headers
	std::string m_http_cookie;
	std::string m_content_type;
	std::string m_content_length_str;
	std::string m_if_modified_since;

private:
	HttpdSocket& operator=(const HttpdSocket& s) { return *this; }
static	int m_request_count;
static	std::string m_start;
	size_t m_content_length;
	IFile *m_file;
	size_t m_received;
	int m_request_id;
	std::string m_http_date;
	HttpdCookies *m_cookies;
	HttpdForm *m_form;
};


#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_HttpdSocket_H

