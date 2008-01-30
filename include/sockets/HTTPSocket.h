/** \file HTTPSocket.h 	Class HTTPSocket definition.
 **	\date  2004-04-06
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
#ifndef _SOCKETS_HTTPSocket_H
#define _SOCKETS_HTTPSocket_H

#include "sockets-config.h"
#include <map>
#include "TcpSocket.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif

/** \defgroup http HTTP Sockets */
/** HTTP request/response base class. 
	\ingroup http */
class HTTPSocket : public TcpSocket
{
	/** map to hold http header values. */
	typedef std::map<std::string,std::string> string_m;
public:
	HTTPSocket(ISocketHandler& );
	~HTTPSocket();

	void OnRawData(const char *buf,size_t len);
	void OnLine(const std::string& line);

	/** Callback executes when first line has been received.
		GetMethod, GetUrl/GetUri, and GetHttpVersion are valid when this callback is executed. */
	virtual void OnFirst() = 0;
	/** For each header line this callback is executed.
		\param key Http header name
		\param value Http header value */
	virtual void OnHeader(const std::string& key,const std::string& value) = 0;
	/** Callback fires when all http headers have been received. */
	virtual void OnHeaderComplete() = 0;
	/** Chunk of http body data recevied. */
	virtual void OnData(const char *,size_t) = 0;

	/** Get http method from incoming request, ie GET/POST/PUT etc */
	const std::string& GetMethod();
	/** Set http method to be used in request. */
	void SetMethod(const std::string& x);
	/** Get url from request. */
	const std::string& GetUrl();
	/** Set url to be used in outgoing request. */
	void SetUrl(const std::string& x);
	/** Get part of url before '?' character. */
	const std::string& GetUri();
	/** Now why would I need this when there is a SetUrl method? */
	void SetUri(const std::string& x);
	/** Get part of url after '?' character. */
	const std::string& GetQueryString();
	/** Get http version from incoming request/response. */
	const std::string& GetHttpVersion();
	/** Get http status from incoming response. */
	const std::string& GetStatus();
	/** Get http statustext from incoming response. */
	const std::string& GetStatusText();
	/** Incoming header has been identified as a request (method url http_version\r\n). */
	bool IsRequest();
	/** Incoming header has been identified as a response (http_version status status_text\r\n). */
	bool IsResponse();
	/** Set http version to be used in outgoing request/response. */
	void SetHttpVersion(const std::string& x);
	/** Set http status for outgoing response. */
	void SetStatus(const std::string& x);
	/** Set http statustext for outgoing response. */
	void SetStatusText(const std::string& x);
	/** Add (and replace if exists) http header. */
	void AddResponseHeader(const std::string& x,const std::string& y);
	/** Add (and replace if exists) http header. */
	void AddResponseHeader(const std::string& x,const char *format, ...);
	/** Add http header. */
	void AppendResponseHeader(const std::string& x,const std::string& y);
	/** See if http header 'name' has been set. */
	bool ResponseHeaderIsSet(const std::string& name);
	/** Send response prepared with calls to methods SetHttpVersion, SetStatus, SetStatusText,
		and AddResponseHeader. */
	void SendResponse();
	/** Send request prepared with calls to methods SetMethod, SetUrl, SetHttpVersion,
		and AddResponseHeader. */
	void SendRequest();

	/** Implement this to return your own User-agent string. */
	virtual std::string MyUseragent();

	/** Parse url. If protocol is https, EnableSSL() will be called. */
	void url_this(const std::string& url_in,std::string& protocol,std::string& host,port_t& port,std::string& url,std::string& file);

protected:
	HTTPSocket(const HTTPSocket& s) : TcpSocket(s) {}
	/** Reset state of socket to sucessfully implement keep-alive. */
	virtual void Reset();

private:
	HTTPSocket& operator=(const HTTPSocket& ) { return *this; }
	bool m_first;
	bool m_header;
	std::string m_line;
	std::string m_method;
	std::string m_url;
	std::string m_uri;
	std::string m_query_string;
	std::string m_http_version;
	std::string m_status;
	std::string m_status_text;
	bool m_request;
	bool m_response;
	string_m m_response_header;
	size_t m_body_size_left;
	bool m_b_http_1_1;
	bool m_b_keepalive;
	std::list<std::pair<std::string, std::string> > m_response_header_append;
};




#ifdef SOCKETS_NAMESPACE
}
#endif

#endif // _SOCKETS_HTTPSocket_H

