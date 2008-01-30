/**
 **	\file HttpRequest.h
 **	\date  2007-10-05
 **	\author grymse@alhem.net
**/
/*
Copyright (C) 2007  Anders Hedstrom

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
#ifndef _SOCKETS_HttpRequest_H
#define _SOCKETS_HttpRequest_H

#include "HttpTransaction.h"
#include "HttpdCookies.h"
#include <memory>
#include "IFile.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class HttpdForm;
class IFile;

class HttpRequest : public HttpTransaction
{
public:
	HttpRequest();
	HttpRequest(const HttpRequest& src);
	~HttpRequest();

	/** Get, Post */
	void SetHttpMethod(const std::string& value);
	const std::string& HttpMethod() const;

	/** HTTP/1.x */
	void SetHttpVersion(const std::string& value);
	const std::string& HttpVersion() const;

	void SetUri(const std::string& value);
	const std::string& Uri() const;

	void SetRemoteAddr(const std::string& value);
	const std::string& RemoteAddr() const;

	void SetRemoteHost(const std::string& value);
	const std::string& RemoteHost() const;

	void SetServerName(const std::string& value);
	const std::string& ServerName() const;

	void SetServerPort(int value);
	int ServerPort() const;

	void SetIsSsl(bool value);
	bool IsSsl() const;

	/** Set / Read attribute value */
	void SetAttribute(const std::string& key, const std::string& value);
	void SetAttribute(const std::string& key, long value);
	const std::string& Attribute(const std::string& key) const;

	const std::map<std::string, std::string>& Attributes() const;

	/** Cookies */
	void AddCookie(const std::string& );
	const std::map<std::string, std::string>& CookieMap() const { return m_cookie; }

	/** Open file for body data */
	void InitBody( size_t sz );

	/** Write body data */
	void Write( const char *buf, size_t sz );

	/** No more writing */
	void CloseBody();

	void ParseBody();

	const HttpdForm& Form() const;
	const HttpdCookies& Cookies() const;

	const IFile *BodyFile() const { return m_body_file.get(); }

	void Reset();

private:
	std::string m_method;
	std::string m_protocol;
	std::string m_req_uri;
	std::string m_remote_addr;
	std::string m_remote_host;
	std::string m_server_name;
	int m_server_port;
	bool m_is_ssl;
	std::map<std::string, std::string> m_attribute;
	std::string m_null;
	mutable std::auto_ptr<IFile> m_body_file;
	mutable std::auto_ptr<HttpdForm> m_form;
	HttpdCookies m_cookies;
	std::map<std::string, std::string> m_cookie;

}; // end of class


#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_HttpRequest_H

