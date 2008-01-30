/**
 **	\file HttpResponse.h
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
#ifndef _SOCKETS_HttpResponse_H
#define _SOCKETS_HttpResponse_H

#include "HttpTransaction.h"
#include <list>

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class IFile;

class HttpResponse : public HttpTransaction
{
public:
	HttpResponse(const std::string& version = "HTTP/1.0");
	~HttpResponse();

	/** HTTP/1.x */
	void SetHttpVersion(const std::string& value);
	const std::string& HttpVersion() const;

	void SetHttpStatusCode(int value);
	int HttpStatusCode() const;

	void SetHttpStatusMsg(const std::string& value);
	const std::string& HttpStatusMsg() const;

	void SetCookie(const std::string& value);
	const std::string Cookie(const std::string& name) const;
	std::list<std::string> CookieNames() const;

	void Write( const std::string& str );
	void Write( const char *buf, size_t sz );
	void Writef( const char *format, ... );

	const IFile& GetFile() const { return *m_file; }

	/** Replace memfile with file on disk, opened for read. */
	void SetFile( const std::string& path );

	void Reset();

private:
	std::string m_http_version;
	int m_http_status_code;
	std::string m_http_status_msg;
	std::map<std::string, std::string> m_cookie;
	IFile *m_file;

}; // end of class


#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_HttpResponse_H

