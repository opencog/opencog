/**
 **	\file HttpBaseSocket.h
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
#ifndef _HttpBaseSocket_H
#define _HttpBaseSocket_H

#include "HTTPSocket.h"
#include "HttpRequest.h"
#include "IHttpServer.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class HttpResponse;

class HttpBaseSocket : public HTTPSocket, public IHttpServer
{
public:
	HttpBaseSocket(ISocketHandler& h);
	~HttpBaseSocket();

	void OnFirst();
	void OnHeader(const std::string& key,const std::string& value);
	void OnHeaderComplete();
	void OnData(const char *,size_t);

	// implements IHttpServer::Respond
	void Respond(const HttpResponse& res);

	void OnTransferLimit();

protected:
	HttpBaseSocket(const HttpBaseSocket& s) : HTTPSocket(s) {} // copy constructor
	//
	HttpRequest m_req;
	void Reset();

private:
	HttpBaseSocket& operator=(const HttpBaseSocket& ) { return *this; } // assignment operator
	void Execute();
	//
	size_t m_body_size_left;
	const IFile *m_res_file;
	bool m_b_keepalive;
};




#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _HttpBaseSocket_H

