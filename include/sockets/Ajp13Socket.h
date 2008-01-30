/**
 **	\file Ajp13Socket.h
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
#ifndef _SOCKETS_Ajp13Socket_H
#define _SOCKETS_Ajp13Socket_H

#include "AjpBaseSocket.h"
#include "HttpRequest.h"
#include "IHttpServer.h"

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class HttpResponse;

class Ajp13Socket : public AjpBaseSocket, public IHttpServer
{
public:
	Ajp13Socket(ISocketHandler& h);

	void OnHeader( short id, short len );
	void OnPacket( const char *buf, size_t sz );

	// implements IHttpServer::Respond
	void Respond(const HttpResponse& res);

	void OnTransferLimit();

private:
	void ReceiveBody( const char *buf, size_t sz );
	void ReceiveForwardRequest( const char *buf, size_t sz );
	void ReceiveShutdown( const char *buf, size_t sz );
	void ReceivePing( const char *buf, size_t sz );
	void ReceiveCPing( const char *buf, size_t sz );
	void Execute();
	//
	size_t m_body_size_left;
	HttpRequest m_req;
	const IFile *m_res_file;
};


#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_Ajp13Socket_H

