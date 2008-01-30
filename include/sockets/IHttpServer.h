/**
 **	\file IHttpServer.h
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
#ifndef _SOCKETS_IHttpServer_H
#define _SOCKETS_IHttpServer_H

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class HttpRequest;
class HttpResponse;

class IHttpServer
{
public:
	virtual ~IHttpServer() {}

	/** Complete request has been received and parsed. Send response
	    using the Respond() method. */
	virtual void OnExec(const HttpRequest& req) = 0;

	/** Send response. */
	virtual void Respond(const HttpResponse& res) = 0;

};




#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_IHttpServer_H

