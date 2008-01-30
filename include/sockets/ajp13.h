/**
 **	\file ajp13.h
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
#ifndef _SOCKETS_Ajp13_H
#define _SOCKETS_Ajp13_H

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif

#define HTTP_REQUEST_ACCEPT 		0xa001
#define HTTP_REQUEST_ACCEPT_CHARSET 	0xa002
#define HTTP_REQUEST_ACCEPT_ENCODING 	0xa003
#define HTTP_REQUEST_ACCEPT_LANGUAGE 	0xa004
#define HTTP_REQUEST_AUTHORIZATION 	0xa005
#define HTTP_REQUEST_CONNECTION 	0xa006
#define HTTP_REQUEST_CONTENT_TYPE 	0xa007
#define HTTP_REQUEST_CONTENT_LENGTH 	0xa008
#define HTTP_REQUEST_COOKIE 		0xa009
#define HTTP_REQUEST_COOKIE2 		0xa00a
#define HTTP_REQUEST_HOST 		0xa00b
#define HTTP_REQUEST_PRAGMA 		0xa00c
#define HTTP_REQUEST_REFERER 		0xa00d
#define HTTP_REQUEST_USER_AGENT 	0xa00e

#define HTTP_METHOD_OPTIONS		1
#define HTTP_METHOD_GET			2
#define HTTP_METHOD_HEAD		3
#define HTTP_METHOD_POST		4
#define HTTP_METHOD_PUT			5
#define HTTP_METHOD_DELETE		6
#define HTTP_METHOD_TRACE		7
#define HTTP_METHOD_PROPFIND		8
#define HTTP_METHOD_PROPPATCH		9
#define HTTP_METHOD_MKCOL		10
#define HTTP_METHOD_COPY		11
#define HTTP_METHOD_MOVE		12
#define HTTP_METHOD_LOCK		13
#define HTTP_METHOD_UNLOCK		14
#define HTTP_METHOD_ACL			15
#define HTTP_METHOD_REPORT		16
#define HTTP_METHOD_VERSION_CONTROL	17 // with a dash "VERSION-CONTROL"
#define HTTP_METHOD_CHECKIN		18
#define HTTP_METHOD_CHECKOUT		19
#define HTTP_METHOD_UNCHECKOUT		20
#define HTTP_METHOD_SEARCH		21
#define HTTP_METHOD_MKWORKSPACE		22
#define HTTP_METHOD_UPDATE		23
#define HTTP_METHOD_LABEL		24
#define HTTP_METHOD_MERGE		25
#define HTTP_METHOD_BASELINE_CONTROL	26
#define HTTP_METHOD_MKACTIVITY		27

#define ATTR_CONTEXT       1
#define ATTR_SERVLET_PATH  2
#define ATTR_REMOTE_USER   3
#define ATTR_AUTH_TYPE     4
#define ATTR_QUERY_STRING  5
#define ATTR_ROUTE         6
#define ATTR_SSL_CERT      7
#define ATTR_SSL_CIPHER    8
#define ATTR_SSL_SESSION   9
#define ATTR_SSL_KEY_SIZE  11
#define ATTR_SECRET        12
#define ATTR_STORED_METHOD 13

#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_Ajp13_H

